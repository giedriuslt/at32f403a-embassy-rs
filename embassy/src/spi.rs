//! SPI master, transmit-focused, for the AT32F403A's F1-style blocks.
//!
//! The panel transport of the KWS-X1 firmware needs three operations, all
//! provided as inherent async methods rather than the full `embedded-hal`
//! bus traits: byte-wise [`Spi::write`] for commands, [`Spi::flush`] for
//! transmit-idle fencing, and [`Spi::write_dma`] for pixel bursts through a
//! [`crate::dma::Channel`].
//!
//! The clock-divider encoding is the vendor's (`CTRL1.MDIV2_0` plus
//! `CTRL2.MDIV3` for /512 and /1024; `spi_init`): SCK = PCLK / 2^(encoding
//! + 1), with the high encoding bit split across the two registers. Modes
//! map to `CLKPOL`/`CLKPHA`, and frames are 8-bit MSB-first. Chip select is
//! the board's job.

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::dma::{self, Channel as DmaChannel, Direction, Request, Width};
use crate::pac::at32f403a::spi1::RegisterBlock;
use crate::pac::at32f403a::spi1::ctrl1::{
    CLKPHA_A as ClkphaA, CLKPOL_A as ClkpolA, FBN_A as FbnA, LTF_A as LtfA, MSTEN_A as MstenA,
    SpienwWO,
};
use crate::pac::at32f403a::{CRM, SPI1};

/// SPI clock mode (CPOL, CPHA).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

/// Master clock divider: SCK = PCLK / divider (vendor `SPI_MCLK_DIV_*`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Divider {
    Div2 = 0,
    Div4 = 1,
    Div8 = 2,
    Div16 = 3,
    Div32 = 4,
    Div64 = 5,
    Div128 = 6,
    Div256 = 7,
    Div512 = 8,
    Div1024 = 9,
}

impl Divider {
    pub const fn factor(self) -> u32 {
        1u32 << ((self as u8 as u32) + 1)
    }

    /// The fastest divider whose SCK stays within `max_hz`, or the slowest
    /// divider when even /1024 exceeds it.
    pub const fn for_hz(pclk_hz: u32, max_hz: u32) -> Self {
        let mut encoding = 0;
        while encoding < 9 {
            let factor = 1u32 << (encoding as u32 + 1);
            if pclk_hz / factor <= max_hz {
                break;
            }
            encoding += 1;
        }
        match encoding {
            0 => Self::Div2,
            1 => Self::Div4,
            2 => Self::Div8,
            3 => Self::Div16,
            4 => Self::Div32,
            5 => Self::Div64,
            6 => Self::Div128,
            7 => Self::Div256,
            8 => Self::Div512,
            _ => Self::Div1024,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("the peripheral did not reach the expected state in time")]
    Timeout,
    #[error("the DMA transfer failed")]
    Dma(#[from] dma::Error),
}

static WAKER: AtomicWaker = AtomicWaker::new();

/// Board interrupt hook for the `SPI1` vector (drives the flag waits).
pub fn on_interrupt() {
    WAKER.wake();
}

/// SPI master over one peripheral instance.
pub struct Spi {
    spi: &'static RegisterBlock,
}

impl Spi {
    /// Create the driver over `SPI1` (APB2) and program mode and divider.
    ///
    /// # Safety
    ///
    /// The caller must exclusively own the peripheral, and route its NVIC
    /// vector to a handler calling [`on_interrupt`], before any transfer.
    /// Pin muxing is the board's job.
    pub unsafe fn new_spi1(mode: Mode, divider: Divider) -> Self {
        // SAFETY: forwarded from this constructor's exclusive-startup contract.
        let crm = unsafe { CRM::steal() };
        critical_section::with(|_| {
            crm.apb2en().modify(|_, w| w.spi1().bit(true));
        });
        // SAFETY: the peripheral's static register block under this
        // constructor's exclusive-ownership contract.
        let spi = unsafe { &*SPI1::ptr() };
        let this = Self { spi };
        this.configure(mode, divider);
        this
    }

    fn configure(&self, mode: Mode, divider: Divider) {
        // Program with the peripheral disabled, then enable (spi_init order).
        self.spi.ctrl1().modify(|_, w| {
            w.spien().variant(SpienwWO::Disable);
            w.msten().variant(MstenA::Master);
            w.clkpol().variant(match mode {
                Mode::Mode0 | Mode::Mode1 => ClkpolA::Low,
                Mode::Mode2 | Mode::Mode3 => ClkpolA::High,
            });
            w.clkpha().variant(match mode {
                Mode::Mode0 | Mode::Mode2 => ClkphaA::First,
                Mode::Mode1 | Mode::Mode3 => ClkphaA::Second,
            });
            w.ltf().variant(LtfA::Msb);
            w.fbn().variant(FbnA::Bit8);
            // SAFETY: divider encoding is 0..=9; the low 3 bits fit MDIV2_0.
            unsafe { w.mdiv2_0().bits((divider as u8) & 0x07) };
            w
        });
        self.spi
            .ctrl2()
            .modify(|_, w| w.mdiv3().bit(u32::from(divider as u8) > 7));
        self.spi
            .ctrl1()
            .modify(|_, w| w.spien().variant(SpienwWO::Enable));
    }

    fn sts(&self) -> crate::pac::at32f403a::spi1::sts::R {
        self.spi.sts().read()
    }

    async fn wait_flag(
        &self,
        ready: impl Fn(&crate::pac::at32f403a::spi1::sts::R) -> bool + Unpin,
    ) -> Result<(), Error> {
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(50);
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            let sts = self.sts();
            if sts.roerr().bit() || sts.tuerr().bit() || sts.mmerr().bit() {
                return Poll::Ready(Err(Error::Timeout));
            }
            if ready(&sts) {
                return Poll::Ready(Ok(()));
            }
            if embassy_time::Instant::now() >= deadline {
                return Poll::Ready(Err(Error::Timeout));
            }
            Poll::Pending
        })
        .await
    }

    /// Wait until the last byte leaves the shift register (`TDBE` set and
    /// `BF` clear), so a chip-select change cannot truncate a frame.
    pub async fn flush(&self) -> Result<(), Error> {
        self.wait_flag(|sts| sts.tdbe().bit() && !sts.bf().bit())
            .await
    }

    /// Write bytes one buffer-empty event at a time (commands, small frames).
    pub async fn write(&self, bytes: &[u8]) -> Result<(), Error> {
        self.spi.ctrl2().modify(|_, w| w.tdbeie().bit(true));
        let result = async {
            for &byte in bytes {
                self.wait_flag(|sts| sts.tdbe().bit()).await?;
                // SAFETY: 8-bit frame in a whole-value 16-bit data register.
                self.spi.dt().write(|w| unsafe { w.bits(u16::from(byte)) });
            }
            self.flush().await
        }
        .await;
        self.spi.ctrl2().modify(|_, w| w.tdbeie().bit(false));
        result
    }

    /// Write a long buffer through DMA (pixel bursts), request `SPI1_TX`.
    /// The buffer must stay valid until the future completes.
    pub async fn write_dma(&self, channel: &mut DmaChannel, bytes: &[u8]) -> Result<(), Error> {
        self.flush().await?;
        self.spi.ctrl2().modify(|_, w| w.dmaten().bit(true));
        let result: Result<(), Error> = async {
            channel.start(
                Request::Spi1Tx,
                Direction::MemoryToPeripheral,
                core::ptr::from_ref(&self.spi.dt()) as *mut u32,
                bytes.as_ptr() as *mut u32,
                bytes.len(),
                Width::Bit8,
                true,
            )?;
            channel.wait().await?;
            Ok(())
        }
        .await;
        self.spi.ctrl2().modify(|_, w| w.dmaten().bit(false));
        result
    }
}

#[cfg(test)]
mod tests;
