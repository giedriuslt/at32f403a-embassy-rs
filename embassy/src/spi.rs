//! SPI master, transmit-focused, for the AT32F403A's F1-style blocks.
//!
//! The panel transport of the KWS-X1 firmware needs three operations, all
//! provided as inherent async methods rather than the full `embedded-hal`
//! bus traits: byte-wise [`Spi::write`] for commands, [`Spi::flush`] for
//! transmit-idle fencing, and [`Spi::write_dma`] for pixel bursts through a
//! [`crate::dma::Channel`].
//!
//! The clock-divider encoding is the vendor's (`spi_init`): SCK equals PCLK
//! divided by 2^(encoding + 1), with the encoding's low three bits in
//! `CTRL1.MDIV2_0` and its high bit in `CTRL2.MDIV3` (for /512 and /1024). Modes
//! map to `CLKPOL`/`CLKPHA`, and frames are 8-bit MSB-first. Chip select is
//! the board's job.

use embassy_time::Timer;

use crate::dma::{self, Channel as DmaChannel, Direction, Request, Width};
use crate::pac::at32f403a::spi1::RegisterBlock;
use crate::pac::at32f403a::spi1::ctrl1::{
    CLKPHA_A as ClkphaA, CLKPOL_A as ClkpolA, FBN_A as FbnA, LTF_A as LtfA, MSTEN_A as MstenA,
    SLBTD_A, SWCSIL_A, SlbenwWO, SpienwWO, SwcsenwWO,
};
use crate::pac::at32f403a::{CRM, SPI1, SPI2};

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
    #[error("mode fault: another master drove the bus (internal CS dropped)")]
    ModeFault,
    #[error("the DMA transfer failed")]
    Dma(#[from] dma::Error),
}

/// SPI master over one peripheral instance.
///
/// `Copy`: the handle is just a `&'static RegisterBlock` — several drivers
/// may hold copies of one bus as long as only one of them drives it at a
/// time (the FNB58's LCD and W25Q128 flash share SPI2, selected by their
/// own chip-select pins).
#[derive(Clone, Copy)]
pub struct Spi {
    spi: &'static RegisterBlock,
}

impl Spi {
    /// Create the driver over `SPI1` (APB2) and program mode and divider.
    ///
    /// # Safety
    ///
    /// The caller must exclusively own the peripheral. All waits poll with
    /// a deadline, so no NVIC routing is required. Pin muxing is the
    /// board's job.
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

    /// Create the driver over `SPI2` (APB1). The register block layout is
    /// identical to SPI1's (same IP; the PAC types share
    /// [`spi1::RegisterBlock`]).
    ///
    /// # Safety
    ///
    /// Same as [`Self::new_spi1`].
    pub unsafe fn new_spi2(mode: Mode, divider: Divider) -> Self {
        let crm = unsafe { CRM::steal() };
        critical_section::with(|_| {
            crm.apb1en().modify(|_, w| w.spi2().bit(true));
        });
        // SAFETY: the peripheral's static register block under this
        // constructor's exclusive-ownership contract; same layout type.
        let spi = unsafe { &*SPI2::ptr() };
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
            // Transmit-only: the single-bi direction sends, so the receive
            // buffer never fills and ROERR cannot latch. RX-only TX mode
            // would keep full-duplex timing but we never read MISO.
            w.slben().variant(SlbenwWO::Enable);
            w.slbtd().variant(SLBTD_A::Transmit);
            // Software chip-select: hardware CS would sample the (panel)
            // CS pin and drop out of master mode with MMERR when it reads
            // low. Keep the internal NSS level high.
            w.swcsen().variant(SwcsenwWO::Enable);
            w.swcsil().variant(SWCSIL_A::High);
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

    /// Poll until `ready` holds or the deadline passes. The wait is
    /// timer-backed rather than interrupt-driven: bring-up does not depend
    /// on NVIC routing, and a masked interrupt can never hang it.
    async fn wait_flag(
        &self,
        ready: impl Fn(&crate::pac::at32f403a::spi1::sts::R) -> bool,
    ) -> Result<(), Error> {
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(50);
        loop {
            let sts = self.sts();
            if sts.mmerr().bit() {
                // MMERR clears on an STS read; clear it before reporting.
                let _ = self.sts();
                return Err(Error::ModeFault);
            }
            if ready(&sts) {
                return Ok(());
            }
            if embassy_time::Instant::now() >= deadline {
                return Err(Error::Timeout);
            }
            Timer::after_micros(2).await;
        }
    }

    /// Wait until the last byte leaves the shift register (`TDBE` set and
    /// `BF` clear), so a chip-select change cannot truncate a frame.
    pub async fn flush(&self) -> Result<(), Error> {
        self.wait_flag(|sts| sts.tdbe().bit() && !sts.bf().bit())
            .await
    }

    /// Write bytes one buffer-empty event at a time (commands, small frames).
    pub async fn write(&self, bytes: &[u8]) -> Result<(), Error> {
        for &byte in bytes {
            self.wait_flag(|sts| sts.tdbe().bit()).await?;
            // SAFETY: 8-bit frame in a whole-value 16-bit data register.
            self.spi.dt().write(|w| unsafe { w.bits(u16::from(byte)) });
        }
        self.flush().await
    }

    /// Switch to normal full-duplex mode (MISO receive active). Used by
    /// drivers that read from a shared bus (the LCD is write-only and
    /// keeps the single-wire transmit mode).
    pub fn set_full_duplex(&self) {
        self.spi
            .ctrl1()
            .modify(|_, w| w.spien().variant(SpienwWO::Disable));
        self.spi
            .ctrl1()
            .modify(|_, w| w.slben().variant(SlbenwWO::Disable));
        self.spi
            .ctrl1()
            .modify(|_, w| w.spien().variant(SpienwWO::Enable));
    }

    /// Switch back to single-wire transmit-only mode (the LCD's mode).
    pub fn set_single_wire_transmit(&self) {
        self.spi
            .ctrl1()
            .modify(|_, w| w.spien().variant(SpienwWO::Disable));
        self.spi
            .ctrl1()
            .modify(|_, w| w.slben().variant(SlbenwWO::Enable));
        self.spi
            .ctrl1()
            .modify(|_, w| w.slbtd().variant(SLBTD_A::Transmit));
        self.spi
            .ctrl1()
            .modify(|_, w| w.spien().variant(SpienwWO::Enable));
    }

    /// Full-duplex transfer: every byte in `bytes` is clocked out while
    /// the received byte is collected into `out` (shorter `out` = extra
    /// bytes drained). Busy-polls; blocks the task for the transfer.
    pub fn transfer_busy(&self, bytes: &[u8], out: &mut [u8]) -> Result<(), Error> {
        for (i, &b) in bytes.iter().enumerate() {
            loop {
                let sts = self.sts();
                if sts.mmerr().bit() {
                    return Err(Error::ModeFault);
                }
                if sts.tdbe().bit() {
                    break;
                }
            }
            // SAFETY: 8-bit frame in a whole-value 16-bit data register.
            self.spi.dt().write(|w| unsafe { w.bits(u16::from(b)) });
            while self.sts().bf().bit() {}
            let received = self.spi.dt().read().bits() as u8;
            if i < out.len() {
                out[i] = received;
            }
        }
        Ok(())
    }

    /// Write bytes with tight TDBE busy-polling — no timer futures, no
    /// executor round-trips per byte. Blocks the task for the transfer
    /// duration (12 MHz SCK is ~0.7 us/byte); intended for bulk pixel
    /// data on single-task firmware, where the C drivers busy-wait too.
    pub fn write_busy(&self, bytes: &[u8]) -> Result<(), Error> {
        for &byte in bytes {
            loop {
                let sts = self.sts();
                if sts.mmerr().bit() {
                    // MMERR clears on an STS read; the read above cleared
                    // it before reporting.
                    return Err(Error::ModeFault);
                }
                if sts.tdbe().bit() {
                    break;
                }
            }
            // SAFETY: 8-bit frame in a whole-value 16-bit data register.
            self.spi.dt().write(|w| unsafe { w.bits(u16::from(byte)) });
        }
        // Idle fence: the last byte must leave the shifter before the
        // caller changes D/C or chip-select.
        while self.sts().bf().bit() {}
        Ok(())
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
