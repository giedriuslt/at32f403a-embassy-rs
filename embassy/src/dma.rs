//! DMA for the AT32F403A: two units of seven F1-style channels each, with
//! the AT32's flexible request mapping (`DMA_SRC_SEL0/1`), async completion
//! waits and abort-on-drop.
//!
//! Configuration order follows the vendor `dma_init`/`dma_channel_enable`:
//! with `CHEN` clear, program direction (`DTD`), widths (`PWIDTH`/`MWIDTH`),
//! increment modes (`PINCM`/`MINCM`), loop mode (`LM`) and the interrupt
//! enables, then `DTCNT` (the whole item count, not count minus one),
//! `PADDR` and `MADDR`, and only then enable `CHEN`. The full-transfer
//! interrupt (`FDTIEN`/`FDTF`) wakes the awaiting task; the board's
//! `DMAn_CHANNELm` handlers call [`on_interrupt`] (the grouped
//! `DMA2_CHANNEL4_5` and `DMA2_CHANNEL6_7` vectors call it for both of
//! their channels).

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac::at32f403a::dma1::channel::ctrl::{
    ChenwWO, DTD_A as DtdA, DterrienwWO, FdtienwWO, LmwWO, M2mwWO, MincmwWO, PincmwWO,
};

/// Which DMA controller a channel belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unit {
    Dma1,
    Dma2,
}

impl Unit {
    /// Both units share the register layout; DMA2's block is transmuted to
    /// DMA1's type, which is layout-identical.
    fn block(self) -> &'static crate::pac::at32f403a::dma1::RegisterBlock {
        match self {
            Self::Dma1 => unsafe { &*crate::pac::at32f403a::DMA1::ptr() },
            Self::Dma2 => unsafe {
                core::mem::transmute::<
                    &'static crate::pac::at32f403a::dma2::RegisterBlock,
                    &'static crate::pac::at32f403a::dma1::RegisterBlock,
                >(&*crate::pac::at32f403a::DMA2::ptr())
            },
        }
    }
}

/// Transfer direction (`CTRL.DTD`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    MemoryToPeripheral,
    PeripheralToMemory,
}

/// Data width, applied to both bus sides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Width {
    Bit8,
    Bit16,
    Bit32,
}

/// Flexible request source (`DMA_SRC_SEL0/1` values; vendor
/// `dma_flexible_request_type`). Only the sources this HAL's drivers need
/// are listed; extend from the vendor enum as drivers appear.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Request {
    Adc1 = 0x00,
    Adc2 = 0x01,
    Adc3 = 0x02,
    Spi1Rx = 0x09,
    Spi1Tx = 0x0A,
    Spi2Rx = 0x0B,
    Spi2Tx = 0x0C,
    Spi3Rx = 0x0D,
    Spi3Tx = 0x0E,
    Spi4Rx = 0x0F,
    Spi4Tx = 0x10,
    I2c1Rx = 0x14,
    I2c1Tx = 0x15,
    I2c2Rx = 0x16,
    I2c2Tx = 0x17,
    I2c3Rx = 0x18,
    I2c3Tx = 0x19,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("channel index must be 1..=7")]
    InvalidChannel,
    #[error("the transfer needs 1..=65535 items")]
    TooLong,
    #[error("the DMA transfer reported a data error")]
    DataError,
    #[error("the channel is already running a transfer")]
    Busy,
}

const CHANNELS: usize = 7;
static WAKERS: [[AtomicWaker; CHANNELS]; 2] =
    [const { [const { AtomicWaker::new() }; CHANNELS] }; 2];
static RUNNING: [[AtomicBool; CHANNELS]; 2] =
    [const { [const { AtomicBool::new(false) }; CHANNELS] }; 2];

fn unit_index(unit: Unit) -> usize {
    match unit {
        Unit::Dma1 => 0,
        Unit::Dma2 => 1,
    }
}

/// One DMA channel. Dropping it mid-transfer aborts (`CHEN` off).
pub struct Channel {
    unit: Unit,
    /// 1-based channel number, matching the vendor's naming.
    pub number: u8,
}

impl Drop for Channel {
    fn drop(&mut self) {
        self.abort();
    }
}

impl Channel {
    /// Claim every channel of one unit and enable its clock.
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: the caller claims the unit's channels
    /// once per boot, and routes the unit's NVIC vectors to handlers that
    /// call [`on_interrupt`] before starting any transfer.
    pub unsafe fn split(unit: Unit) -> Result<[Self; CHANNELS], Error> {
        // SAFETY: forwarded from this constructor's exclusive-startup contract.
        let crm = unsafe { crate::pac::at32f403a::CRM::steal() };
        critical_section::with(|_| {
            crm.ahben().modify(|_, w| {
                w.dma1().bit(matches!(unit, Unit::Dma1));
                w.dma2().bit(matches!(unit, Unit::Dma2));
                w
            });
        });
        let all = core::array::from_fn(|i| Self {
            unit,
            number: i as u8 + 1,
        });
        Ok(all)
    }

    fn regs(&self) -> &'static crate::pac::at32f403a::dma1::channel::Channel {
        self.unit.block().channel(self.number as usize - 1)
    }

    /// Program the channel for a transfer and enable it. `len` is the item
    /// count (per `Width`), 1..=65535; the register stores it whole.
    #[allow(clippy::too_many_arguments)]
    pub fn start(
        &mut self,
        request: Request,
        direction: Direction,
        peripheral: *mut u32,
        memory: *mut u32,
        len: usize,
        width: Width,
        memory_increment: bool,
    ) -> Result<(), Error> {
        if self.number == 0 || self.number > CHANNELS as u8 {
            return Err(Error::InvalidChannel);
        }
        if len == 0 || len > u16::MAX as usize {
            return Err(Error::TooLong);
        }
        let slot = (unit_index(self.unit), self.number as usize - 1);
        if RUNNING[slot.0][slot.1].swap(true, Ordering::AcqRel) {
            return Err(Error::Busy);
        }

        let block = self.unit.block();
        // Flexible request mapping: enable the selector, then point this
        // channel at its request source (4-bit fields).
        // SAFETY: request codes are 0..=0x19, fitting the 4-bit fields.
        unsafe {
            block
                .dma_src_sel1()
                .modify(|_, w| w.dma_flex_en().bit(true));
            let selector = request as u8;
            if self.number <= 4 {
                block.dma_src_sel0().modify(|_, w| {
                    match self.number {
                        1 => w.ch1_src().bits(selector),
                        2 => w.ch2_src().bits(selector),
                        3 => w.ch3_src().bits(selector),
                        _ => w.ch4_src().bits(selector),
                    };
                    w
                });
            } else {
                block.dma_src_sel1().modify(|_, w| {
                    match self.number {
                        5 => w.ch5_src().bits(selector),
                        6 => w.ch6_src().bits(selector),
                        _ => w.ch7_src().bits(selector),
                    };
                    w
                });
            }
        }

        let channel = self.regs();
        use crate::pac::at32f403a::dma1::channel::ctrl::{MWIDTH_A, PWIDTH_A};
        let (pwidth, mwidth) = match width {
            Width::Bit8 => (PWIDTH_A::Bit8, MWIDTH_A::Bit8),
            Width::Bit16 => (PWIDTH_A::Bit16, MWIDTH_A::Bit16),
            Width::Bit32 => (PWIDTH_A::Bit32, MWIDTH_A::Bit32),
        };
        channel.ctrl().modify(|_, w| {
            w.chen().variant(ChenwWO::Disable);
            w.m2m().variant(M2mwWO::Disable);
            w.dtd().variant(match direction {
                Direction::MemoryToPeripheral => DtdA::Peripherals,
                Direction::PeripheralToMemory => DtdA::Memory,
            });
            w.pwidth().variant(pwidth);
            w.mwidth().variant(mwidth);
            w.mincm().variant(if memory_increment {
                MincmwWO::Enable
            } else {
                MincmwWO::Disable
            });
            w.pincm().variant(PincmwWO::Disable);
            w.lm().variant(LmwWO::Disable);
            w.fdtien().variant(FdtienwWO::Enable);
            w.dterrien().variant(DterrienwWO::Enable);
            w
        });
        // SAFETY: len fits u16 (checked above); whole-value registers.
        unsafe {
            channel.dtcnt().write(|w| w.bits(len as u16));
            channel.paddr().write(|w| w.bits(peripheral as u32));
            channel.maddr().write(|w| w.bits(memory as u32));
        }

        // Clear stale flags from any earlier use, then start.
        clear_flags(block, self.number);
        channel
            .ctrl()
            .modify(|_, w| w.chen().variant(ChenwWO::Enable));
        Ok(())
    }

    /// Stop the channel and forget a pending transfer.
    pub fn abort(&mut self) {
        let channel = self.regs();
        channel.ctrl().modify(|_, w| {
            w.chen().variant(ChenwWO::Disable);
            w.fdtien().variant(FdtienwWO::Disable);
            w.dterrien().variant(DterrienwWO::Disable);
            w
        });
        let slot = (unit_index(self.unit), self.number as usize - 1);
        RUNNING[slot.0][slot.1].store(false, Ordering::Release);
    }

    /// Wait for the transfer to complete (full-transfer flag) or report its
    /// data error. Dropping the future aborts the channel through `Drop`.
    pub async fn wait(&mut self) -> Result<(), Error> {
        TransferWait {
            unit: self.unit,
            number: self.number,
        }
        .await
    }

    /// Whether the channel still has a transfer in flight.
    pub fn is_running(&self) -> bool {
        let slot = (unit_index(self.unit), self.number as usize - 1);
        RUNNING[slot.0][slot.1].load(Ordering::Acquire)
    }
}

fn sts_fdt(sts: &crate::pac::at32f403a::dma1::sts::R, number: u8) -> bool {
    match number {
        1 => sts.fdtf1().bit(),
        2 => sts.fdtf2().bit(),
        3 => sts.fdtf3().bit(),
        4 => sts.fdtf4().bit(),
        5 => sts.fdtf5().bit(),
        6 => sts.fdtf6().bit(),
        _ => sts.fdtf7().bit(),
    }
}

fn sts_dterr(sts: &crate::pac::at32f403a::dma1::sts::R, number: u8) -> bool {
    match number {
        1 => sts.dterrf1().bit(),
        2 => sts.dterrf2().bit(),
        3 => sts.dterrf3().bit(),
        4 => sts.dterrf4().bit(),
        5 => sts.dterrf5().bit(),
        6 => sts.dterrf6().bit(),
        _ => sts.dterrf7().bit(),
    }
}

fn clear_flags(block: &crate::pac::at32f403a::dma1::RegisterBlock, number: u8) {
    block.clr().write(|w| dma_clear_field!(w, number));
}

/// Board interrupt hook: service one channel's full/error flags and wake its
/// waiter. Returns `true` when something was pending.
pub fn on_interrupt(unit: Unit, number: u8) -> bool {
    let block = unit.block();
    let sts = block.sts().read();
    let done = sts_fdt(&sts, number);
    let error = sts_dterr(&sts, number);
    if done || error {
        clear_flags(block, number);
        let slot = (unit_index(unit), number as usize - 1);
        if done {
            RUNNING[slot.0][slot.1].store(false, Ordering::Release);
        }
        WAKERS[slot.0][slot.1].wake();
    }
    done || error
}

struct TransferWait {
    unit: Unit,
    number: u8,
}

impl Future for TransferWait {
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let slot = (unit_index(self.unit), self.number as usize - 1);
        WAKERS[slot.0][slot.1].register(cx.waker());
        let block = self.unit.block();
        let sts = block.sts().read();
        if sts_dterr(&sts, self.number) {
            clear_flags(block, self.number);
            RUNNING[slot.0][slot.1].store(false, Ordering::Release);
            return Poll::Ready(Err(Error::DataError));
        }
        if sts_fdt(&sts, self.number) {
            clear_flags(block, self.number);
            RUNNING[slot.0][slot.1].store(false, Ordering::Release);
            return Poll::Ready(Ok(()));
        }
        Poll::Pending
    }
}

/// Write one channel's clear bits (full, half, global and error).
macro_rules! dma_clear_field {
    ($w:ident, $number:expr) => {
        match $number {
            1 => {
                $w.fdtfc1().set_bit();
                $w.hdtfc1().set_bit();
                $w.gfc1().set_bit();
                $w.dterrfc1().set_bit();
                $w
            }
            2 => {
                $w.fdtfc2().set_bit();
                $w.hdtfc2().set_bit();
                $w.gfc2().set_bit();
                $w.dterrfc2().set_bit();
                $w
            }
            3 => {
                $w.fdtfc3().set_bit();
                $w.hdtfc3().set_bit();
                $w.gfc3().set_bit();
                $w.dterrfc3().set_bit();
                $w
            }
            4 => {
                $w.fdtfc4().set_bit();
                $w.hdtfc4().set_bit();
                $w.gfc4().set_bit();
                $w.dterrfc4().set_bit();
                $w
            }
            5 => {
                $w.fdtfc5().set_bit();
                $w.hdtfc5().set_bit();
                $w.gfc5().set_bit();
                $w.dterrfc5().set_bit();
                $w
            }
            6 => {
                $w.fdtfc6().set_bit();
                $w.hdtfc6().set_bit();
                $w.gfc6().set_bit();
                $w.dterrfc6().set_bit();
                $w
            }
            _ => {
                $w.fdtfc7().set_bit();
                $w.hdtfc7().set_bit();
                $w.gfc7().set_bit();
                $w.dterrfc7().set_bit();
                $w
            }
        }
    };
}
use dma_clear_field;
