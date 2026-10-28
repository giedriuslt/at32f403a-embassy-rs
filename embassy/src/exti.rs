//! Async external interrupts (EXINT lines 0..=15) for the AT32F403A.
//!
//! The GPIO port feeding each line is selected in `IOMUX.EXINTC1..4` (one
//! nibble per line, value 0..=4 for PA..PE); edge polarity and the interrupt
//! enable live in `EXINT.POLCFG1/2/INTEN`, and `INTSTS` is cleared by
//! writing one to the line bit (vendor `gpio_exint_line_config`,
//! `exint_init`, `exint_flag_clear`). NVIC vectors are grouped EXINTF1-style:
//! `EXINT0..EXINT4` are individual, `EXINT9_5` covers 5..=9 and
//! `EXINT15_10` covers 10..=15; the board's handlers call
//! [`on_interrupt`] for their lines.
//!
//! Two-flag design (as in the HC32 driver this replaces): the hardware flag
//! is cleared in the interrupt handler *and* a software latch is set before
//! the wake, so a waiter that registers after the interrupt still observes
//! the edge. Multiple edges coalesce into one pending event; this is not an
//! edge counter.

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicU16, Ordering};
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{self, Port};

/// GPIO-mapped EXINT lines.
pub const MAX_LINE: u8 = 15;
const LINE_COUNT: usize = MAX_LINE as usize + 1;

static WAKERS: [AtomicWaker; LINE_COUNT] = [const { AtomicWaker::new() }; LINE_COUNT];

/// Software latch: bit `n` means "line `n` saw an edge no waiter consumed".
static EDGE: AtomicU16 = AtomicU16::new(0);

/// Trigger configuration for an EXINT line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Trigger {
    Rising,
    Falling,
    Both,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("line number must be 0..=15")]
    InvalidLine,
}

/// Record an edge for `line` and wake its waiter. Order matters: latch, then wake.
pub fn note_edge(line: u8) {
    debug_assert!(line <= MAX_LINE);
    EDGE.fetch_or(1 << line, Ordering::Release);
    WAKERS[line as usize].wake();
}

/// Take the latched edge for `line`, if any.
pub fn take_edge(line: u8) -> bool {
    debug_assert!(line <= MAX_LINE);
    EDGE.fetch_and(!(1 << line), Ordering::AcqRel) & (1 << line) != 0
}

/// Whether an edge is latched and not yet consumed for `line`.
pub fn is_pending(line: u8) -> bool {
    EDGE.load(Ordering::Relaxed) & (1 << line) != 0
}

/// Hardware side of an incoming external interrupt.
///
/// Clears the hardware flag (it must be cleared, or the handler re-enters
/// forever), latches the edge in software and wakes the waiter. Returns
/// `true` when the line actually had a pending flag.
///
/// The board's `EXINTn` handlers call this with their line number; grouped
/// vectors (`EXINT9_5`, `EXINT15_10`) can use [`on_interrupts_in`].
pub fn on_interrupt(line: u8) -> bool {
    let exint = unsafe { crate::pac::at32f403a::EXINT::steal() };
    if line_pending(&exint, line) {
        clear_line(&exint, line);
        note_edge(line);
        true
    } else {
        false
    }
}

/// Service every pending line in `lo..=hi` (the grouped EXINT vectors).
pub fn on_interrupts_in(range: core::ops::RangeInclusive<u8>) {
    for line in range {
        on_interrupt(line);
    }
}

/// EXINT interrupt enable for one line, without touching its polarity.
fn set_line_enabled(line: u8, enabled: bool) {
    let exint = unsafe { crate::pac::at32f403a::EXINT::steal() };
    exint.inten().modify(|_, w| inten_field!(w, line, enabled));
}

/// Configure one line's port source, polarity and pending state.
fn configure_line(port: Port, line: u8, trigger: Trigger) {
    // IOMUX clock gates the EXINTC port-selection registers.
    let crm = unsafe { crate::pac::at32f403a::CRM::steal() };
    critical_section::with(|_| {
        crm.apb2en().modify(|_, w| w.iomux().bit(true));
        let iomux = unsafe { crate::pac::at32f403a::IOMUX::steal() };
        // SAFETY: port discriminants are 0..=4, fitting EXINTC's 4-bit fields.
        exintc_field!(iomux, line, port as u8);

        let exint = unsafe { crate::pac::at32f403a::EXINT::steal() };
        exint.polcfg1().modify(|_, w| {
            polcfg1_field!(w, line, matches!(trigger, Trigger::Rising | Trigger::Both))
        });
        exint.polcfg2().modify(|_, w| {
            polcfg2_field!(w, line, matches!(trigger, Trigger::Falling | Trigger::Both))
        });
        exint.inten().modify(|_, w| inten_field!(w, line, true));
        clear_line(&exint, line);
    });
}

fn line_pending(exint: &crate::pac::at32f403a::exint::RegisterBlock, line: u8) -> bool {
    let sts = exint.intsts().read();
    intsts_field!(sts, line)
}

fn clear_line(exint: &crate::pac::at32f403a::exint::RegisterBlock, line: u8) {
    exint.intsts().write(|w| intsts_clear_field!(w, line));
}

/// An input pin routed to its EXINT line, with async edge waiting.
pub struct ExtiInput {
    pin: gpio::Pin,
}

impl ExtiInput {
    /// Configure `port`'s pin `num` as an EXINT input with the given trigger.
    ///
    /// # Safety
    ///
    /// The caller must exclusively own the pin *and* its EXINT line across
    /// all GPIO ports (only one port feeds a line at a time), and must have
    /// the line's NVIC vector disabled or its handler installed before any
    /// interrupt can arrive.
    pub unsafe fn new(port: Port, num: u8, trigger: Trigger) -> Result<Self, Error> {
        if num > MAX_LINE {
            return Err(Error::InvalidLine);
        }
        // num <= 15 is checked; Pin::new's only error is InvalidPin (> 15).
        let pin = unsafe { gpio::Pin::new(port, num) }.map_err(|_| Error::InvalidLine)?;
        configure_line(port, num, trigger);
        take_edge(num);
        Ok(Self { pin })
    }

    /// Consume an already-configured input pin and attach an EXINT line.
    ///
    /// # Safety
    /// Same line-ownership contract as [`Self::new`].
    pub unsafe fn from_input(input: gpio::InputPin, trigger: Trigger) -> Result<Self, Error> {
        let pin = input.degrade();
        if pin.num > MAX_LINE {
            return Err(Error::InvalidLine);
        }
        configure_line(pin.port, pin.num, trigger);
        take_edge(pin.num);
        Ok(Self { pin })
    }

    pub fn is_high(&self) -> bool {
        let gpio = self.pin;
        gpio_input_level(gpio)
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Whether an edge is already latched and not yet consumed.
    pub fn is_pending(&self) -> bool {
        is_pending(self.pin.num)
    }

    fn arm(&mut self, trigger: Trigger) {
        configure_line(self.pin.port, self.pin.num, trigger);
        take_edge(self.pin.num);
    }

    /// Wait until the pin produces a configured edge event.
    ///
    /// Registers the waker *before* checking the latch, so an interrupt that
    /// arrives between two polls cannot be lost.
    pub async fn wait_for_edge(&mut self) {
        WaitEdge { line: self.pin.num }.await;
    }

    pub async fn wait_for_rising_edge(&mut self) {
        self.arm(Trigger::Rising);
        self.wait_for_edge().await;
    }

    pub async fn wait_for_falling_edge(&mut self) {
        self.arm(Trigger::Falling);
        self.wait_for_edge().await;
    }

    pub async fn wait_for_any_edge(&mut self) {
        self.arm(Trigger::Both);
        self.wait_for_edge().await;
    }

    /// Return immediately if high, otherwise wait for a rising edge even if
    /// the signal falls again before the executor polls this task.
    pub async fn wait_for_high(&mut self) {
        self.arm(Trigger::Rising);
        if !self.is_high() {
            self.wait_for_edge().await;
        }
    }

    pub async fn wait_for_low(&mut self) {
        self.arm(Trigger::Falling);
        if !self.is_low() {
            self.wait_for_edge().await;
        }
    }

    /// Stop generating interrupts for this line (the pin stays an input).
    pub fn disarm(&mut self) {
        set_line_enabled(self.pin.num, false);
    }

    pub fn rearm(&mut self, trigger: Trigger) {
        configure_line(self.pin.port, self.pin.num, trigger);
        take_edge(self.pin.num);
    }
}

fn gpio_input_level(pin: gpio::Pin) -> bool {
    use crate::gpio::Port;
    let block = match pin.port {
        Port::A => unsafe { &*crate::pac::at32f403a::GPIOA::steal() },
        Port::B => unsafe { &*crate::pac::at32f403a::GPIOB::steal() },
        Port::C => unsafe { &*crate::pac::at32f403a::GPIOC::steal() },
        Port::D => unsafe { &*crate::pac::at32f403a::GPIOD::steal() },
        Port::E => unsafe { &*crate::pac::at32f403a::GPIOE::steal() },
    };
    idt_line!(block, pin.num)
}

impl embedded_hal::digital::ErrorType for ExtiInput {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for ExtiInput {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(ExtiInput::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(ExtiInput::is_low(self))
    }
}

impl embedded_hal_async::digital::Wait for ExtiInput {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        ExtiInput::wait_for_high(self).await;
        Ok(())
    }
    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        ExtiInput::wait_for_low(self).await;
        Ok(())
    }
    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        ExtiInput::wait_for_rising_edge(self).await;
        Ok(())
    }
    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        ExtiInput::wait_for_falling_edge(self).await;
        Ok(())
    }
    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        ExtiInput::wait_for_any_edge(self).await;
        Ok(())
    }
}

struct WaitEdge {
    line: u8,
}

impl Future for WaitEdge {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        // Register first, then consume: a handler between the two only sets
        // the latch, which this poll picks up.
        WAKERS[self.line as usize].register(cx.waker());
        if take_edge(self.line) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// Register-field dispatch macros: per-line field names differ, so each macro
// lists the arms explicitly instead of indexing.

fn inten_variant(enabled: bool) -> crate::pac::at32f403a::exint::inten::Inten0wWO {
    if enabled {
        crate::pac::at32f403a::exint::inten::Inten0wWO::Enable
    } else {
        crate::pac::at32f403a::exint::inten::Inten0wWO::Disable
    }
}

/// Write one line's INTEN bit.
macro_rules! inten_field {
    ($w:ident, $line:expr, $enabled:expr) => {
        match $line {
            0 => $w.inten0().variant(inten_variant($enabled)),
            1 => $w.inten1().variant(inten_variant($enabled)),
            2 => $w.inten2().variant(inten_variant($enabled)),
            3 => $w.inten3().variant(inten_variant($enabled)),
            4 => $w.inten4().variant(inten_variant($enabled)),
            5 => $w.inten5().variant(inten_variant($enabled)),
            6 => $w.inten6().variant(inten_variant($enabled)),
            7 => $w.inten7().variant(inten_variant($enabled)),
            8 => $w.inten8().variant(inten_variant($enabled)),
            9 => $w.inten9().variant(inten_variant($enabled)),
            10 => $w.inten10().variant(inten_variant($enabled)),
            11 => $w.inten11().variant(inten_variant($enabled)),
            12 => $w.inten12().variant(inten_variant($enabled)),
            13 => $w.inten13().variant(inten_variant($enabled)),
            14 => $w.inten14().variant(inten_variant($enabled)),
            15 => $w.inten15().variant(inten_variant($enabled)),
            _ => $w,
        }
    };
}
use inten_field;

fn rising_variant(set: bool) -> crate::pac::at32f403a::exint::polcfg1::Rp0wWO {
    if set {
        crate::pac::at32f403a::exint::polcfg1::Rp0wWO::Enable
    } else {
        crate::pac::at32f403a::exint::polcfg1::Rp0wWO::Disable
    }
}

fn falling_variant(set: bool) -> crate::pac::at32f403a::exint::polcfg2::Fp0wWO {
    if set {
        crate::pac::at32f403a::exint::polcfg2::Fp0wWO::Enable
    } else {
        crate::pac::at32f403a::exint::polcfg2::Fp0wWO::Disable
    }
}

/// Write one line's rising-polarity bit in POLCFG1.
macro_rules! polcfg1_field {
    ($w:ident, $line:expr, $set:expr) => {
        match $line {
            0 => $w.rp0().variant(rising_variant($set)),
            1 => $w.rp1().variant(rising_variant($set)),
            2 => $w.rp2().variant(rising_variant($set)),
            3 => $w.rp3().variant(rising_variant($set)),
            4 => $w.rp4().variant(rising_variant($set)),
            5 => $w.rp5().variant(rising_variant($set)),
            6 => $w.rp6().variant(rising_variant($set)),
            7 => $w.rp7().variant(rising_variant($set)),
            8 => $w.rp8().variant(rising_variant($set)),
            9 => $w.rp9().variant(rising_variant($set)),
            10 => $w.rp10().variant(rising_variant($set)),
            11 => $w.rp11().variant(rising_variant($set)),
            12 => $w.rp12().variant(rising_variant($set)),
            13 => $w.rp13().variant(rising_variant($set)),
            14 => $w.rp14().variant(rising_variant($set)),
            15 => $w.rp15().variant(rising_variant($set)),
            _ => $w,
        }
    };
}
use polcfg1_field;

/// Write one line's falling-polarity bit in POLCFG2.
macro_rules! polcfg2_field {
    ($w:ident, $line:expr, $set:expr) => {
        match $line {
            0 => $w.fp0().variant(falling_variant($set)),
            1 => $w.fp1().variant(falling_variant($set)),
            2 => $w.fp2().variant(falling_variant($set)),
            3 => $w.fp3().variant(falling_variant($set)),
            4 => $w.fp4().variant(falling_variant($set)),
            5 => $w.fp5().variant(falling_variant($set)),
            6 => $w.fp6().variant(falling_variant($set)),
            7 => $w.fp7().variant(falling_variant($set)),
            8 => $w.fp8().variant(falling_variant($set)),
            9 => $w.fp9().variant(falling_variant($set)),
            10 => $w.fp10().variant(falling_variant($set)),
            11 => $w.fp11().variant(falling_variant($set)),
            12 => $w.fp12().variant(falling_variant($set)),
            13 => $w.fp13().variant(falling_variant($set)),
            14 => $w.fp14().variant(falling_variant($set)),
            15 => $w.fp15().variant(falling_variant($set)),
            _ => $w,
        }
    };
}
use polcfg2_field;

/// Read one line's INTSTS pending bit.
macro_rules! intsts_field {
    ($sts:expr, $line:expr) => {
        match $line {
            0 => $sts.line0().is_interrupt(),
            1 => $sts.line1().is_interrupt(),
            2 => $sts.line2().is_interrupt(),
            3 => $sts.line3().is_interrupt(),
            4 => $sts.line4().is_interrupt(),
            5 => $sts.line5().is_interrupt(),
            6 => $sts.line6().is_interrupt(),
            7 => $sts.line7().is_interrupt(),
            8 => $sts.line8().is_interrupt(),
            9 => $sts.line9().is_interrupt(),
            10 => $sts.line10().is_interrupt(),
            11 => $sts.line11().is_interrupt(),
            12 => $sts.line12().is_interrupt(),
            13 => $sts.line13().is_interrupt(),
            14 => $sts.line14().is_interrupt(),
            15 => $sts.line15().is_interrupt(),
            _ => false,
        }
    };
}
use intsts_field;

/// Write-1-to-clear one line's INTSTS bit.
macro_rules! intsts_clear_field {
    ($w:ident, $line:expr) => {
        match $line {
            0 => $w.line0().clear(),
            1 => $w.line1().clear(),
            2 => $w.line2().clear(),
            3 => $w.line3().clear(),
            4 => $w.line4().clear(),
            5 => $w.line5().clear(),
            6 => $w.line6().clear(),
            7 => $w.line7().clear(),
            8 => $w.line8().clear(),
            9 => $w.line9().clear(),
            10 => $w.line10().clear(),
            11 => $w.line11().clear(),
            12 => $w.line12().clear(),
            13 => $w.line13().clear(),
            14 => $w.line14().clear(),
            15 => $w.line15().clear(),
            _ => $w,
        }
    };
}
use intsts_clear_field;

/// Write one line's port source into IOMUX.EXINTC1..4. Field names are
/// absolute: EXINTC1 carries `exint0..3`, EXINTC2 `exint4..7`, and so on.
macro_rules! exintc_field {
    ($iomux:ident, $line:expr, $port:expr) => {
        match $line {
            0..=3 => $iomux.exintc1().modify(|_, w| match $line {
                0 => unsafe { w.exint0().bits($port) },
                1 => unsafe { w.exint1().bits($port) },
                2 => unsafe { w.exint2().bits($port) },
                _ => unsafe { w.exint3().bits($port) },
            }),
            4..=7 => $iomux.exintc2().modify(|_, w| match $line {
                4 => unsafe { w.exint4().bits($port) },
                5 => unsafe { w.exint5().bits($port) },
                6 => unsafe { w.exint6().bits($port) },
                _ => unsafe { w.exint7().bits($port) },
            }),
            8..=11 => $iomux.exintc3().modify(|_, w| match $line {
                8 => unsafe { w.exint8().bits($port) },
                9 => unsafe { w.exint9().bits($port) },
                10 => unsafe { w.exint10().bits($port) },
                _ => unsafe { w.exint11().bits($port) },
            }),
            _ => $iomux.exintc4().modify(|_, w| match $line {
                12 => unsafe { w.exint12().bits($port) },
                13 => unsafe { w.exint13().bits($port) },
                14 => unsafe { w.exint14().bits($port) },
                _ => unsafe { w.exint15().bits($port) },
            }),
        }
    };
}
use exintc_field;

/// Read one pin's input level through the per-bit IDT readers.
macro_rules! idt_line {
    ($block:expr, $num:expr) => {
        match $num {
            0 => $block.idt().read().idt0().bit(),
            1 => $block.idt().read().idt1().bit(),
            2 => $block.idt().read().idt2().bit(),
            3 => $block.idt().read().idt3().bit(),
            4 => $block.idt().read().idt4().bit(),
            5 => $block.idt().read().idt5().bit(),
            6 => $block.idt().read().idt6().bit(),
            7 => $block.idt().read().idt7().bit(),
            8 => $block.idt().read().idt8().bit(),
            9 => $block.idt().read().idt9().bit(),
            10 => $block.idt().read().idt10().bit(),
            11 => $block.idt().read().idt11().bit(),
            12 => $block.idt().read().idt12().bit(),
            13 => $block.idt().read().idt13().bit(),
            14 => $block.idt().read().idt14().bit(),
            15 => $block.idt().read().idt15().bit(),
            _ => false,
        }
    };
}
use idt_line;
