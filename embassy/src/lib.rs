//! Embassy HAL support for the Artery AT32F403A (Cortex-M4F, 240 MHz).
//!
//! Built for the FNIRSI FNB58 USB-tester firmware port of
//! `kws-x1-standalone-rs`, following the structure of `embassy-hc32`: the
//! board owns startup, clocks and the linker script; this HAL provides
//! checked peripheral drivers over `at32f4xx-pac`.
//!
//! Peripheral support (growing):
//! - **Clocks:** HICK/HEXT PLL startup with checked dividers and bus limits.
//! - **Time:** optional exact 1 kHz SysTick driver with a multi-deadline
//!   queue (disable the default feature to supply your own Embassy time
//!   driver).
//! - **GPIO:** pin tokens with digital/analog/alternate modes over the
//!   F1-style configuration registers.
//!
//! - **EXINT:** async edge-triggered inputs with a two-flag latch design
//!   (ported from the HC32 driver), grouped NVIC vectors serviced through
//!   `on_interrupt`/`on_interrupts_in`.
//! - **I²C:** async master (I2C1 today) with vendor timing formulas and the
//!   vendor read endgame (1/2/3-byte ACK/POS cases).
//!
//! Not yet implemented: SPI, ADC, DMA, timers/PWM, USB device, flash
//! programming.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

pub use at32f4xx_pac as pac;

pub mod clocks;
pub mod exti;
pub mod gpio;
pub mod i2c;
#[cfg(feature = "time-driver")]
pub mod time_driver;

/// Initialize SysTick after the board configures its clocks.
///
/// Prefer [`time_driver::init`] with an owned `SYST` token and checked
/// errors. The application must forward its SysTick exception to
/// [`time_driver::systick_tick`]. No other Embassy time driver may be linked.
///
/// # Panics
/// Panics if already initialized or if HCLK cannot produce an exact 1 kHz
/// tick.
///
/// # Safety
/// Call once at startup, before other drivers run. The caller must
/// exclusively own SysTick and supply the configured HCLK in `clocks.hclk_hz`.
#[cfg(feature = "time-driver")]
pub unsafe fn init(clocks: clocks::Clocks) {
    // SAFETY: The caller transfers exclusive SysTick ownership to the driver.
    let systick = unsafe { cortex_m::Peripherals::steal() }.SYST;
    // Invariant: legacy startup must provide an exact supported clock and run once.
    time_driver::init(systick, clocks.hclk_hz)
        .expect("SysTick initialization requires an exact 1 kHz clock and exclusive startup");
}
