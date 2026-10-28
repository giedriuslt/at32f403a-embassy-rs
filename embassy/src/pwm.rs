//! PWM output through general-purpose timers, scoped to TMR3 (the
//! backlight-class use of the KWS-X1 firmware).
//!
//! The time base follows the vendor `tmr_base_init` example: `DIV` holds
//! the prescaler minus one, `PR` the period minus one (both whole-value
//! registers), and an overflow software event reloads them immediately.
//! Channels use output-control PWM mode A (output active while the counter
//! runs below the channel data value), active-high, with the compare
//! buffer (preload) enabled — the `pwm_output_tmr3` example's settings.
//!
//! One PAC gap: the SVD leaves the capture/compare-mode registers `CM1`
//! (offset 0x18) and `CM2` (0x1c) unmapped, so the mode bits are written
//! through a raw volatile pointer to `TMR3 + 0x18/0x1c` under a critical
//! section, with the bit positions stated next to each write (vendor
//! `tmr_output_channel_config`, `c1octrl`/`c2octrl` and their buffer bits).

use crate::pac::at32f403a::tmr3::RegisterBlock;
use crate::pac::at32f403a::tmr3::ctrl1::{PrbenwWO, TmrenwWO};
use crate::pac::at32f403a::{CRM, TMR3};

/// TMR3 register base; `CM1`/`CM2` sit 0x18/0x1c into it.
const TMR3_BASE: usize = 0x4000_0400;
/// PWM mode A in the three `CxOCTRL` bits.
const PWM_MODE_A: u32 = 0b110;

/// Timer output channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    C1,
    C2,
    C3,
    C4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("frequency must divide the timer clock with a period of 2..=65536 ticks")]
    Frequency,
    #[error("duty must be within 0..=period")]
    Duty,
}

/// The `(div, pr)` register pair for a target frequency: `div` is the
/// written `DIV` value (prescaler minus one), `pr` the written `PR` value
/// (period minus one). Pure, so it can be tested on the host.
pub fn compute_base(timer_hz: u32, freq_hz: u32) -> Result<(u16, u16), Error> {
    if freq_hz == 0 {
        return Err(Error::Frequency);
    }
    // Smallest prescaler that brings the period within 16 bits.
    let mut div: u32 = 0;
    while div <= u16::MAX as u32 {
        let period = timer_hz / ((div + 1) * freq_hz);
        if period >= 2 && period <= u16::MAX as u32 + 1 {
            // Exact when possible; otherwise the closest slower period.
            return Ok((div as u16, (period - 1) as u16));
        }
        if period > u16::MAX as u32 + 1 {
            div += 1;
            continue;
        }
        // period < 2: the frequency is too low to represent distinctly.
        return Err(Error::Frequency);
    }
    Err(Error::Frequency)
}

/// PWM over TMR3.
pub struct Pwm {
    tmr: &'static RegisterBlock,
    period: u16,
}

impl Pwm {
    /// Claim TMR3 and program `freq_hz` from its input clock.
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: the caller claims TMR3 once per boot.
    /// Pin muxing for the output channels is the board's job.
    pub unsafe fn new_tmr3(timer_hz: u32, freq_hz: u32) -> Result<Self, Error> {
        let (div, pr) = compute_base(timer_hz, freq_hz)?;
        // SAFETY: forwarded from this constructor's exclusive-startup contract.
        let crm = unsafe { CRM::steal() };
        critical_section::with(|_| {
            crm.apb1en().modify(|_, w| w.tmr3().bit(true));
        });
        // SAFETY: the peripheral's static register block, exclusively owned
        // by this constructor's contract.
        let tmr = unsafe { &*TMR3::ptr() };
        // SAFETY: both values fit their 16-bit registers by construction.
        unsafe {
            tmr.div().write(|w| w.bits(div));
            tmr.pr().write(|w| w.bits(pr));
        }
        // Reload DIV/PR immediately (vendor tmr_base_init's overflow event).
        tmr.swevt().write(|w| w.ovfswtr().set_bit());
        // Buffer the period for glitch-free duty changes.
        tmr.ctrl1()
            .modify(|_, w| w.prben().variant(PrbenwWO::Enable));
        Ok(Self { tmr, period: pr })
    }

    fn cm_address(channel: Channel) -> usize {
        match channel {
            Channel::C1 | Channel::C2 => TMR3_BASE + 0x18, // CM1
            Channel::C3 | Channel::C4 => TMR3_BASE + 0x1c, // CM2
        }
    }

    fn cm_shift(channel: Channel) -> u32 {
        match channel {
            Channel::C1 | Channel::C3 => 4,  // c{octrl} bits [6:4]
            Channel::C2 | Channel::C4 => 12, // bits [14:12]
        }
    }

    /// Enable one channel's PWM output and set its duty numerator (of
    /// `period + 1` ticks). `duty = 0` holds the line low; the full period
    /// value holds it high.
    pub fn enable(&mut self, channel: Channel, duty: u16) -> Result<(), Error> {
        if duty > self.period {
            return Err(Error::Duty);
        }
        // Mode and preload through the unmapped CM registers.
        let address = Self::cm_address(channel) as *mut u32;
        let shift = Self::cm_shift(channel);
        critical_section::with(|_| {
            // SAFETY: TMR3's CM1/CM2 registers, owned through this driver;
            // the mode field, its preload bit and nothing else is touched.
            unsafe {
                let cm = address.read_volatile();
                let mode = PWM_MODE_A << shift;
                let preload = 1 << (shift - 1);
                let mask = 0b111 << shift | preload;
                address.write_volatile((cm & !mask) | mode | preload);
            }
        });

        let index = match channel {
            Channel::C1 => 0,
            Channel::C2 => 1,
            Channel::C3 => 2,
            Channel::C4 => 3,
        };
        let cdt = self.tmr.cdt(index);
        // SAFETY: 16-bit compare value in a whole-value register.
        cdt.write(|w| unsafe { w.bits(duty) });

        self.tmr.cctrl().modify(|_, w| match channel {
            Channel::C1 => w.c1en().enable(),
            Channel::C2 => w.c2en().enable(),
            Channel::C3 => w.c3en().enable(),
            Channel::C4 => w.c4en().enable(),
        });
        Ok(())
    }

    /// Change an enabled channel's duty (buffered; takes effect at the
    /// next period).
    pub fn set_duty(&mut self, channel: Channel, duty: u16) -> Result<(), Error> {
        if duty > self.period {
            return Err(Error::Duty);
        }
        let index = match channel {
            Channel::C1 => 0,
            Channel::C2 => 1,
            Channel::C3 => 2,
            Channel::C4 => 3,
        };
        let cdt = self.tmr.cdt(index);
        // SAFETY: 16-bit compare value in a whole-value register.
        cdt.write(|w| unsafe { w.bits(duty) });
        Ok(())
    }

    /// The period value in ticks (the register's `PR` value).
    pub fn period(&self) -> u16 {
        self.period
    }

    /// Start counting (all enabled channels run).
    pub fn start(&mut self) {
        self.tmr
            .ctrl1()
            .modify(|_, w| w.tmren().variant(TmrenwWO::Enable));
    }

    pub fn stop(&mut self) {
        self.tmr
            .ctrl1()
            .modify(|_, w| w.tmren().variant(TmrenwWO::Disable));
    }
}

#[cfg(test)]
mod tests;
