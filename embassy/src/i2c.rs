//! Async I²C master (one peripheral instance at a time) for the AT32F403A's
//! STM32F1-style I²C blocks.
//!
//! Timing follows the vendor `i2c_init` formulas: `CLKFREQ = PCLK / 1 MHz`,
//! standard mode `SPEED = PCLK / (2 × Hz)` (minimum 4), fast mode
//! `SPEED = PCLK / (3 × Hz)` at duty 2:1 or `PCLK / (25 × Hz)` at duty 16:9,
//! rise time `freq_MHz + 1` (standard) or `freq_MHz × 0.3 + 1` (fast, up to
//! 400 kHz). I2C1/I2C2 sit on APB1; **I2C3 sits on APB2** (vendor
//! `i2c_init`), so its configuration takes the APB2 clock.
//!
//! Transaction sequences follow the vendor `i2c_application` library:
//! START → address (wait `STARTF`, then `ADDR7F`, ACK-checked) → the
//! 1/2/3-byte read endgames with ACK and POS (`MACKCTRL`) → STOP. Like the
//! vendor, every receive begins by resetting the master ACK state (POS to
//! the current byte, ACK enabled) — the peripheral latches no per-transfer
//! defaults, so the endgame state of one read must not leak into the next. The
//! `ADDR7F` flag is cleared by reading STS1 then STS2, as the F1-style
//! hardware requires.
//!
//! The driver is interrupt-driven: the board's `I2Cn_EVT` and `I2Cn_ERR`
//! handlers call [`on_interrupt`], which wakes the task waiting for the next
//! status change. Every wait is bounded by an Embassy timeout; a transfer
//! that cannot progress latches its error and stops generating START/STOP.

use embassy_time::Duration;

use crate::pac::at32f403a::i2c1::RegisterBlock;
use crate::pac::at32f403a::{CRM, I2C1, I2C2};

/// Default per-step deadline; generous for a 100 kHz bus with clock
/// stretching (a stuck bus fails at this bound rather than hanging).
const STEP_TIMEOUT: Duration = Duration::from_millis(50);

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("the peripheral did not reach the expected state in time")]
    Timeout,
    #[error("arbitration lost")]
    ArbitrationLoss,
    #[error("bus error (misplaced START/STOP, no external pull-ups)")]
    Bus,
    #[error("data overrun/underrun")]
    Overrun,
    #[error("SMBus timeout")]
    ClockTimeout,
    #[error("the address or data was not acknowledged")]
    NoAcknowledge(NoAcknowledgeSource),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoAcknowledgeSource {
    Address,
    Data,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Self::NoAcknowledge(source) => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(match source {
                    NoAcknowledgeSource::Address => embedded_hal::i2c::NoAcknowledgeSource::Address,
                    NoAcknowledgeSource::Data => embedded_hal::i2c::NoAcknowledgeSource::Data,
                })
            }
            Self::ArbitrationLoss => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Self::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            _ => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

pub type Result<T> = core::result::Result<T, Error>;

/// Fast-mode duty cycle (vendor `i2c_fsmode_duty_cycle_type`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Duty {
    /// SCL high:low = 1:2 (`SPEED = PCLK / (3 × Hz)`).
    Duty2To1,
    /// SCL high:low = 9:16 (`SPEED = PCLK / (25 × Hz)`).
    Duty16To9,
}

/// Bus speed selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    /// Up to 100 kHz.
    Standard,
    /// Up to 400 kHz with the given duty cycle.
    Fast(Duty),
}

impl Speed {
    fn hz(self) -> u32 {
        match self {
            Self::Standard => 100_000,
            Self::Fast(_) => 400_000,
        }
    }
}

/// Computed `CLKCTRL`/`TMRISE` values for one configuration; pure, so it
/// can be tested on the host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
    /// `CTRL2.CLKFREQ`: the peripheral's PCLK in MHz.
    pub clkfreq: u8,
    /// `CLKCTRL.SPEED`: the divider value.
    pub speed: u16,
    /// `CLKCTRL.SPEEDMODE`: fast mode.
    pub fast: bool,
    /// `CLKCTRL.DUTYMODE`: 16:9 duty in fast mode.
    pub duty_16_9: bool,
    /// `TMRISE.RISETIME`.
    pub risetime: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ConfigError {
    #[error("PCLK must be a whole number of MHz")]
    ClockNotMhz,
    #[error("the computed CLKCTRL.SPEED value does not fit 12 bits")]
    SpeedRange,
    #[error("PCLK is too high for this bus speed (SPEED below the minimum)")]
    SpeedTooLow,
}

/// Compute the F1-style timing registers for `bus_hz` from a peripheral
/// clocked at `pclk_hz`, exactly as the vendor `i2c_init` does.
pub fn compute_timing(pclk_hz: u32, bus: Speed) -> core::result::Result<Timing, ConfigError> {
    let freq_mhz = pclk_hz / 1_000_000;
    if freq_mhz == 0 || !pclk_hz.is_multiple_of(1_000_000) || freq_mhz > u8::MAX as u32 {
        return Err(ConfigError::ClockNotMhz);
    }
    let freq_mhz = freq_mhz as u8;
    match bus {
        Speed::Standard => {
            // Standard mode: SPEED = PCLK / (2 × Hz), minimum 4.
            let speed = (pclk_hz / (bus.hz() * 2)) as u16;
            let speed = if speed < 4 { 4 } else { speed };
            if speed > 0x0FFF {
                return Err(ConfigError::SpeedRange);
            }
            let rise = (u16::from(freq_mhz) + 1).min(0x3F) as u8;
            Ok(Timing {
                clkfreq: freq_mhz,
                speed,
                fast: false,
                duty_16_9: false,
                risetime: rise,
            })
        }
        Speed::Fast(duty) => {
            let (div, duty_16_9) = match duty {
                Duty::Duty2To1 => (3u32, false),
                Duty::Duty16To9 => (25u32, true),
            };
            let mut speed = (pclk_hz / (bus.hz() * div)) as u16;
            if speed == 0 {
                speed = 1;
            }
            if speed > 0x0FFF {
                return Err(ConfigError::SpeedRange);
            }
            // Fast mode rise time for <= 400 kHz.
            let rise = ((u16::from(freq_mhz) * 300) / 1000 + 1).min(0x3F) as u8;
            Ok(Timing {
                clkfreq: freq_mhz,
                speed,
                fast: true,
                duty_16_9,
                risetime: rise,
            })
        }
    }
}

/// Which APB bus a peripheral instance hangs from (vendor `i2c_init`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Instance {
    /// I2C1 and I2C2: APB1.
    Apb1,
    /// I2C3: APB2.
    Apb2,
}

/// The async I²C master over one peripheral instance.
pub struct I2c {
    i2c: &'static RegisterBlock,
    /// APB prescaler bit position is per-instance; clock enable is done in `new`.
    _instance: Instance,
}

impl I2c {
    /// Create the driver over `I2C1` and program its timing.
    ///
    /// # Safety
    ///
    /// The caller must exclusively own the peripheral and its `I2C1_EVT` /
    /// `I2C1_ERR` vectors (both routed to handlers that call
    /// [`on_interrupt`]) before any transfer. Pin muxing is the board's job.
    pub unsafe fn new_i2c1(pclk_hz: u32, speed: Speed) -> core::result::Result<Self, ConfigError> {
        let timing = compute_timing(pclk_hz, speed)?;
        // SAFETY: clock setup under this constructor's exclusive-ownership contract.
        let crm = unsafe { CRM::steal() };
        critical_section::with(|_| {
            crm.apb1en().modify(|_, w| w.i2c1().bit(true));
        });
        // SAFETY: the peripheral's static register block, exclusively owned by
        // this constructor's contract.
        let i2c = unsafe { &*I2C1::ptr() };
        let this = Self {
            i2c,
            _instance: Instance::Apb1,
        };
        this.apply_timing(timing);
        Ok(this)
    }

    /// Create the driver over `I2C2` (APB1) and program its timing.
    ///
    /// # Safety
    ///
    /// Same contract as [`Self::new_i2c1`], with the `I2C2_EVT` / `I2C2_ERR`
    /// vectors.
    pub unsafe fn new_i2c2(pclk_hz: u32, speed: Speed) -> core::result::Result<Self, ConfigError> {
        let timing = compute_timing(pclk_hz, speed)?;
        // SAFETY: clock setup under this constructor's exclusive-ownership contract.
        let crm = unsafe { CRM::steal() };
        critical_section::with(|_| {
            crm.apb1en().modify(|_, w| w.i2c2().bit(true));
        });
        // SAFETY: the peripheral's static register block, exclusively owned by
        // this constructor's contract.
        let i2c = unsafe { &*I2C2::ptr() };
        let this = Self {
            i2c,
            _instance: Instance::Apb1,
        };
        this.apply_timing(timing);
        Ok(this)
    }

    fn apply_timing(&self, timing: Timing) {
        self.i2c
            .ctrl1()
            .modify(|_, w| w.i2cen().variant(I2cenwWO::Disable));
        self.i2c
            .ctrl2()
            .modify(|_, w| unsafe { w.clkfreq().bits(timing.clkfreq) });
        self.i2c.clkctrl().modify(|_, w| {
            unsafe { w.speed().bits(timing.speed) };
            w.speedmode().bit(timing.fast);
            w.dutymode().bit(timing.duty_16_9)
        });
        self.i2c
            .tmrise()
            .modify(|_, w| unsafe { w.risetime().bits(timing.risetime) });
        self.i2c
            .ctrl1()
            .modify(|_, w| w.i2cen().variant(I2cenwWO::Enable));
        // Waits poll STS on an embassy timer (the SPI-review lesson: never
        // depend on interrupt delivery), so the peripheral's interrupt
        // sources stay off; nothing is wired to I2Cn_EVT/I2Cn_ERR.
    }

    fn sts1(&self) -> crate::pac::at32f403a::i2c1::sts1::R {
        self.i2c.sts1().read()
    }

    /// Latched error from the status registers, or the error bits' absence.
    fn check_errors(&self, sts: &crate::pac::at32f403a::i2c1::sts1::R) -> Option<Error> {
        if sts.ackfail().bit() {
            Some(Error::NoAcknowledge(NoAcknowledgeSource::Data))
        } else if sts.arlost().bit() {
            Some(Error::ArbitrationLoss)
        } else if sts.buserr().bit() {
            Some(Error::Bus)
        } else if sts.ouf().bit() {
            Some(Error::Overrun)
        } else if sts.tmout().bit() {
            Some(Error::ClockTimeout)
        } else {
            None
        }
    }

    fn clear_errors(&self) {
        self.i2c.sts1().write(|w| {
            w.ackfail().clear();
            w.arlost().clear();
            w.buserr().clear();
            w.ouf().clear();
            w.tmout().clear();
            w
        });
    }

    fn stop(&self) {
        self.i2c
            .ctrl1()
            .modify(|_, w| w.genstop().variant(GENSTOP_A::Stop));
    }

    /// Wait until `ready` holds, returning its error if a status error
    /// latches first. The waker is registered before the flag check, so an
    /// interrupt between the two only re-polls.
    /// Poll until `ready` holds or the deadline passes. Timer-backed (a
    /// 100 us tick) with no interrupt dependency: a masked or never-firing
    /// I2C interrupt can never hang a transfer — the same lesson the SPI
    /// driver learned in review. On timeout a STOP is issued so the bus
    /// is released for the next attempt.
    async fn wait_until(
        &self,
        ready: impl Fn(&crate::pac::at32f403a::i2c1::sts1::R) -> bool,
    ) -> Result<()> {
        let deadline = embassy_time::Instant::now() + STEP_TIMEOUT;
        loop {
            let sts = self.sts1();
            if let Some(error) = self.check_errors(&sts) {
                self.stop();
                self.clear_errors();
                return Err(error);
            }
            if (ready)(&sts) {
                return Ok(());
            }
            if embassy_time::Instant::now() >= deadline {
                self.stop();
                return Err(Error::Timeout);
            }
            embassy_time::Timer::after_micros(100).await;
        }
    }

    /// Send START and the address byte, leaving ADDR7F pending. The caller
    /// clears it via [`Self::clear_addr`], which applies the endgame ACK
    /// configuration in the vendor order.
    async fn start_address(&self, address: u8, read: bool) -> Result<()> {
        // Clear errors from any previous transfer before judging this one.
        self.clear_errors();
        // Vendor receive prologue (i2c_memory_read/i2c_master_receive): POS
        // back to "current byte", ACK enabled. The previous receive's
        // endgame leaves ACKEN=0/MACKCTRL=Next behind; if that leaks into
        // this transfer, the first data byte gets NACKed, the slave
        // releases SDA mid-read, and trailing bytes read as 0xFF garbage
        // (silently — a master-side NACK raises no ACKFAIL).
        self.i2c
            .ctrl1()
            .modify(|_, w| w.mackctrl().variant(MACKCTRL_A::Current));
        self.i2c
            .ctrl1()
            .modify(|_, w| w.acken().variant(AckenwWO::Enable));
        self.i2c
            .ctrl1()
            .modify(|_, w| w.genstart().variant(GENSTART_A::Start));
        self.wait_until(|sts| sts.startf().bit()).await?;
        let byte = (address << 1) | u8::from(read);
        self.i2c.dt().write(|w| unsafe { w.dt().bits(byte) });
        self.wait_until(|sts| sts.addr7f().bit()).await?;
        Ok(())
    }

    /// Clear ADDR7F (F1-style: read STS1, then STS2). Clearing ADDR ends
    /// the clock stretch and starts the first data byte, so for a receive
    /// the endgame ACK configuration goes around the clear in the vendor
    /// `i2c_memory_read` order: MACKCTRL/ACKEN first, ADDR clear, then the
    /// 2-byte ACKEN.
    fn clear_addr(&self, receive_len: Option<usize>) {
        match receive_len {
            Some(1) => {
                // The only byte is NACKed: disable ACK before the clear.
                self.ack_set(false);
            }
            Some(2) => {
                // POS=Next defers the coming ACKEN=0 to byte 2.
                self.i2c
                    .ctrl1()
                    .modify(|_, w| w.mackctrl().variant(MACKCTRL_A::Next));
            }
            _ => {
                self.ack_set(true);
            }
        }
        let _ = self.sts1();
        let _ = self.i2c.sts2().read();
        if receive_len == Some(2) {
            // Byte 1's ACK is now pipelined; NACK byte 2.
            self.ack_set(false);
        }
    }

    fn ack_set(&self, enable: bool) {
        self.i2c.ctrl1().modify(|_, w| {
            w.acken().variant(if enable {
                AckenwWO::Enable
            } else {
                AckenwWO::Disable
            })
        });
    }

    async fn write_bytes(&self, bytes: &[u8], last_in_transaction: bool) -> Result<()> {
        for &byte in bytes {
            self.wait_until(|sts| sts.tdbe().bit()).await?;
            self.i2c.dt().write(|w| unsafe { w.dt().bits(byte) });
        }
        if last_in_transaction {
            self.wait_until(|sts| sts.tdc().bit()).await?;
            self.stop();
        }
        Ok(())
    }

    /// The vendor read endgame: exact ACK/POS handling for the last three
    /// bytes (`i2c_application.c` receive sequence). The ACK configuration
    /// around the ADDR clear already happened in [`Self::clear_addr`].
    async fn read_bytes(&self, buffer: &mut [u8]) -> Result<()> {
        // A single byte is NACKed and stopped before it arrives (vendor
        // size==1 flow: ACK off, ADDR clear, STOP, then wait for data).
        if buffer.len() == 1 {
            self.stop();
        }

        let mut remaining = buffer.len();
        let mut index = 0;
        while remaining > 0 {
            if remaining == 1 {
                self.wait_until(|sts| sts.rdbf().bit()).await?;
                buffer[index] = self.i2c.dt().read().dt().bits();
                remaining -= 1;
            } else if remaining == 2 {
                self.wait_until(|sts| sts.tdc().bit()).await?;
                self.stop();
                buffer[index] = self.i2c.dt().read().dt().bits();
                index += 1;
                buffer[index] = self.i2c.dt().read().dt().bits();
                remaining = 0;
            } else if remaining == 3 {
                self.wait_until(|sts| sts.tdc().bit()).await?;
                // NACK from the next received byte on.
                self.ack_set(false);
                buffer[index] = self.i2c.dt().read().dt().bits();
                index += 1;
                self.wait_until(|sts| sts.tdc().bit()).await?;
                self.stop();
                buffer[index] = self.i2c.dt().read().dt().bits();
                index += 1;
                buffer[index] = self.i2c.dt().read().dt().bits();
                remaining = 0;
            } else {
                self.wait_until(|sts| sts.rdbf().bit()).await?;
                buffer[index] = self.i2c.dt().read().dt().bits();
                index += 1;
                remaining -= 1;
            }
        }
        Ok(())
    }
}

impl embedded_hal::i2c::ErrorType for I2c {
    type Error = Error;
}

impl embedded_hal_async::i2c::I2c for I2c {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<()> {
        let last = operations.len().saturating_sub(1);
        for (i, operation) in operations.iter_mut().enumerate() {
            let is_last = i == last;
            match operation {
                embedded_hal_async::i2c::Operation::Write(buffer) => {
                    self.start_address(address, false).await?;
                    self.clear_addr(None);
                    self.write_bytes(buffer, is_last).await?;
                }
                embedded_hal_async::i2c::Operation::Read(buffer) => {
                    self.start_address(address, true).await?;
                    self.clear_addr(Some(buffer.len()));
                    self.read_bytes(buffer).await?;
                    // Single-byte reads already generated STOP inside the
                    // endgame; longer ones stopped after the final read.
                    let _ = is_last;
                }
            }
        }
        // A read that did not end in STOP (a following WriteRead) leaves the
        // bus held; every path here stops, so nothing to do.
        Ok(())
    }
}

// The register enums re-exported for the driver's own use.
use crate::pac::at32f403a::i2c1::ctrl1::{AckenwWO, GENSTART_A, GENSTOP_A, I2cenwWO, MACKCTRL_A};

#[cfg(test)]
mod tests;
