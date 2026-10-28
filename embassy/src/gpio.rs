//! GPIO for the AT32F403A: pin tokens with digital input/output, analog and
//! alternate-function modes over the F1-style CFGLR/CFGHR configuration
//! registers.
//!
//! Per-pin CFGR nibble encodings (vendor `gpio_init`, `at32f403a_407_gpio.c`):
//! mode bits [1:0] select input (00) or an output drive strength (01
//! "stronger", 10 "moderate"); config bits [3:2] select, per direction:
//! input - analog (00), floating (01), pull up/down (10, level latched
//! through the set/clear registers); output - GPIO push-pull (00), GPIO
//! open-drain (01), mux push-pull (10), mux open-drain (11). The output data
//! register is write-only; levels are set and cleared atomically through the
//! SCR register's `IOSB`/`IOCB` bits, exactly as the vendor library does.
//!
//! Pin construction is `unsafe` with an exclusive-startup contract: the
//! caller claims each (port, number) once and owns it afterwards, mirroring
//! the legacy constructors of `embassy-hc32`. Construction enables the
//! port's CRM clock.

use at32f4xx_pac::at32f403a::gpioa::RegisterBlock;
use at32f4xx_pac::at32f403a::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};

/// Output drive strength, mapped to the CFGR mode bits for outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drive {
    /// Vendor `GPIO_DRIVE_STRENGTH_STRONGER` (mode 01).
    Stronger,
    /// Vendor `GPIO_DRIVE_STRENGTH_MODERATE` (mode 10).
    Moderate,
}

impl Drive {
    const fn mode_bits(self) -> u8 {
        match self {
            Self::Stronger => 0b01,
            Self::Moderate => 0b10,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    A,
    B,
    C,
    D,
    E,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("pin number must be 0..=15")]
    InvalidPin,
}

/// A claimed pin location; mode constructors configure and consume it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pin {
    pub port: Port,
    pub num: u8,
}

impl Pin {
    /// Claim a pin location.
    ///
    /// # Safety
    ///
    /// Exclusive-startup contract: each (port, number) may be claimed once
    /// per boot, and must not belong to a peripheral driver or another task.
    /// The port's CRM clock is enabled here.
    pub unsafe fn new(port: Port, num: u8) -> Result<Self, Error> {
        if num > 15 {
            return Err(Error::InvalidPin);
        }
        // SAFETY: forwarded from this constructor's exclusive-startup contract.
        unsafe { enable_port_clock(port) };
        Ok(Self { port, num })
    }

    /// The port's register block. All GPIO blocks share one layout.
    fn gpio(self) -> &'static RegisterBlock {
        // SAFETY: stealing a singleton under the pin-ownership contract; the
        // transmuted reference is layout-identical across GPIO ports.
        unsafe {
            let block = match self.port {
                Port::A => &*GPIOA::steal(),
                Port::B => &*GPIOB::steal(),
                Port::C => &*GPIOC::steal(),
                Port::D => &*GPIOD::steal(),
                Port::E => &*GPIOE::steal(),
            };
            core::mem::transmute::<
                &'_ at32f4xx_pac::at32f403a::gpiob::RegisterBlock,
                &'static RegisterBlock,
            >(core::mem::transmute::<
                &'_ RegisterBlock,
                &'_ at32f4xx_pac::at32f403a::gpiob::RegisterBlock,
            >(block))
        }
    }

    /// Configure digital input, analog input, or an output/mux mode.
    fn configure(self, mode: u8, cfg: u8) {
        let gpio = self.gpio();
        critical_section::with(|_| {
            if self.num < 8 {
                gpio.cfglr()
                    .modify(|_, w| cfg_field_low!(w, self.num, mode, cfg));
            } else {
                gpio.cfghr()
                    .modify(|_, w| cfg_field_high!(w, self.num, mode, cfg));
            }
        });
    }

    /// Atomically drive the pin's latched output level (also selects the
    /// pull direction while the pin is a pull up/down input).
    fn set_level(self, high: bool) {
        let gpio = self.gpio();
        critical_section::with(|_| {
            gpio.scr().write(|w| scr_field!(w, self.num, high));
        });
    }

    fn read_level(self) -> bool {
        let gpio = self.gpio();
        critical_section::with(|_| idt_field!(gpio, self.num))
    }
}

/// Write one pin's CFGR fields. CFGLR exposes pins 0-7 (`iomc0..7`), CFGHR
/// pins 8-15 (`iomc8..15`, absolute numbering); the caller passes the
/// absolute pin number and picks the matching register.
macro_rules! cfg_field_low {
    ($w:ident, $pin:expr, $mode:expr, $cfg:expr) => {
        match $pin {
            0 => {
                unsafe { $w.iomc0().bits($mode) };
                unsafe { $w.iofc0().bits($cfg) }
            }
            1 => {
                unsafe { $w.iomc1().bits($mode) };
                unsafe { $w.iofc1().bits($cfg) }
            }
            2 => {
                unsafe { $w.iomc2().bits($mode) };
                unsafe { $w.iofc2().bits($cfg) }
            }
            3 => {
                unsafe { $w.iomc3().bits($mode) };
                unsafe { $w.iofc3().bits($cfg) }
            }
            4 => {
                unsafe { $w.iomc4().bits($mode) };
                unsafe { $w.iofc4().bits($cfg) }
            }
            5 => {
                unsafe { $w.iomc5().bits($mode) };
                unsafe { $w.iofc5().bits($cfg) }
            }
            6 => {
                unsafe { $w.iomc6().bits($mode) };
                unsafe { $w.iofc6().bits($cfg) }
            }
            7 => {
                unsafe { $w.iomc7().bits($mode) };
                unsafe { $w.iofc7().bits($cfg) }
            }
            _ => $w,
        }
    };
}
use cfg_field_low;
macro_rules! cfg_field_high {
    ($w:ident, $pin:expr, $mode:expr, $cfg:expr) => {
        match $pin {
            8 => {
                unsafe { $w.iomc8().bits($mode) };
                unsafe { $w.iofc8().bits($cfg) }
            }
            9 => {
                unsafe { $w.iomc9().bits($mode) };
                unsafe { $w.iofc9().bits($cfg) }
            }
            10 => {
                unsafe { $w.iomc10().bits($mode) };
                unsafe { $w.iofc10().bits($cfg) }
            }
            11 => {
                unsafe { $w.iomc11().bits($mode) };
                unsafe { $w.iofc11().bits($cfg) }
            }
            12 => {
                unsafe { $w.iomc12().bits($mode) };
                unsafe { $w.iofc12().bits($cfg) }
            }
            13 => {
                unsafe { $w.iomc13().bits($mode) };
                unsafe { $w.iofc13().bits($cfg) }
            }
            14 => {
                unsafe { $w.iomc14().bits($mode) };
                unsafe { $w.iofc14().bits($cfg) }
            }
            15 => {
                unsafe { $w.iomc15().bits($mode) };
                unsafe { $w.iofc15().bits($cfg) }
            }
            _ => $w,
        }
    };
}
use cfg_field_high;

/// Read one pin's input level through the per-bit IDT readers.
macro_rules! idt_field {
    ($gpio:expr, $pin:expr) => {
        match $pin {
            0 => $gpio.idt().read().idt0().bit(),
            1 => $gpio.idt().read().idt1().bit(),
            2 => $gpio.idt().read().idt2().bit(),
            3 => $gpio.idt().read().idt3().bit(),
            4 => $gpio.idt().read().idt4().bit(),
            5 => $gpio.idt().read().idt5().bit(),
            6 => $gpio.idt().read().idt6().bit(),
            7 => $gpio.idt().read().idt7().bit(),
            8 => $gpio.idt().read().idt8().bit(),
            9 => $gpio.idt().read().idt9().bit(),
            10 => $gpio.idt().read().idt10().bit(),
            11 => $gpio.idt().read().idt11().bit(),
            12 => $gpio.idt().read().idt12().bit(),
            13 => $gpio.idt().read().idt13().bit(),
            14 => $gpio.idt().read().idt14().bit(),
            15 => $gpio.idt().read().idt15().bit(),
            _ => false,
        }
    };
}
use idt_field;

/// Build the SCR set/clear field write for one pin index: `IOSBn` sets the
/// level, `IOCBn` (bit 16+n) clears it.
macro_rules! scr_field {
    ($w:ident, $pin:expr, $high:expr) => {
        match $pin {
            0 => {
                if $high {
                    $w.iosb0().bit(true)
                } else {
                    $w.iocb0().bit(true)
                }
            }
            1 => {
                if $high {
                    $w.iosb1().bit(true)
                } else {
                    $w.iocb1().bit(true)
                }
            }
            2 => {
                if $high {
                    $w.iosb2().bit(true)
                } else {
                    $w.iocb2().bit(true)
                }
            }
            3 => {
                if $high {
                    $w.iosb3().bit(true)
                } else {
                    $w.iocb3().bit(true)
                }
            }
            4 => {
                if $high {
                    $w.iosb4().bit(true)
                } else {
                    $w.iocb4().bit(true)
                }
            }
            5 => {
                if $high {
                    $w.iosb5().bit(true)
                } else {
                    $w.iocb5().bit(true)
                }
            }
            6 => {
                if $high {
                    $w.iosb6().bit(true)
                } else {
                    $w.iocb6().bit(true)
                }
            }
            7 => {
                if $high {
                    $w.iosb7().bit(true)
                } else {
                    $w.iocb7().bit(true)
                }
            }
            8 => {
                if $high {
                    $w.iosb8().bit(true)
                } else {
                    $w.iocb8().bit(true)
                }
            }
            9 => {
                if $high {
                    $w.iosb9().bit(true)
                } else {
                    $w.iocb9().bit(true)
                }
            }
            10 => {
                if $high {
                    $w.iosb10().bit(true)
                } else {
                    $w.iocb10().bit(true)
                }
            }
            11 => {
                if $high {
                    $w.iosb11().bit(true)
                } else {
                    $w.iocb11().bit(true)
                }
            }
            12 => {
                if $high {
                    $w.iosb12().bit(true)
                } else {
                    $w.iocb12().bit(true)
                }
            }
            13 => {
                if $high {
                    $w.iosb13().bit(true)
                } else {
                    $w.iocb13().bit(true)
                }
            }
            14 => {
                if $high {
                    $w.iosb14().bit(true)
                } else {
                    $w.iocb14().bit(true)
                }
            }
            15 => {
                if $high {
                    $w.iosb15().bit(true)
                } else {
                    $w.iocb15().bit(true)
                }
            }
            _ => $w,
        }
    };
}
use scr_field;

unsafe fn enable_port_clock(port: Port) {
    // SAFETY: forwarded from Pin::new's exclusive-startup contract.
    let crm = unsafe { at32f4xx_pac::at32f403a::CRM::steal() };
    crm.apb2en().modify(|_, w| match port {
        Port::A => w.gpioa().bit(true),
        Port::B => w.gpiob().bit(true),
        Port::C => w.gpioc().bit(true),
        Port::D => w.gpiod().bit(true),
        Port::E => w.gpioe().bit(true),
    });
}

/// Digital input. The pull direction is part of the constructor because it
/// is latched through the set/clear registers.
pub struct InputPin {
    pin: Pin,
}

impl InputPin {
    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new_floating(port: Port, num: u8) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.configure(0b00, 0b01); // input, floating
        Ok(Self { pin })
    }

    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new_pull_up(port: Port, num: u8) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.set_level(true);
        pin.configure(0b00, 0b10); // input, pull up/down with latch high
        Ok(Self { pin })
    }

    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new_pull_down(port: Port, num: u8) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.set_level(false);
        pin.configure(0b00, 0b10);
        Ok(Self { pin })
    }

    pub fn is_high(&self) -> bool {
        self.pin.read_level()
    }

    pub fn is_low(&self) -> bool {
        !self.pin.read_level()
    }

    pub fn degrade(self) -> Pin {
        self.pin
    }
}

impl embedded_hal::digital::ErrorType for InputPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for InputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(InputPin::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(InputPin::is_low(self))
    }
}

/// Digital output, GPIO push-pull or open-drain.
pub struct OutputPin {
    pin: Pin,
}

impl OutputPin {
    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new_push_pull(
        port: Port,
        num: u8,
        level: bool,
        drive: Drive,
    ) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.set_level(level);
        pin.configure(drive.mode_bits(), 0b00); // GPIO push-pull
        Ok(Self { pin })
    }

    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new_open_drain(
        port: Port,
        num: u8,
        level: bool,
        drive: Drive,
    ) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.set_level(level);
        pin.configure(drive.mode_bits(), 0b01); // GPIO open-drain
        Ok(Self { pin })
    }

    pub fn set_high(&mut self) {
        self.pin.set_level(true);
    }

    pub fn set_low(&mut self) {
        self.pin.set_level(false);
    }

    pub fn is_set(&self) -> bool {
        // ODT is write-only; keep the logical level in the caller instead.
        // Reading the pin through IDT gives the actual line state.
        self.pin.read_level()
    }

    pub fn degrade(self) -> Pin {
        self.pin
    }
}

impl embedded_hal::digital::ErrorType for OutputPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for OutputPin {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        OutputPin::set_high(self);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        OutputPin::set_low(self);
        Ok(())
    }
}

/// Analog input or output (ADC/DAC); the digital input Schmitt trigger is
/// disabled in this mode.
pub struct AnalogPin {
    pub pin: Pin,
}

impl AnalogPin {
    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new(port: Port, num: u8) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.configure(0b00, 0b00); // input, analog
        Ok(Self { pin })
    }
}

/// Alternate-function output (peripheral mux drives the pin).
pub struct AlternatePin {
    pub pin: Pin,
}

impl AlternatePin {
    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`]. The peripheral's pin
    /// mux is fixed by package pin function on this device; no mux register
    /// is programmed here.
    pub unsafe fn new_push_pull(port: Port, num: u8, drive: Drive) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.configure(drive.mode_bits(), 0b10); // mux push-pull
        Ok(Self { pin })
    }

    /// # Safety
    /// Same exclusive-startup contract as [`Pin::new`].
    pub unsafe fn new_open_drain(port: Port, num: u8, drive: Drive) -> Result<Self, Error> {
        let pin = unsafe { Pin::new(port, num)? };
        pin.configure(drive.mode_bits(), 0b11); // mux open-drain
        Ok(Self { pin })
    }
}
