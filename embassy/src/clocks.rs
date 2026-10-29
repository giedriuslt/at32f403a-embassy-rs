//! Clock control (CRM) for the AT32F403A: PLL startup, bus dividers and a
//! checked [`Clocks`] snapshot.
//!
//! The programming sequence follows the vendor's `system_clock_config`
//! examples (AT32F403A_407 firmware library V2.2.2): enable the source, wait
//! for stability, program the PLL while it is off and not driving SCLK, set
//! the bus dividers, enable auto-step for the switchover, select PLL as the
//! system clock, then disable auto-step. Every wait is bounded; every
//! frequency is computed exactly and checked against the device limits
//! (SCLK/AHB <= 240 MHz, APB1/APB2 <= 120 MHz; vendor example comments).
//!
//! The flash controller programs its own wait states on this device (the
//! vendor library never writes latency fields), so this module does not touch
//! FLASH registers.

use at32f4xx_pac::at32f403a::CRM;
use at32f4xx_pac::at32f403a::crm::cfg::{AHBDIV_A, APB1DIV_A, PLLHEXTDIV_A, SCLKSEL_A};
use at32f4xx_pac::at32f403a::crm::ctrl;

/// Reset-state HICK frequency: the 48 MHz internal RC divided by six.
pub const HICK_HZ: u32 = 8_000_000;
/// Maximum system and AHB clock (device limit).
pub const SCLK_MAX_HZ: u32 = 240_000_000;
/// Maximum APB1 and APB2 peripheral clock (device limit).
pub const APB_MAX_HZ: u32 = 120_000_000;

/// Busy-wait bound for oscillator/PLL stability and clock-switch loops. A
/// generous cycle-count ceiling only; not a calibrated timeout.
const WAIT_BOUND: u32 = 4_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("PLL multiplier must be 2..=64")]
    InvalidMultiplier,
    #[error("HEXT frequency must be even for HextDiv2 and nonzero")]
    InvalidHext,
    #[error("computed SCLK exceeds the 240 MHz device limit")]
    SclkLimit,
    #[error("computed AHB clock exceeds 240 MHz")]
    AhbLimit,
    #[error("computed APB clock exceeds 120 MHz")]
    ApbLimit,
    #[error("HICK did not become stable")]
    HickTimeout,
    #[error("HEXT did not become stable")]
    HextTimeout,
    #[error("PLL did not become stable")]
    PllTimeout,
    #[error("system clock did not switch")]
    SwitchTimeout,
}

/// PLL input source. `Hick` is the internal 8 MHz RC divided by two (fixed
/// hardware divide on the PLL entry, vendor `system_core_clock_update`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PllSource {
    Hick,
    Hext,
    HextDiv2,
}

/// AHB divider from SCLK. Encodings without a divider (32) do not exist on
/// this device, so they are not offered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AhbDiv {
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
    Div64,
    Div128,
    Div256,
    Div512,
}

impl AhbDiv {
    pub const fn divisor(self) -> u32 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
            Self::Div64 => 64,
            Self::Div128 => 128,
            Self::Div256 => 256,
            Self::Div512 => 512,
        }
    }

    const fn pac(self) -> AHBDIV_A {
        match self {
            Self::Div1 => AHBDIV_A::Sclk,
            Self::Div2 => AHBDIV_A::Div2,
            Self::Div4 => AHBDIV_A::Div4,
            Self::Div8 => AHBDIV_A::Div8,
            Self::Div16 => AHBDIV_A::Div16,
            Self::Div64 => AHBDIV_A::Div64,
            Self::Div128 => AHBDIV_A::Div128,
            Self::Div256 => AHBDIV_A::Div256,
            Self::Div512 => AHBDIV_A::Div512,
        }
    }
}

/// APB1/APB2 divider from the AHB clock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApbDiv {
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
}

impl ApbDiv {
    pub const fn divisor(self) -> u32 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
        }
    }

    const fn pac1(self) -> APB1DIV_A {
        match self {
            Self::Div1 => APB1DIV_A::Hclk,
            Self::Div2 => APB1DIV_A::Div2,
            Self::Div4 => APB1DIV_A::Div4,
            Self::Div8 => APB1DIV_A::Div8,
            Self::Div16 => APB1DIV_A::Div16,
        }
    }

    const fn pac2(self) -> APB1DIV_A {
        match self {
            Self::Div1 => APB1DIV_A::Hclk,
            Self::Div2 => APB1DIV_A::Div2,
            Self::Div4 => APB1DIV_A::Div4,
            Self::Div8 => APB1DIV_A::Div8,
            Self::Div16 => APB1DIV_A::Div16,
        }
    }
}

/// Requested clock profile. The board supplies its crystal frequency; this
/// driver validates the resulting frequencies against the device limits.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// External crystal frequency in Hz; ignored unless the PLL source is HEXT.
    pub hext_hz: u32,
    pub pll_source: PllSource,
    /// PLL multiplication factor, 2..=64.
    pub pll_multiplier: u16,
    pub ahb: AhbDiv,
    pub apb1: ApbDiv,
    pub apb2: ApbDiv,
}

/// Exact, checked clock frequencies produced by [`configure`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Clocks {
    pub sclk_hz: u32,
    /// AHB clock (HCLK).
    pub hclk_hz: u32,
    pub pclk1_hz: u32,
    pub pclk2_hz: u32,
    hclk_div: u32,
    pclk1_div: u32,
    pclk2_div: u32,
}

impl Clocks {
    /// Timer clocks on APB1/APB2: undivided clocks run timers at the bus
    /// clock, divided buses run them at twice the bus clock (as on STM32F1;
    /// vendor `crm_tmr_clock_select` documentation).
    pub const fn timer1_hz(&self) -> u32 {
        if self.pclk1_div == 1 {
            self.pclk1_hz
        } else {
            self.pclk1_hz * 2
        }
    }

    pub const fn timer2_hz(&self) -> u32 {
        if self.pclk2_div == 1 {
            self.pclk2_hz
        } else {
            self.pclk2_hz * 2
        }
    }

    /// The AHB prescaler ratio, for drivers that need HCLK from SCLK.
    pub const fn hclk_div(&self) -> u32 {
        self.hclk_div
    }
}

/// Compute and validate all frequencies for `config` without touching
/// registers. `Err` carries the first violated limit.
pub fn compute(config: &Config) -> Result<Clocks, Error> {
    if !(2..=64).contains(&config.pll_multiplier) {
        return Err(Error::InvalidMultiplier);
    }
    let pll_in = match config.pll_source {
        PllSource::Hick => HICK_HZ / 2,
        PllSource::Hext => {
            if config.hext_hz == 0 {
                return Err(Error::InvalidHext);
            }
            config.hext_hz
        }
        PllSource::HextDiv2 => {
            if config.hext_hz == 0 || !config.hext_hz.is_multiple_of(2) {
                return Err(Error::InvalidHext);
            }
            config.hext_hz / 2
        }
    };
    // The PLL output equals its input times the multiplier; there is no
    // output divider on this device.
    let sclk = pll_in
        .checked_mul(u32::from(config.pll_multiplier))
        .filter(|&sclk| sclk <= SCLK_MAX_HZ)
        .ok_or(Error::SclkLimit)?;
    let hclk = sclk / config.ahb.divisor();
    if hclk * config.ahb.divisor() != sclk || hclk > SCLK_MAX_HZ {
        return Err(Error::AhbLimit);
    }
    let pclk1 = hclk / config.apb1.divisor();
    let pclk2 = hclk / config.apb2.divisor();
    if pclk1 * config.apb1.divisor() != hclk
        || pclk2 * config.apb2.divisor() != hclk
        || pclk1 > APB_MAX_HZ
        || pclk2 > APB_MAX_HZ
    {
        return Err(Error::ApbLimit);
    }
    Ok(Clocks {
        sclk_hz: sclk,
        hclk_hz: hclk,
        pclk1_hz: pclk1,
        pclk2_hz: pclk2,
        hclk_div: config.ahb.divisor(),
        pclk1_div: config.apb1.divisor(),
        pclk2_div: config.apb2.divisor(),
    })
}

/// Run the system on the internal 48 MHz HICK, with USB clocked straight
/// from the same HICK (CRM MISC3.HICK_TO_USB) — the crystalless profile
/// the AT32 system boot ROM itself uses: no PLL, no USB divider. HICK
/// accuracy (±1 %) is sufficient for full-speed enumeration because USB
/// receivers resynchronize every packet; long-run signal-integrity work
/// (ACC SOF calibration) is a later concern, not a smoke-test one.
///
/// Returns the checked [`Clocks`] (everything 48 MHz; divided-bus timer
/// doubling does not apply at divider 1).
///
/// # Safety
///
/// Call once during exclusive startup, before any peripheral driver or
/// interrupt runs. Also switches the USB clock source, so call before the
/// USB cell is enabled.
pub unsafe fn configure_hick48() -> Result<Clocks, Error> {
    // SAFETY: forwarded from this function's exclusive-startup contract.
    let crm = unsafe { CRM::steal() };

    // HICK on and stable (it is the reset source, but verify anyway).
    crm.ctrl().modify(|_, w| w.hicken().variant(ctrl::HickenwWO::Enable));
    wait_stable(|| crm.ctrl().read().hickstbl().bit()).map_err(|_| Error::HickTimeout)?;

    // If an inherited image left the PLL driving SCLK, step down to HICK
    // first (auto-step keeps flash access safe during the switch).
    if crm.cfg().read().sclksts().is_pll() {
        crm.misc3().modify(|_, w| unsafe { w.auto_step_en().bits(0b01) });
        crm.cfg()
            .modify(|_, w| w.sclksel().variant(SCLKSEL_A::Hick));
        wait_stable(|| crm.cfg().read().sclksts().is_hick())
            .map_err(|_| Error::SwitchTimeout)?;
        crm.misc3().modify(|_, w| unsafe { w.auto_step_en().bits(0) });
        crm.ctrl().modify(|_, w| w.pllen().variant(ctrl::PllenwWO::Disable));
    }

    critical_section::with(|_| {
        // Vendor crm_usb_clock_source_select(HICK) +
        // crm_hick_sclk_frequency_select(48MHZ), verbatim:
        // 1. HICK runs at 48 MHz instead of the reset 8 MHz (MISC1.HICKDIV
        //    = CRM_HICK48_NODIV).
        // 2. MISC3.HICK_TO_SCLK routes the 48 MHz HICK to SCLK — without
        //    this bit, "SCLK from HICK" is the pre-divided 8 MHz.
        // 3. MISC3.HICK_TO_USB feeds the USB cell from the same HICK (no
        //    USBDIV involved).
        crm.misc1().modify(|_, w| w.hickdiv().bit(true));
        crm.misc3().modify(|_, w| w.hick_to_sclk().bit(true));
        crm.cfg().modify(|_, w| {
            w.sclksel().variant(SCLKSEL_A::Hick);
            w.ahbdiv().variant(AHBDIV_A::Sclk);
            w.apb1div().variant(APB1DIV_A::Hclk);
            w.apb2div().variant(APB1DIV_A::Hclk);
            w
        });
        crm.misc3().modify(|_, w| w.hick_to_usb().bit(true));
    });

    wait_stable(|| crm.cfg().read().sclksts().is_hick()).map_err(|_| Error::SwitchTimeout)?;

    Ok(Clocks {
        sclk_hz: 48_000_000,
        hclk_hz: 48_000_000,
        pclk1_hz: 48_000_000,
        pclk2_hz: 48_000_000,
        hclk_div: 1,
        pclk1_div: 1,
        pclk2_div: 1,
    })
}

fn wait_stable(mut read: impl FnMut() -> bool) -> Result<(), Error> {
    for _ in 0..WAIT_BOUND {
        if read() {
            return Ok(());
        }
    }
    Err(Error::SwitchTimeout)
}

/// Apply `config` and return the checked clock frequencies.
///
/// # Safety
///
/// Call once during exclusive startup, before any peripheral driver or
/// interrupt runs. The caller must own CRM; nothing else may change the
/// system clock afterwards (the SysTick time driver depends on a stable
/// HCLK). An inherited PLL-driven SCLK (a bootloader may leave one running)
/// is first stepped down to HICK before the PLL is reprogrammed.
pub unsafe fn configure(config: Config) -> Result<Clocks, Error> {
    let clocks = compute(&config)?;
    // SAFETY: forwarded from this function's exclusive-startup contract.
    let crm = unsafe { CRM::steal() };

    // HICK is the reset source and stays enabled throughout.
    crm.ctrl()
        .modify(|_, w| w.hicken().variant(ctrl::HickenwWO::Enable));
    wait_stable(|| crm.ctrl().read().hickstbl().bit()).map_err(|_| Error::HickTimeout)?;

    // If an inherited image already runs from the PLL, step down to HICK
    // before touching it (auto-step keeps flash access safe while slow).
    if crm.cfg().read().sclksts().is_pll() {
        crm.misc3()
            .modify(|_, w| unsafe { w.auto_step_en().bits(0b01) });
        crm.cfg()
            .modify(|_, w| w.sclksel().variant(SCLKSEL_A::Hick));
        wait_stable(|| crm.cfg().read().sclksts().is_hick()).map_err(|_| Error::SwitchTimeout)?;
        crm.misc3()
            .modify(|_, w| unsafe { w.auto_step_en().bits(0) });
    }

    if matches!(config.pll_source, PllSource::Hext | PllSource::HextDiv2) {
        crm.ctrl()
            .modify(|_, w| w.hexten().variant(ctrl::HextenwWO::Enable));
        wait_stable(|| crm.ctrl().read().hextstbl().bit()).map_err(|_| Error::HextTimeout)?;
    }

    // Program the PLL while it is disabled.
    crm.ctrl()
        .modify(|_, w| w.pllen().variant(ctrl::PllenwWO::Disable));
    let raw_mult = (config.pll_multiplier - 2) as u8;
    crm.cfg().modify(|_, w| {
        w.pllrcs().bit(matches!(
            config.pll_source,
            PllSource::Hext | PllSource::HextDiv2
        ));
        w.pllhextdiv()
            .variant(if matches!(config.pll_source, PllSource::HextDiv2) {
                PLLHEXTDIV_A::Div2
            } else {
                PLLHEXTDIV_A::Hext
            });
        // raw_mult <= 62 fits the two PLLMULT fields (compute checked it).
        unsafe { w.pllmult3_0().bits(raw_mult & 0x0F) };
        unsafe { w.pllmult5_4().bits((raw_mult >> 4) & 0x03) };
        // Output range class: > 72 MHz when above the boundary.
        w.pllrange().bit(clocks.sclk_hz > 72_000_000);
        w.ahbdiv().variant(config.ahb.pac());
        w.apb1div().variant(config.apb1.pac1());
        w.apb2div().variant(config.apb2.pac2());
        w
    });
    crm.ctrl()
        .modify(|_, w| w.pllen().variant(ctrl::PllenwWO::Enable));
    wait_stable(|| crm.ctrl().read().pllstbl().bit()).map_err(|_| Error::PllTimeout)?;

    crm.misc3()
        .modify(|_, w| unsafe { w.auto_step_en().bits(0b01) });
    crm.cfg().modify(|_, w| w.sclksel().variant(SCLKSEL_A::Pll));
    wait_stable(|| crm.cfg().read().sclksts().is_pll()).map_err(|_| Error::SwitchTimeout)?;
    crm.misc3()
        .modify(|_, w| unsafe { w.auto_step_en().bits(0) });

    Ok(clocks)
}

#[cfg(test)]
mod tests;
