//! Host tests for the pure clock arithmetic and limit checks.

use super::{AhbDiv, ApbDiv, Config, Error, HICK_HZ, PllSource};

fn config(source: PllSource, mult: u16, hext: u32) -> Config {
    Config {
        hext_hz: hext,
        pll_source: source,
        pll_multiplier: mult,
        ahb: AhbDiv::Div1,
        apb1: ApbDiv::Div2,
        apb2: ApbDiv::Div2,
    }
}

#[test]
fn hick_pll_input_is_half_hick() {
    // 8 MHz / 2 * 16 = 64 MHz, the vendor's HICK example.
    let clocks = super::compute(&config(PllSource::Hick, 16, 0)).unwrap();
    assert_eq!(clocks.sclk_hz, 64_000_000);
    assert_eq!(clocks.hclk_hz, 64_000_000);
    assert_eq!(clocks.pclk1_hz, 32_000_000);
    assert_eq!(clocks.pclk2_hz, 32_000_000);
    assert_eq!(HICK_HZ / 2, 4_000_000);
}

#[test]
fn hext_direct_240m() {
    // 8 MHz * 30 = 240 MHz: AHB /1, APB /2 = 120 MHz (both at their limits).
    let clocks = super::compute(&config(PllSource::Hext, 30, 8_000_000)).unwrap();
    assert_eq!(clocks.sclk_hz, 240_000_000);
    assert_eq!(clocks.hclk_hz, 240_000_000);
    assert_eq!(clocks.pclk1_hz, 120_000_000);
    assert_eq!(clocks.pclk2_hz, 120_000_000);
}

#[test]
fn hext_div2_192m() {
    // 8 MHz / 2 * 48 = 192 MHz, the vendor USB example profile.
    let clocks = super::compute(&config(PllSource::HextDiv2, 48, 8_000_000)).unwrap();
    assert_eq!(clocks.sclk_hz, 192_000_000);
}

#[test]
fn timer_clocks_double_on_divided_bus() {
    let mut c = config(PllSource::Hext, 30, 8_000_000);
    c.apb1 = ApbDiv::Div2;
    c.apb2 = ApbDiv::Div4;
    let clocks = super::compute(&c).unwrap();
    // APB1: 120 MHz bus, timers 240 MHz; APB2: 60 MHz bus, timers 120 MHz.
    assert_eq!(clocks.timer1_hz(), 240_000_000);
    assert_eq!(clocks.timer2_hz(), 120_000_000);

    // Undivided APB1 needs an AHB within the 120 MHz APB limit: 8 MHz * 15.
    let mut c = config(PllSource::Hext, 15, 8_000_000);
    c.apb1 = ApbDiv::Div1;
    let clocks = super::compute(&c).unwrap();
    assert_eq!(clocks.pclk1_hz, 120_000_000);
    assert_eq!(clocks.timer1_hz(), 120_000_000);
    // A 240 MHz AHB with an undivided APB exceeds the 120 MHz APB limit.
    let mut c = config(PllSource::Hext, 30, 8_000_000);
    c.apb1 = ApbDiv::Div1;
    assert_eq!(super::compute(&c).unwrap_err(), Error::ApbLimit);
}

#[test]
fn rejects_limit_violations() {
    assert_eq!(
        super::compute(&config(PllSource::Hext, 31, 8_000_000)).unwrap_err(),
        Error::SclkLimit
    );
    assert_eq!(
        super::compute(&config(PllSource::Hick, 1, 0)).unwrap_err(),
        Error::InvalidMultiplier
    );
    assert_eq!(
        super::compute(&config(PllSource::Hick, 65, 0)).unwrap_err(),
        Error::InvalidMultiplier
    );
    assert_eq!(
        super::compute(&config(PllSource::HextDiv2, 10, 0)).unwrap_err(),
        Error::InvalidHext
    );
    assert_eq!(
        super::compute(&config(PllSource::HextDiv2, 10, 25_000_001)).unwrap_err(),
        Error::InvalidHext
    );
    // 240 MHz SCLK with APB /1 would need 240 MHz on APB.
    let mut c = config(PllSource::Hext, 30, 8_000_000);
    c.apb1 = ApbDiv::Div1;
    assert_eq!(super::compute(&c).unwrap_err(), Error::ApbLimit);
    // 192 MHz AHB with a /4 AHB divider is fine; /1 AHB on a >240 MHz PLL is
    // caught earlier as SclkLimit, so check the AHB path with a valid SCLK.
    let mut c = config(PllSource::Hext, 30, 8_000_000);
    c.ahb = AhbDiv::Div2;
    let clocks = super::compute(&c).unwrap();
    assert_eq!(clocks.hclk_hz, 120_000_000);
    assert_eq!(clocks.hclk_div(), 2);
}

#[test]
fn multiplier_range_covers_raw_encoding() {
    // Raw register value is mult - 2 in 6 bits: 2 -> 0, 64 -> 62.
    for mult in [2u16, 3, 32, 60] {
        assert!(super::compute(&config(PllSource::Hick, mult, 0)).is_ok());
    }
    // 4 MHz HICK input times 63 exceeds the 240 MHz SCLK limit.
    assert_eq!(
        super::compute(&config(PllSource::Hick, 63, 0)).unwrap_err(),
        Error::SclkLimit
    );
}
