//! Host tests for the SPI clock-divider selection.

use super::Divider;

#[test]
fn factors_are_powers_of_two() {
    assert_eq!(Divider::Div2.factor(), 2);
    assert_eq!(Divider::Div4.factor(), 4);
    assert_eq!(Divider::Div256.factor(), 256);
    assert_eq!(Divider::Div512.factor(), 512);
    assert_eq!(Divider::Div1024.factor(), 1024);
}

#[test]
fn picks_the_fastest_divider_within_the_limit() {
    // 120 MHz PCLK, panel clocks up to 40 MHz: /4 gives 30 MHz.
    assert_eq!(Divider::for_hz(120_000_000, 40_000_000), Divider::Div4);
    // Up to 60 MHz: /2 gives 60 MHz.
    assert_eq!(Divider::for_hz(120_000_000, 60_000_000), Divider::Div2);
    // Up to 15 MHz: /8 gives 15 MHz.
    assert_eq!(Divider::for_hz(120_000_000, 15_000_000), Divider::Div8);
    // A limit that /2 exceeds falls to /4.
    assert_eq!(Divider::for_hz(120_000_000, 50_000_000), Divider::Div4);
}

#[test]
fn slow_limits_fall_through_to_the_largest_divider() {
    assert_eq!(Divider::for_hz(120_000_000, 100_000), Divider::Div1024);
    assert_eq!(Divider::for_hz(120_000_000, 1), Divider::Div1024);
}

#[test]
fn encoding_splits_across_both_registers() {
    // Encodings 0..=7 use MDIV2_0 only; 8 and 9 set CTRL2.MDIV3 as well.
    assert!(u32::from(Divider::Div256 as u8) <= 7);
    assert!(u32::from(Divider::Div512 as u8) > 7);
    assert!(u32::from(Divider::Div1024 as u8) > 7);
}
