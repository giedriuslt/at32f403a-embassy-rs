//! Host tests for the F1-style I²C timing arithmetic (vendor `i2c_init`).

use super::{ConfigError, Duty, Speed, compute_timing};

#[test]
fn standard_mode_at_30m_apb() {
    // 30 MHz / (2 × 100 kHz) = 150; rise = 30 + 1.
    let t = compute_timing(30_000_000, Speed::Standard).unwrap();
    assert_eq!(t.clkfreq, 30);
    assert_eq!(t.speed, 150);
    assert!(!t.fast);
    assert_eq!(t.risetime, 31);
}

#[test]
fn fast_mode_duty_2_1_at_120m_apb() {
    // 120 MHz / (3 × 400 kHz) = 100; rise = 120 × 0.3 + 1 = 37.
    let t = compute_timing(120_000_000, Speed::Fast(Duty::Duty2To1)).unwrap();
    assert_eq!(t.clkfreq, 120);
    assert_eq!(t.speed, 100);
    assert!(t.fast);
    assert!(!t.duty_16_9);
    assert_eq!(t.risetime, 37);
}

#[test]
fn fast_mode_duty_16_9_at_120m_apb() {
    // 120 MHz / (25 × 400 kHz) = 12.
    let t = compute_timing(120_000_000, Speed::Fast(Duty::Duty16To9)).unwrap();
    assert_eq!(t.speed, 12);
    assert!(t.duty_16_9);
}

#[test]
fn slow_apb_stays_above_the_speed_minimum() {
    // 1 MHz / (2 × 100 kHz) = 5, above the minimum of 4.
    let t = compute_timing(1_000_000, Speed::Standard).unwrap();
    assert_eq!(t.speed, 5);
    assert_eq!(t.risetime, 2);
}

#[test]
fn rejects_non_whole_mhz_pclk() {
    assert_eq!(
        compute_timing(30_500_000, Speed::Standard).unwrap_err(),
        ConfigError::ClockNotMhz
    );
    assert_eq!(
        compute_timing(500_000, Speed::Standard).unwrap_err(),
        ConfigError::ClockNotMhz
    );
}
