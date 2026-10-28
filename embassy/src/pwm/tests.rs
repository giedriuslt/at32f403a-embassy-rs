//! Host tests for the PWM time-base selection.

use super::compute_base;

#[test]
fn one_khz_from_120m_timer_clock() {
    // A 120 MHz timer clock at 1 kHz is 120000 ticks, beyond 16 bits:
    // the smallest fitting prescaler is /2 (div=1) for 60000 ticks.
    let (div, pr) = compute_base(120_000_000, 1_000).unwrap();
    assert_eq!(div, 1);
    assert_eq!(pr, 59_999);
    // Effective period: 60000 ticks at 60 MHz = 1 kHz exact.
}

#[test]
fn fast_pwm_needs_no_prescaler() {
    // 120 MHz / 20 kHz = 6000 ticks: div 0.
    let (div, pr) = compute_base(120_000_000, 20_000).unwrap();
    assert_eq!(div, 0);
    assert_eq!(pr, 5_999);
}

#[test]
fn slow_frequency_uses_the_largest_prescaler() {
    // 120 MHz / 50 Hz = 2.4M ticks: needs prescaler 37 (div=36): 120M/37/50
    // = 64864... the function takes the smallest div that fits, computing
    // period = floor(timer/((div+1)*freq)) without exactness.
    let (div, pr) = compute_base(120_000_000, 50).unwrap();
    let period = u32::from(pr) + 1;
    let prescaled = 120_000_000 / (u32::from(div) + 1);
    assert!(period <= 65_536);
    assert!(prescaled / period >= 50);
    assert!(prescaled / period <= 50);
}

#[test]
fn unreasonable_frequencies_are_rejected() {
    // Faster than timer_hz/2 has no two-tick period.
    assert!(compute_base(120_000_000, 60_000_001).is_err());
    assert!(compute_base(120_000_000, 0).is_err());
}
