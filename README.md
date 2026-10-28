# at32f403a-embassy-rs

Embassy HAL support for the Artery **AT32F403A** (Cortex-M4F, 240 MHz),
built for porting the [kws-x1-standalone-rs](https://github.com/kralonur/kws-x1-standalone-rs)
USB-C power-meter firmware to the **FNIRSI FNB58** USB tester. Structure and
conventions follow [hc32f460-embassy-rs](https://github.com/kralonur/hc32f460-embassy-rs)
(`embassy-hc32`), which proved out this layered-HAL approach on the KOWSI
KWS-X1. Register semantics are cross-checked against the vendor
[AT32F403A_407 firmware library](https://github.com/ArteryTek/AT32F403A_407_Firmware_Library)
(V2.2.2); the PAC is [`at32f4xx-pac`](https://crates.io/crates/at32f4xx-pac)
0.4 with the `at32f403a` feature.

> **Warning:** like `embassy-hc32`, this is an incomplete HAL built for one
> project. Peripheral support covers what the FNB58 port needs; review and
> validate on hardware before reusing elsewhere.

## Status

| Module | State | Notes |
| ------ | ----- | ----- |
| `clocks` | done (needs silicon validation) | HICK/HEXT(÷2)/PLL startup, checked dividers, SCLK/AHB ≤ 240 MHz and APB ≤ 120 MHz limits, inherited-PLL step-down, auto-step switchover, exact `Clocks` snapshot with timer-clock doubling |
| `time_driver` | done (needs silicon validation) | exact 1 kHz SysTick driver, 16-slot timer queue, ported verbatim from the proven HC32 driver |
| `gpio` | done (needs silicon validation) | pin tokens with input (floating/pull up/down), output (push-pull/open-drain, two drive strengths), analog and mux modes; atomic set/clear through SCR; `embedded-hal` 1.0 digital traits |
| `exti` | done (needs silicon validation) | two-flag latch design from embassy-hc32; IOMUX port select, edge polarity, grouped vectors; `embedded-hal-async` `Wait` |
| `i2c` | done (needs silicon validation) | async master over I2C1 with vendor timing formulas (5 host tests) and the vendor read endgame (1/2/3-byte ACK/POS cases); I2C3 sits on APB2 |
| `spi` | done (needs silicon validation) | TX-focused master (SPI1): mode + MDIV2_0/MDIV3 divider, async byte write, TDBE+!BF flush fence, DMA pixel bursts (`SPI1_TX` request) |
| `pwm` | done (needs silicon validation) | TMR3 output channels; smallest-fitting-prescaler time base; CM1/CM2 mode bits via documented raw pointers (PAC gap) |
| `adc` | not started | scan (D±/CC lines + VREFINT) and continuous DMA capture (ripple) |
| `dma` | done (needs silicon validation) | 2 units × 7 F1-style channels, flexible request mapping, async full-transfer waits, abort-on-drop |
| `pwm` | not started | backlight timer |
| flash | not started | AT32 flash controller differs from HC32 EFM; sector sizes from the RM |

## Integration

```toml
embassy-at32 = { path = "path/to/embassy", features = ["rt", "critical-section-single-core"] }
```

- `time-driver` (default) links the SysTick driver; the board calls
  `clocks::configure` (exclusive startup), then `embassy_at32::init(clocks)`,
  and forwards the SysTick exception to `time_driver::systick_tick`.
- `rt` re-exports the PAC's vector support; `critical-section-single-core`
  supplies Cortex-M critical sections (link exactly one implementation).

### Reference documents

- **RM** — AT32F403A reference manual (Artery; `AT32F403A_407_User_Manual`)
- **DS** — AT32F403A datasheet
- **BSP** — the vendor firmware library commit cloned alongside this project
  (`../AT32F403A_407_Firmware_Library`); register sequences cite it as `BSP`.

FNB58 board facts and the porting plan live in the parent project's research
notes; key hardware: FUSB302B PD controller, TPA626 (INA226-compatible
monitor), KXTJ3-1057 accelerometer, ST7789-class SPI display.

## Build and validation

```sh
cargo fmt --all -- --check
cargo check -p embassy-at32 --target thumbv7em-none-eabihf
cargo check -p embassy-at32 --no-default-features --target thumbv7em-none-eabihf
cargo clippy -p embassy-at32 --target thumbv7em-none-eabihf -- -D warnings
cargo test -p embassy-at32 --lib
```

21 host tests cover the clock, I²C timing, SPI divider and PWM time-base
arithmetic.

Host tests cover the pure clock arithmetic, limit checks and SysTick reload
math. They do not prove timing or behavior on silicon.

## License

MIT OR Apache-2.0, matching the upstream projects this port follows.
