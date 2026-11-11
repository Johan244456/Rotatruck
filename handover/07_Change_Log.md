# 07 Change Log (relative to the previous team's delivery)

What OmniPros changed in the repository compared with the Power Assist Rangers delivery (files dated 13 Aug 2026). The repository has no git history, so this list was built from file dates and the code. Once the repository is committed, use git for future changes.

`[FILL IN: if earlier changes were made on another copy or branch (for example during September and October), list them here.]`

## Summary by date

| Date | Change | Who |
|---|---|---|
| Aug 2026 | Prototype audit (photos, `../../Evid/`), handover review (`RotaTruck_Handover_Summary.pdf`), Sprint 1 specification draft | `[FILL IN]` |
| 28 Aug 2026 | Project Specification v1.0 issued (Team 4, Scope B) | Whole team |
| 2 Sep 2026 | Scope approved by C. Borwieck (Omnia Wheel) | |
| Sep 2026 | Strain gauges bonded to the load plate (`../../SG/`, 28 Sep); bench rig on Arduino Uno, middle bridge calibrated | Dang Khoa Tran `[VERIFY]` |
| Sep to Oct 2026 | IMU evaluation and procurement plan (MPU6050 to ISM330DHCX, ESP32-S3 bench rig) | `[FILL IN]` |
| Oct to Nov 2026 | Load-detect bench firmware ported to the Nucleo-G491RE (v4.1, v4.2), host GUIs | Dang Khoa Tran `[VERIFY]` |
| 10 Nov 2026 | ISM330DHCX fitted on the truck, MPU6050 removed, axis signs measured | `[FILL IN]` |
| 10 Nov 2026 | Slope-based gravity compensation, flat-floor test, gravity comp auto-enabled | `[FILL IN]` |
| 10 Nov 2026 | Handle (dead-man) switch and release-stop ramp | `[FILL IN]` |
| 10 Nov 2026 | Load detection integrated into the truck firmware (`loadcell.c`, `payload.c`), new LCD layout, `LOAD_DETECTION.md` | Dang Khoa Tran `[VERIFY]` |
| 11 Nov 2026 | This handover pack (draft), including the safety analysis | `[FILL IN]` |

## Firmware files (`code/STM_RotaTruck/`)

### Added

| File | Purpose |
|---|---|
| `src/deadman.c`, `include/deadman.h` | Handle safety switch on A0: fail-safe pull-up, asymmetric debounce (release 20 ms, press 80 ms) |
| `src/ism330.c`, `include/ism330.h` | ISM330DHCX register driver (WHO_AM_I at 0x6B / 0x6A, configuration, burst read) |
| `src/loadcell.c`, `include/loadcell.h` | Load detection: port of bench-rig firmware v4.2 with integration changes marked `[RT]` (no own clock / UART / IMU driver, console-only output, shortened power-up) |
| `src/payload.c`, `include/payload.h` | Assist payload source: latch on STANDING to DRIVE when armed at STANDING, manual fallback, `W` status |
| `LOAD_DETECTION.md` | Engineering note for the load-detection integration |
| `host/stm_dashboard.py`, `host/stm_calib_gui.py`, `host/stm_dual_test.py`, `host/stm_bench_common.py` | Calibration and drift-test GUIs (copies of the bench-rig GUIs with the `LD 1` / `LD 0` handshake; dashboard keeps the board calibration) |
| `host/*.json` | Last-used GUI settings and a calibration record (the calibration record is invalid, see 04) |
| `Team 4 Rotatruck Project Specification.pdf` | Approved Project Specification v1.0 |

### Changed

Files with a 10 Nov 2026 date. Without the original files to diff against, the descriptions come from reading the current code and comments; small details may have existed before. `[VERIFY: diff against the original delivery (Power_Assisted_Rotatruck GitHub / zip) if it is available.]`

| File | Change |
|---|---|
| `src/main.c` | Init and super-loop: `deadmanSetup/Poll`, `loadcellInit/Update`, `payloadInit/Update`, `imuScanBus()`; header comment updated |
| `include/board.h` | ISM330DHCX on I2C1 with `IMU_AXIS_SIGN_*`; `DEADMAN_*`; HX711 pins `LC_*`; status LED; `LCD_ROW1_SHOW_HANDLE`; buttons documented as manual fallback |
| `include/assist_tuning.h` | `SLOPE_*` estimator constants, `GRAVITY_SLOPE_DEADBAND_DEG`, `GRAVITY_COMP_AUTO_ENABLE 1`, `DEADMAN_DECEL_REV_PER_S2` |
| `src/imu.c`, `include/imu.h` | Rewritten for the ISM330DHCX only: cart-frame rotation, `imuGetLatest()`, bus clear at power-up, bus recovery, boot retries, lazy re-init, dead-sensor detection; MPU6050 removed |
| `src/observer.c`, `include/observer.h` | Road-slope estimator (complementary tilt filter + vertical-acceleration method) feeding gravity comp; `observerGetSlopeDeg()`; `observerReseedMomentum()`; slope reset on `OB Z` |
| `src/assist_control.c` | Release-stop mode `MM_STOP_RAMP` (velocity hold at current speed, ramp to 0, hold); handle gate in `assistDriveActive()`; dev-mode handling; IMU-late-at-boot handling |
| `src/admittance.c` | Assist refused while the `LD` console is open |
| `src/autoseq.c` | Drive-entry message and slope reset; gravity comp follows `GRAVITY_COMP_AUTO_ENABLE` |
| `src/motor_control.c`, `include/motor_control.h` | `motorControlEnterVelocityHold()` for the release stop |
| `src/command.c` | `W`, `W <kg>`, `W A`, `LD` routing, `I S` in cart frame, `I2C`, `I B`; slope in `OB` output; handle state in `M`; junk-byte filter; help text |
| `src/display.c`, `include/display.h` | New layout: payload with `M` for manual, `STOP` state, handle `PRESS` / `RELSD` (or load status), `LD CONSOLE` line |
| `src/lcd_i2c.c` | Uses the shared I2C1 handle `hi2c_imu` owned by `imu.c` `[VERIFY: extent of change]` |
| `src/mode_input.c` | +/- buttons now call `payloadAdjustManual()` |
| `src/stm32g4xx_hal_msp.c` | I2C1 MSP for the shared ISM330DHCX + LCD bus |

### Unchanged

`src/odrive_can.c`, `include/odrive_can.h`, `src/button.c`, `src/pid.c`, `src/stm32g4xx_it.c`, `include/admittance.h`, `include/assist_control.h`, `include/autoseq.h`, `include/button.h`, `include/command.h`, `include/lcd_i2c.h`, `include/mode_input.h`, `include/pid.h`, `platformio.ini`.

## Hardware

| Change | Detail |
|---|---|
| IMU | MPU6050 (GY-521) removed; SparkFun SEN-19764 ISM330DHCX fitted on I2C1, mounted rotated 180 deg about Y `[FILL IN: mounting location / bracket]` |
| Handle switch | Added on A0 to GND `[FILL IN: part, location]` |
| Load plate | Six BF350 gauges in two bridges, two HX711 boards on D2 to D5 `[FILL IN: plate / weighing plates fitted?, HX711 mounting]` |
| `[FILL IN]` | Any wiring, connector or mounting changes |

## Documentation

| Item | Change |
|---|---|
| `handover/` | This pack (new) |
| `RotaTruck_Handover_Summary.pdf` | Review of the previous team's handover (new) |
| `Project Specification.docx` | Sprint 1 specification draft (superseded by v1.0) |
| `docs/` | **Not updated.** Still describes the MPU6050, manual payload and old LCD layout. See README of this pack for precedence |
