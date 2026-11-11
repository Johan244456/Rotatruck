# 04 Status, Limitations and Notes

Where the project stands at handover (draft of 11 Nov 2026). Each item states how far it has been verified, using these levels:

| Level | Meaning |
|---|---|
| **Verified on truck** | Run on the RotaTruck and observed to work, with a date |
| **Bench verified** | Works on a bench rig, not yet on the truck |
| **Built, not run** | Compiles and is integrated, never run on hardware |
| **Not started** | |

Update this document as the tests in [06 Testing and Commissioning](06_Testing_and_Commissioning.md) are completed.

---

## 1. Status by work item

| Work item | Status | Evidence / notes |
|---|---|---|
| Firmware builds | **Verified** 11 Nov 2026 | `pio run`: success, 90.5 KB flash, 2.7 KB RAM, `-Wall -Wextra` |
| ISM330DHCX IMU replacing MPU6050 | **Verified on truck** 10 Nov 2026 | Axis mapping measured against the old sensor (r = 0.997 pitch); pose references -43.6 / -1.2 deg |
| Slope-based gravity compensation | **Verified on truck (flat floor only)** 10 Nov 2026 | No phantom push on flat floor; estimate within +/-1 deg in steady pushes, +/-2 to 3 deg at reversals. **Ramp test pending** |
| Handle (dead-man) switch and release-stop ramp | `[FILL IN: Verified on truck <date> / Built, not run]` | Wiring described as built in `board.h`. Record the release-stop test (06, T-SAF) |
| Load detection signal chain (strain gauges, HX711) | **Bench verified** (Uno, middle bridge calibrated) | Full bridge CH2 on the Uno showed a large resting offset (about 4.9 mV) and fast drift (about 50 microstrain/min after power-up) |
| Load detection inside the truck firmware | **Built, not run** | Integrated 10 Nov 2026 per `LOAD_DETECTION.md`; bring-up plan in 06 |
| Payload latch and manual fallback | **Built, not run** | Depends on the item above |
| Calibration GUIs with `LD` handshake | **Bench verified** (GUIs used on the bench rig) | Not yet used against the truck firmware |
| Safety analysis (fail-scenario table) | Drafted | [08 Safety Analysis](08_Safety_Analysis.md); evidence column mostly open |
| Wiring, connector, mounting, enclosure robustness | `[FILL IN]` | Shake / handling test results go in 06, section 7 |
| Bench / controlled ramp tests of the control algorithm (Scope B) | `[FILL IN]` | 06, section 6 |
| Full field-test program (surfaces, inclines, kerb, effort reduction) | Owned by Team 5 (Scope A) | `[FILL IN: Team 5 status, and which Scope B data they need]` |
| Telemetry / data logging feature (FR6) | Not started (beyond existing serial streams) | |

---

## 2. The previous team's limitations: where they stand now

| # | Previous limitation | Status now |
|---|---|---|
| 1 | Tilt / incline assist does not engage correctly when the cart is held at an angle | **Root cause addressed, partly verified.** Gravity comp used frame pitch, so handle angle was read as slope. Replaced by an acceleration-based road-slope estimator. Flat floor verified; ramp not yet |
| 2 | No incline testing | **Still open.** Controlled ramp checks of the algorithm are Scope B; the full field program is Team 5's. `[FILL IN: ramp availability, planned date]` |
| 3 | Payload entered by hand | **Implemented, not run on truck.** Strain-gauge load plate with latch; buttons kept as fallback |
| 4 | Wired power only | Out of this team's scope (Scope A). `[FILL IN: Scope A status]` |
| 5 | No breakaway assist from a dead stop | **Still open.** Intrinsic to the momentum observer. The load plate measures payload, not handle force, so it does not solve this |
| 6 | Tuning constants are bench fits from an earlier cart | **Still open.** Mass, wheelbase, inertia, friction and Kt not re-measured. Becomes more urgent once the battery changes the cart's mass |
| 7 | No cross-axis fault handling | **Still open** |
| 8 | No fault-handler diagnostics | **Still open** |
| (audit) | No handle-mounted emergency stop or dead-man control (this team's prototype audit, spec FR4) | **Dead-man implemented** (handle switch, fail-safe, controlled stop). A separate emergency stop that removes assistance immediately is not fitted yet (see FR4 below) |

---

## 3. Requirements traceability (Project Specification v1.0, approved 2 Sep 2026)

The requirement numbers used in the code comments and `LOAD_DETECTION.md` match this specification. The fail scenarios behind FR1, FR3 and FR4 are detailed in [08 Safety Analysis](08_Safety_Analysis.md).

### Functional requirements

| ID | Requirement (short) | Priority | Status | How it is met / gap |
|---|---|---|---|---|
| FR1 | Assist only when valid operating conditions and an operator enable / presence condition are met | Must | **Implemented**, presence switch `[FILL IN: verified?]` | Assist must be switched on (opt-in), handle switch pressed, pose DRIVE, anti-rollback elapsed, not tilting, cart rolling (`assistDriveActive()`, standstill guard). Presence method chosen: handle switch |
| FR2 | Smooth, bounded acceleration, speed and motor command outputs | Must | **Partly met** | Force clamp 80 N, torque cap 5 N·m per wheel, command low-pass (40 ms), velocity rolloff to 0.7 m/s, overspeed brake above 1.0 m/s, release ramp 1.5 rev/s^2. No explicit acceleration / slew-rate limit on the assist output; limits still TBC with the client |
| FR3 | Safe state on handle release, e-stop, invalid sensor data or lost controller communication | Must | **Partly met** | Release: controlled stop and hold. Invalid load data: payload falls back to manual / 0. IMU loss: recovery and IMU_FAULT. **Gaps:** lost CAN / ODrive heartbeat while driving is not acted on, ODrive CAN watchdog not enabled, no response to one ODrive faulting, STM32 hard fault leaves the ODrives on their last command |
| FR4 | Accessible emergency stop that removes motor assistance immediately | Must | **Partly met** | The handle switch is a dead-man with a controlled (about 1.3 s) stop, not an immediate removal. `S` (console) and [A] switch the assist off. No dedicated, accessible e-stop button yet `[FILL IN: decision on a physical e-stop, coordinated with Team 5's power system]` |
| FR5 | Automatic payload estimation / measurement, or a validated method that needs no payload input | Must | **Implemented, not run on truck** | Strain-gauge load plate with latch at tip-back, buttons as fallback. Repeatable test results still needed to justify the approach (06, section 3) |
| FR6 | Log motor command, speed, payload estimate, safety state and faults | Should | **Partly met** | Serial streams `OB S`, `AS S`, `M S`, load-detect CSV, fault prints. No single log record with all FR6 signals and no on-board storage |
| FR7 | Safety limits and calibration values stored in a controlled, documented configuration | Must | **Partly met** | Limits in `assist_tuning.h` / `board.h` (one file each, commented); load calibration in a versioned flash layout (v6). **Gap:** repository not under version control yet; no record of which build and calibration are on the cart |

### Non-functional requirements

| ID | Requirement (short) | Status | Notes |
|---|---|---|---|
| NFR1 | No uncommanded assist while idle, disabled or in a fault state | Partly | Idle / disabled / standing are covered (axes idle or held). Fault states: see FR3 gaps |
| NFR2 | Deterministic safety-state transitions, recorded in the verification log | Partly | Transitions are explicit state machines with fixed debounce times; verification log in 06 not filled yet |
| NFR3 | Repeatable payload behaviour across the agreed load range | Not yet tested | Accuracy target TBC |
| NFR4 | Stable within the client-approved speed, acceleration, load and incline limits | Flat floor only | Limits TBC; ramp tests pending |
| NFR5 | Telemetry / extra features do not interfere with the control loop or safety | Met by design | Load detection is non-blocking outside the `LD` console; the assist is locked out while the console is open; quiet outside the console |
| NFR6 | Software and calibration documented so another team can reproduce the verified configuration | In progress | This pack, `LOAD_DETECTION.md`, tuning comments. Needs the final calibration record and build ID |
| NFR7 | Compatible with Scope A's power system and agreed interfaces | `[FILL IN]` | Interface agreement with Team 5 not recorded yet |
| NFR8 | No new manual-handling, pinch, trip, thermal or electrical hazards | `[FILL IN]` | Assess the handle switch (grip fatigue), new wiring routes (trip / snag), HX711 / IMU mounting |

---

## 4. Current limitations and known issues

### Safety relevant

1. **Load detection has never run on the truck.** Expect first-run issues: HX711 timing at 170 MHz (DWT based, should be unchanged), the flash page save, and I2C sharing. Until bring-up is complete, treat the payload as manual.
2. **No ramp test of the new gravity compensation.** On a ramp the slope is only learnt once the cart accelerates; the first metres start from slope = 0. Gentle ramps under 2.5 deg get no gravity help.
3. **Single IMU, no backup.** If the ISM330DHCX fails: pose machine loses pitch, load detection goes to IMU_FAULT (payload 0), slope estimate stops. The IMU does recover by itself if it comes back on the bus.
4. **5 V pull-ups on the shared I2C bus** (from the LCD backpack) would over-drive the 3.3 V-only IMU. Measure SDA / SCL before powering a modified harness.
5. **Release stop uses the ODrive velocity loop.** On a steep downhill or with a heavy load the regen during the stop could raise the bus voltage; untested. Brake resistors are fitted per ODrive.
6. **The dead-man stop is software.** It depends on the STM32 running and the CAN bus being healthy. There is no hardware power cut. A CAN fault or a hard fault in the STM32 leaves the ODrives on their last command (ODrive CAN watchdog not enabled).
7. **Cross-axis fault handling and hard-fault diagnostics** are still missing (inherited). The assist will also arm with only one ODrive responding.
8. **IMU loss while driving is not acted on.** The pose machine keeps using the last reading, so the assist carries on without pose, tilt protection or slope updates (08, F07).

The full list of fail scenarios, with the controls in place and the gaps, is in [08 Safety Analysis](08_Safety_Analysis.md).

### Functional

9. **Autonomous mode does not use load detection.** It arms the assist in DRIVE, which bypasses the standing latch, so the manual value (0 kg at power-up) is used unless set with the buttons.
10. **Load placement.** Side-to-side placement is handled by the two bridges; **front-to-back placement error** and **toe friction** (zero offset after unloading) are not. A low-friction skid under the toe is the planned mechanical fix.
11. **Calibration is tied to the cart's balance.** Any change by Team 5 (battery) or to the enclosure needs a recalibration of the load plate and an update of `TROLLEY_MASS_EMPTY_KG`.
12. **Pose threshold margin.** DRIVING now reads about -1.2 deg against a -5 deg threshold. Check a low-handle push does not drop the cart into the hysteresis band.
13. **Kt mismatch to resolve.** Firmware `MOTOR_KT_NM_PER_A` = 0.951 N·m/A; the ODrive setup guide derives 0.3446 N·m/A (8.27 / 24 kV). The observer uses commanded torque so the effect is limited, but the value must match the ODrive's configured torque constant.
14. **Yaw (steering) assist** is computed but not used.
15. **Stale documentation and comments** (see [02](02_Technical_Handover.md), section 10). The ODrive setup guide's STM32 pin table is wrong for this firmware.
16. **Invalid calibration record** in `host/stm_calibration_combined.json`. Do not reuse.

### Project / repository

17. The repository has no commit history, and the load-detect bench firmware is outside the repository (`../../Code_STM/`). Both should be fixed before handover.
18. `[FILL IN: any open procurement, borrowed equipment to return, lab access constraints.]`

---

## 5. Notes for the next team

- **Start with [06 Testing and Commissioning](06_Testing_and_Commissioning.md).** Bring up the load detection on the truck before changing anything else; it is integrated but unproven.
- **`assist_tuning.h` and `board.h` are the only files you should need for tuning and wiring changes.** Every constant says which way to move it.
- **Use the console streams for evidence.** `OB S` (force, velocity, slope), `AS S` (assist state and force), `M S` (pose), `I S` (IMU), and the `LD` CSV. Log them with a serial terminal to a file during every test.
- **Do not calibrate the load plate while anyone touches the cart**, and always use paired capture with slope and offset.
- **Keep `DEADMAN_ENABLE 1`** except on a bench with the wheels off the ground.
- **Talk to Team 5 (Scope A) before they move mass on the cart**, and agree the data they need before each field test (spec objective 5).
- `[FILL IN: anything the team learnt the hard way, contacts at Omnia Wheel / UTS lab, where the physical cart and spares are stored.]`

---

## 6. Recommended next steps (priority order)

1. **Close the priority safety gaps** in [08](08_Safety_Analysis.md), section 2: ODrive CAN watchdog and STM32 watchdog, stop on IMU loss, stop both axes on a one-axis fault, accessible e-stop (agreed with Team 5).
2. **Bring up load detection on the truck** and complete the accuracy and failure tests (06, sections 3 and 5).
3. **Controlled ramp test** of the slope-based gravity compensation at 2 to 3 angles, loaded and empty, uphill and downhill; verify `OB S` slope against a measured ramp angle.
4. **Re-measure the cart's mechanical constants** (mass, wheelbase, inertia, friction) and confirm Kt against the ODrive config, after Team 5's battery is fitted.
5. **Agree the "TBC" limits with the client** (speed, acceleration, incline, reaction times, payload accuracy) and set them in `assist_tuning.h` (FR7).
6. **Support Team 5's field tests** with an FR6 log of motor command, speed, payload, safety state and faults, so the effort-reduction results can be tied to controller behaviour.
7. **Breakaway assist**: consider a handle force sensor to give help from a standstill (the observer cannot).
8. **Telemetry / data collection** (stretch goal) for longer trials, for example SD card or wireless, as groundwork for the ML ideas in the client scope.
9. Tidy-up: update the stale comments and `../docs/`, commit the repository, archive the bench projects.
