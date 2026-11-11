# 06 Testing and Commissioning

Bring-up procedure for the hardware added this phase, and the verification tests with blank result tables. Record every result with the date, the firmware build, who ran it, and a saved console log. Move results into [04 Status](04_Status_and_Limitations.md) when done.

**General rules**

- First runs with the **wheels off the ground** (cart on blocks), then on the floor empty, then loaded.
- Log the console to a file during every test (PlatformIO monitor with a log filter, PuTTY logging, or similar).
- Keep `DEADMAN_ENABLE 1` for all floor tests.
- Two people for floor tests: one operating, one watching with the PSU switch in reach.

---

## 1. Pre-power checks

| # | Check | Expected | Result | Date / by |
|---|---|---|---|---|
| P1 | 3V3 to GND resistance with power off (HX711s, IMU fitted) | No short | `[FILL IN]` | |
| P2 | HX711 VCC wired to 3V3, not 5 V | 3V3 | `[FILL IN]` | |
| P3 | Handle switch continuity: pressed A0 to GND closed, released open | As stated | `[FILL IN]` | |
| P4 | CAN termination: CAN_H to CAN_L with power off | About 60 ohm | `[FILL IN]` | |
| P5 | Power on, SDA and SCL to GND | About 3.3 V (not 5 V) | `[FILL IN]` | |

## 2. Boot and sensor bring-up

| # | Step | Expected | Result | Date / by |
|---|---|---|---|---|
| B1 | Flash, open console, reset | Boot log as in [02](02_Technical_Handover.md) section 2, `LOAD: HX711 CH1 ok, CH2 ok` | `[FILL IN]` | |
| B2 | `I2C` | IMU at 0x6B, LCD at 0x27 or 0x3F | `[FILL IN]` | |
| B3 | `I S`, tilt STANDING to DRIVING | Pitch about -43.6 to -1.2 deg, gx positive towards DRIVING | `[FILL IN]` | |
| B4 | `I B` with the cart still | Gyro bias and noise small, abs(a) about 1 g | `[FILL IN]` | |
| B5 | `M` with the switch released / pressed | `handle RELEASED` / `PRESSED`; LCD `RELSD` / `PRESS` | `[FILL IN]` | |
| B6 | `OD` | Both ODrives seen, no errors, Vbus about 24 V | `[FILL IN]` | |
| B7 | `LD`, then `b` and `d` | Both channels respond, sensible bridge voltages | `[FILL IN]` | |
| B8 | Unplug and replug the IMU while running | `IMU lost`, then recovers by itself within about 0.5 s | `[FILL IN]` | |

## 3. Load detection calibration on the truck

Follow [02 Technical Handover](02_Technical_Handover.md) section 6.4 and `../code/STM_RotaTruck/LOAD_DETECTION.md`.

| # | Step | Result | Date / by |
|---|---|---|---|
| C1 | `q s` (standing, at least 15 kg, hands off) and `q r` (empty, tipped back) | Angles: `[FILL IN]` | |
| C2 | 40 min unloaded drift run (`stm_dual_test.py`) | Drift CH1 / CH2: `[FILL IN]` | |
| C3 | Check signs (press each strip, reading rises) | `[FILL IN]` | |
| C4 | Calibrate 2 SG (middle) | Slope / offset / R^2: `[FILL IN]` | |
| C5 | Calibrate 4 SG (left + right) | Slope / offset / R^2: `[FILL IN]` | |
| C6 | Balance g | g = `[FILL IN]` | |
| C7 | Calibrate All 6 (at least 15, 30, 40 kg, 2 to 3 rounds) | Slope / offset / R^2: `[FILL IN]` | |
| C8 | Power-cycle, check calibration kept (`LD`, `b`) | `[FILL IN]` | |

**Accuracy check (after calibration)**

| Reference mass (kg) | Placement | Reading 1 | Reading 2 | Reading 3 | Mean error (kg) |
|---|---|---|---|---|---|
| 15 | Centred | | | | |
| 30 | Centred | | | | |
| 45 | Centred | | | | |
| 60 | Centred | | | | |
| 30 | Left side | | | | |
| 30 | Right side | | | | |
| 30 | Front of plate | | | | |
| 30 | Back of plate | | | | |
| 0 (after unloading) | | | | | |

## 4. Safety tests (T-SAF)

Wheels off the ground first, then on the floor.

| # | Test | Expected | Result | Date / by |
|---|---|---|---|---|
| S1 | Assist ON, DRIVE, switch released | No assist; LCD `STOP` | `[FILL IN]` | |
| S2 | Drive at walking pace, release the switch | Smooth stop in about 1 to 1.5 s, then held; no freewheel, no slam | Stop time / distance: `[FILL IN]` | |
| S3 | S2 loaded (60 kg) | As S2; no ODrive overvoltage fault | `[FILL IN]` | |
| S4 | Unplug the switch connector while driving | Treated as released: controlled stop | `[FILL IN]` | |
| S5 | Re-press after a stop | Assist resumes without a jerk or a phantom push | `[FILL IN]` | |
| S6 | Release while STANDING / during anti-rollback | Stays braked | `[FILL IN]` | |
| S7 | Fast tilt with assist on | Wheels held, no spin, PSU does not trip | `[FILL IN]` | |
| S8 | Overspeed: push past 1 m/s (on blocks) | Assist fades by 0.7 m/s, braking above 1 m/s | `[FILL IN]` | |
| S9 | Cart at rest in DRIVE, no hands on the cart, switch pressed (taped) | No creep | `[FILL IN]` | |
| S10 | Press [A] while driving (wheels on blocks) | Assist off, axes idle immediately; record reaction time | `[FILL IN]` | |
| S11 | Unplug the CAN cable while the assist drives a wheel (on blocks, low gain) | **Expected today: the wheel keeps its last torque (known gap F13).** After the fix: wheels go idle within the watchdog time | `[FILL IN]` | |
| S12 | Power off one ODrive while driving (on blocks) | **Expected today: the other wheel keeps driving (known gap F14).** After the fix: both axes stop | `[FILL IN]` | |

Scenario IDs (F01 to F26) and the controls being tested are in [08 Safety Analysis](08_Safety_Analysis.md). Record results there as evidence too.

## 5. Load detection on the truck (T-LD)

From `LOAD_DETECTION.md`, "Bring-up on the truck", step 7.

| # | Test | Expected | Result | Date / by |
|---|---|---|---|---|
| L1 | Stand loaded, hands off, assist ON at STANDING, tip back | `PAYLOAD: latched ... kg`, LCD payload without `M` | `[FILL IN]` | |
| L2 | Latched value vs reference mass | About 0.9 x reference | `[FILL IN]` | |
| L3 | Unplug one HX711 DT wire, arm at STANDING, tip back | `LOAD NO_DATA` within 0.5 s; on tip-back `no valid load reading`, manual value used | `[FILL IN]` | |
| L4 | Switch the assist on in DRIVE | `assist ON outside STANDING`, manual value, no latch | `[FILL IN]` | |
| L5 | Press +/- during a push | Payload changes by 5 kg, LCD shows `M` | `[FILL IN]` | |
| L6 | Unplug the IMU | About 0.1 s later `LOAD: IMU lost`, status `IMU!` | `[FILL IN]` | |
| L7 | Open `LD` with the assist on | Assist switches off; D8 / B1 / `AS 1` refused until `LD` closed | `[FILL IN]` | |
| L8 | Power-cycle | Zero, scales, g, signs and pose references kept | `[FILL IN]` | |
| L9 | Stand up and tip back again (assist still on) with a different load | Re-latches the new load | `[FILL IN]` | |

## 6. Slope / gravity compensation (T-SLOPE)

| # | Test | Expected | Result | Date / by |
|---|---|---|---|---|
| G1 | Flat floor, push and pull with the handle high and low (`OB S`) | Slope within +/-2.5 deg, no phantom push | **Passed 10 Nov 2026** (+/-1 deg steady, +/-2 to 3 deg at reversals) `[FILL IN: who, log file]` | 10 Nov 2026 |
| G2 | Ramp angle measured with an inclinometer | | Angle: `[FILL IN]` | |
| G3 | Uphill, empty, `OB S` | Slope reads about the ramp angle after the first acceleration | `[FILL IN]` | |
| G4 | Uphill, 60 kg | Assist helps hold and climb; no roll-back after the 1 s hold | `[FILL IN]` | |
| G5 | Downhill, 60 kg | No runaway; overspeed brake works; release stop works | `[FILL IN]` | |
| G6 | Stop and hold on the ramp, release switch | Cart held at 0 | `[FILL IN]` | |
| G7 | Cross-slope / turning on the ramp | `[FILL IN: expected]` | `[FILL IN]` | |

## 7. Wiring and robustness tests (W, Scope B)

Spec objective: inspection and shake / handling tests show secured wiring, strain relief, protected connections and no intermittent faults. IP target TBC with the client.

| # | Test | Expected | Result | Date / by |
|---|---|---|---|---|
| W1 | Visual inspection of every harness against the wiring record (02, section 3.7) | Secured, strain-relieved, labelled, no exposed splices | `[FILL IN]` | |
| W2 | Shake / handling test: tip STANDING to DRIVE 20 times, roll over a kerb step `[FILL IN: agreed procedure]` while streaming `M S`, `OB S` and the `LD` status | No resets, no `NO_DATA`, no `IMU lost`, no handle-switch glitches, no ODrive errors | `[FILL IN]` | |
| W3 | Wiggle test on each connector with the console streaming | No dropouts | `[FILL IN]` | |
| W4 | Cable pull on the handle-switch and sensor leads | Strain relief holds; if a lead comes out, the cart stops (fail-safe) | `[FILL IN]` | |
| W5 | Hazard walk-round (NFR8): trip, snag, pinch, hot surfaces (brake resistors), exposed conductors | None found, or recorded and mitigated | `[FILL IN]` | |
| W6 | Enclosure / ingress check against the agreed IP target | `[FILL IN]` | `[FILL IN]` | |

## 8. Performance and benefit test matrix (Team 5 runs, Scope B supports)

Under the approved scope, **Team 5 (Scope A) owns the full real-life field-test program**. Scope B supplies the controls data, telemetry and technical support, and runs its own bench and controlled ramp checks (sections 4 to 6). Use this table to record the Scope B data handed to Team 5, or their results that bear on the controls.

Agreed matrix: `[FILL IN: surfaces, angles and loads agreed between the client and Team 5]`. Suggested measures: peak and mean handle force, task completion time, Borg RPE, assist on vs off, same operator and route.

| Surface | Incline | Load (kg) | Assist | Peak force (N) | Mean force (N) | Time (s) | Borg RPE | Notes |
|---|---|---|---|---|---|---|---|---|
| Indoor smooth | 0 deg | 0 | Off | | | | | |
| Indoor smooth | 0 deg | 0 | On | | | | | |
| Indoor smooth | 0 deg | 60 | Off | | | | | |
| Indoor smooth | 0 deg | 60 | On | | | | | |
| `[FILL IN]` | `[FILL IN]` | | | | | | | |

Targets carried over from the previous phase: response 100 to 300 ms, max assisted speed 4 to 6 km/h, max acceleration 0.5 to 1.0 m/s^2, idle movement 0 km/h, load 60 to 90 kg.

## 9. Test log

| Date | Test IDs | Firmware build / change | People | Log file | Outcome |
|---|---|---|---|---|---|
| 10 Nov 2026 | IMU axis mapping, G1 | IMU swap + slope estimator | `[FILL IN]` | `[FILL IN]` | Pass (flat floor) |
| `[FILL IN]` | | | | | |
