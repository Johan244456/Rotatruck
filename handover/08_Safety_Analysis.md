# 08 Safety Analysis: Fail Scenarios, Controls and Evidence

Spec deliverable "Safety Analysis: fail-scenario analysis, adopted controls and verification evidence for each safety response" (Project Specification v1.0, sections 4 and 8; requirements FR1, FR3, FR4, NFR1, NFR2, NFR8).

The **current control** column was written from the firmware as of 11 Nov 2026 (file references given). The **status** column says whether the control exists in code; the **evidence** column says whether it has been demonstrated. Nothing is "closed" until it has evidence in [06 Testing and Commissioning](06_Testing_and_Commissioning.md).

Reaction-time targets are TBC with the client (spec NFR2). `[FILL IN: agreed targets]`

Status key: **Implemented** (in code), **Partial**, **Gap** (no control), **Inherited** (previous team's control, unchanged).

## 1. Fail-scenario table

### Operator

| ID | Scenario | Hazard | Current control | Status | Test | Evidence |
|---|---|---|---|---|---|---|
| F01 | Operator lets go of the handle while the cart is driven | Cart keeps rolling or is driven with nobody in control | Handle switch released: both wheels taken into velocity control at their current speed, ramped to 0 at 1.5 rev/s^2, then held (`assist_control.c` MM_STOP_RAMP). Assist gate requires the switch (`assistDriveActive()`) | Implemented | S1, S2, S3 | `[FILL IN]` |
| F02 | Handle switch wire broken or connector unplugged | Loss of the presence signal | Internal pull-up: open circuit reads RELEASED = stop. Starts RELEASED at boot, must be seen pressed (`deadman.c`) | Implemented | S4 | `[FILL IN]` |
| F03 | Handle switch defeated (taped down) or shorted to GND | Presence check bypassed | None detectable in firmware | Gap (residual risk; manage by design of the switch / procedure) | | |
| F04 | Operator needs to stop everything now (emergency) | Delayed stop | [A] button or B1 switches the assist off (axes go idle); console `S` disables assist and holds 0; handle release gives a controlled stop | Partial: no dedicated, accessible e-stop (spec FR4) | S10 | `[FILL IN: decision on a physical e-stop with Team 5]` |
| F05 | Operator pushes from a dead stop on a slope | No help, or rollback | Anti-rollback 1 s hold after tip-back; no breakaway assist below 0.05 m/s | Inherited / partial | G4 | |
| F06 | Assist switched on by mistake / unexpectedly | Uncommanded motion | Assist is opt-in (off at boot); standstill guard (no assist until rolling above 0.05 m/s); deadband 7 N | Inherited | S9 | |

### Sensors

| ID | Scenario | Hazard | Current control | Status | Test | Evidence |
|---|---|---|---|---|---|---|
| F07 | IMU lost or returns invalid data **while driving** | Pose, tilt protection and slope estimate frozen at their last values; assist continues | `imu.c` retries / re-configures the sensor (bus recovery, dead-sensor check, lazy re-init every 500 ms). Observer holds the last good pitch. The pose machine is **not** told the IMU is gone (it only checks presence at boot) | **Gap**: on IMU loss the assist should be disabled or the wheels brought to a stop | B8, L6 | |
| F08 | IMU absent at boot | No pose detection | "Dev mode": only the handle switch gates the assist; if the IMU appears later, pose control is enabled | Partial: dev mode allows assist without pose; consider refusing `AS 1` with no IMU outside bench use | | |
| F09 | Load sensor fault (HX711 silent, railed, frozen, implausible) | Wrong assist scaling | Status NO_DATA / RAILED / FROZEN / IMPLAUSIBLE: assist payload 0; latch falls back to the manual value; fault printed (`loadcell.c`, `payload.c`) | Implemented, not run on truck | L3 | |
| F10 | Payload over-estimated (calibration drift, someone leaning on the plate) | Too much assist | 0.9 safety factor, cap 150 kg; reading only counts when standing, hands off 1 s; assist force and torque caps independent of payload; manual override | Partial: no plausibility check against the observer's own mass estimate | L2 | |
| F11 | Phantom push from handle angle on flat ground | Uncommanded motion | Slope estimated from acceleration (handle angle drops out), 2.5 deg deadband (`observer.c`) | Implemented | G1 | **Flat floor, 10 Nov 2026: passed** |
| F12 | Standstill FWD / BACK oscillation with a load | Cart rocks on its own | Standstill guard with hysteresis (0.05 / 0.02 m/s, 150 ms) | Inherited | S9 | |

### Controllers and communication

| ID | Scenario | Hazard | Current control | Status | Test | Evidence |
|---|---|---|---|---|---|---|
| F13 | CAN bus lost (wire off, transceiver fault) while driving | ODrives keep applying the last torque command | Heartbeat only checked when arming (`motor_control.c`); ODrive CAN watchdog not enabled | **Gap**: enable the ODrive watchdog (axis to idle if no command for N ms) and stop on missing heartbeat | S11 | |
| F14 | One ODrive faults or loses power while driving | One wheel drives, the other does not: cart yaws | ODrive error shown as `ERR` on the LCD; no action. The assist can also be armed with only one axis alive | **Gap**: stop / disarm both axes if either faults; require both axes on the assembled truck | S12 | |
| F15 | STM32 hard fault or hang | ODrives keep the last command | `Error_Handler` / default fault handler spin with no output; no independent watchdog (IWDG not used) | **Gap**: ODrive watchdog (F13) plus MCU IWDG and a fault indication | | |
| F16 | Load-detect console calibration blocks the main loop | Control loop stalls with torque applied | Assist forced off on entering `LD` and refused while open (`admittance.c`) | Implemented | L7 | |
| F17 | I2C bus locks (LCD or IMU) | Main loop stalls / stale IMU data | 25 ms I2C timeouts, bus clear and re-init (`imu.c`, `lcd_i2c.c`) | Implemented | B8 | |

### Motion and environment

| ID | Scenario | Hazard | Current control | Status | Test | Evidence |
|---|---|---|---|---|---|---|
| F18 | Overspeed / downhill runaway | Loss of control on a slope | Assist rolloff to 0 by 0.7 m/s; brake (-20 x v) above 1.0 m/s; torque cap 5 N·m per wheel; velocity limit 4 rev/s on the ODrive | Inherited, untested on a slope | S8, G5 | |
| F19 | Ramp steeper than the system can hold | Rollback / runaway | Slope estimate clamped at 12 deg; no warning or refusal | Gap: define the incline limit with the client (NFR4) and react to it | G2 to G6 | |
| F20 | Cart rolls back when tipped onto its wheels | Unintended motion | Anti-rollback: braked for 1 s after STANDING to DRIVE | Inherited | S6 | |
| F21 | Wheels spin while the cart is being tilted | Unexpected motion, supply trip | Tilt protection: hold while pitch rate > 15 deg/s, 250 ms settle | Inherited | S7 | |
| F22 | Regenerative over-voltage during a release stop or downhill | ODrive trips, wheels go limp | Brake resistor per ODrive; ODrive over-voltage trip | Partial, untested with a load or on a slope | S3, G5 | |
| F23 | Supply brown-out / reset | Uncontrolled restart | After reset the assist is off (opt-in), handle switch must be seen pressed; IMU re-configured after a brown-out | Implemented | | |

### Wiring and mechanical (Scope B robustness)

| ID | Scenario | Hazard | Current control | Status | Test | Evidence |
|---|---|---|---|---|---|---|
| F24 | Intermittent connector on a sensor or switch | Spurious stops or bad data | Fail-safe defaults: switch open = stop, HX711 DT pull-ups = NO_DATA, IMU recovery | Partial `[FILL IN: connector / strain-relief changes]` | W1 to W4 | |
| F25 | 5 V pull-ups on the shared I2C bus | IMU damage | Documented check (SDA / SCL about 3.3 V) | Procedure only | P5 | |
| F26 | New cables create trip / snag / pinch points (NFR8) | Injury | `[FILL IN]` | `[FILL IN]` | W5 | |

## 2. Priority gaps

1. **F13 / F15 CAN loss and controller hang.** Enable the ODrive CAN watchdog (`config.can.watchdog` / axis watchdog in the Web GUI, then feed it from the assist loop) so the wheels go idle if the STM32 stops sending. Add the STM32 independent watchdog.
2. **F07 IMU loss while driving.** Let the pose machine see `imuIsPresent()` and stop the wheels (or disable the assist) when it is lost.
3. **F14 one-axis fault.** Disarm both axes when either reports an error or misses its heartbeat; refuse to arm with one axis on the assembled truck.
4. **F04 accessible e-stop.** Agree with Team 5 whether a physical e-stop cuts motor power (hardware) or signals the STM32, and add it.
5. **F19 incline limit.** Agree the maximum incline with the client and decide the response beyond it.

## 3. Evidence log

| Date | Scenario IDs | Test | Result | Log / video | Witness |
|---|---|---|---|---|---|
| 10 Nov 2026 | F11 | Flat-floor push / pull, handle high and low | Pass | `[FILL IN]` | `[FILL IN]` |
| `[FILL IN]` | | | | | |
