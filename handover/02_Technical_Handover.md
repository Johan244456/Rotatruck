# 02 Technical Handover

For the engineer who will maintain or extend the cart. It describes the system **as it is now**, with the focus on what this team changed. For the parts that did not change (ODrive configuration, CAN protocol, observer theory, motor harness) the previous team's documents are still the reference:

- [`../docs/03_software/architecture.md`](../docs/03_software/architecture.md): control architecture, momentum observer, assist loop
- [`../docs/02_branch/ODRIVE_INTEGRATION.md`](../docs/02_branch/ODRIVE_INTEGRATION.md): CAN / ODrive deep dive
- [`../docs/06_operation/maintenance-and-updates.md`](../docs/06_operation/maintenance-and-updates.md): re-flashing and ODrive setup
- [`../docs/07_handover/ODrive_S1_Setup_Guide.pdf`](../docs/07_handover/ODrive_S1_Setup_Guide.pdf): ODrive Web GUI setup (see the pin warning in section 3.4)

The authoritative sources are always the firmware files [`board.h`](../code/STM_RotaTruck/include/board.h) (pins, wiring options) and [`assist_tuning.h`](../code/STM_RotaTruck/include/assist_tuning.h) (every tuning constant, with comments on which way to move it).

---

## 1. System at a glance

```
                      +-------------------- STM32 Nucleo-G491RE (supervisor) --------------------+
 Handle switch  A0 -->|  deadman  ->  assist_control (pose, brake, release-stop ramp)            |
 ISM330DHCX  I2C1 --->|  imu  ->  observer (momentum observer + slope estimator)  ->  admittance |--> FDCAN1 --> CAN bus --> ODrive S1 #0 (LEFT)  --> hub motor
 16x2 LCD    I2C1 <---|  display                                                                  |                     \-> ODrive S1 #1 (RIGHT) --> hub motor
 HX711 x2  (D2..D5)-->|  loadcell  ->  payload (latch / manual)  ->  observer mass model          |
 Buttons D8/D9/D10 -->|  mode_input, button (B1)                                                  |
 USB (ST-Link VCP) <->|  command (console 115200 8N1, LD load-detect console)                     |
                      +--------------------------------------------------------------------------+
```

Unchanged: the supervisor + two smart drives split, the CAN bus, the momentum observer that infers the operator's push from motion, the admittance (direct torque) assist loop, the pose state machine, manual / autonomous modes.

New in this phase: handle (dead-man) switch, ISM330DHCX IMU, road-slope estimator for gravity compensation, automatic load detection with a latch into the assist, new LCD layout, load-detect console and calibration GUIs.

---

## 2. Build, flash, console

The firmware is a PlatformIO project in [`../code/STM_RotaTruck/`](../code/STM_RotaTruck/) (STM32Cube HAL, no RTOS).

```sh
pio run -d code/STM_RotaTruck              # build
pio run -d code/STM_RotaTruck -t upload    # flash over the Nucleo ST-Link (USB)
pio device monitor -d code/STM_RotaTruck   # console, 115200 8N1
```

Build checked on 11 Nov 2026: **success**, flash 90,472 B (17.3 %), RAM 2,728 B (2.4 %), `-Wall -Wextra`.

The last flash page (0x0807F800, 2 KB) holds the load-detect settings. Keep the firmware image well below 510 KB or a build will overwrite the calibration.

Expected boot log (abbreviated):

```
IMU: using ISM330DHCX for pose + observer.
LCD: 16x2 I2C display detected.
SAFETY: handle switch on A0 (PA0), active-low - RELEASED now.
LOAD: HX711 CH1 ok, CH2 ok | zero ... | view x 0 | pose gating on
RotaTruck ODrive-S1 firmware online. Type 'H' for help.
```

On a board with no load calibration it also prints `LOAD: view not calibrated -> assist payload 0 kg (NO_CAL)`.

---

## 3. Hardware

### 3.1 Current STM32 pin map

| Function | STM32 pin | Nucleo label | Notes |
|---|---|---|---|
| FDCAN1 TX / RX | PA12 / PA11 | Morpho CN10 | To CAN Pal transceiver; 250 kbit/s; unchanged |
| I2C1 SCL / SDA | PB8 / PB9 | D15 / D14 | IMU + LCD, 100 kHz, **3.3 V bus** |
| Handle (dead-man) switch | **PA0** | **A0** (CN8 pin 1) | **New.** Switch to GND, internal pull-up, active-low |
| HX711 #1 (CH1, middle strip) DT / SCK | **PA10 / PB3** | **D2 / D3** | **New.** CN9 pins 3 / 4 |
| HX711 #2 (CH2, left + right strips) DT / SCK | **PB5 / PB4** | **D4 / D5** | **New.** CN9 pins 5 / 6 |
| HX711 VCC / GND | 3V3 / GND | CN6 pins 4 / 6 | **3V3, not 5 V** |
| Load-detect status LED | PA5 | LD2 (on board) | **New.** Solid = valid, slow blink = settling / no cal / held / overload, fast blink = fault |
| Assist on/off button | PA9 | D8 | Unchanged; on-board B1 (PC13) mirrors it |
| Payload -5 kg / +5 kg buttons | PC7 / PB6 | D9 / D10 | Now set the **manual fallback** payload |
| Debug UART | PA2 / PA3 | ST-Link VCP | USART2, 115200 8N1 |

PB3 is released from JTAG for HX711 #1. SWD on PA13/PA14 (used for flashing and debugging) is not affected.

### 3.2 IMU: ISM330DHCX (replaces MPU6050)

- Part: SparkFun SEN-19764 (ST ISM330DHCX), I2C address 0x6B (0x6A also probed). **3.3 V only.**
- Why: the MPU6050 is obsolete (TDK, not recommended for new designs) and its behaviour under hub-motor vibration was unknown. The MPU6050 code path has been removed completely.
- Driver: [`ism330.c`](../code/STM_RotaTruck/src/ism330.c) (register level), [`imu.c`](../code/STM_RotaTruck/src/imu.c) (bus ownership, cart-frame rotation, caching).
- **Cart frame.** Every reading is multiplied by `IMU_AXIS_SIGN_X/Y/Z` in `board.h` so all downstream code sees the same frame the pose thresholds were tuned in. As mounted the board is rotated 180 deg about Y (signs -1, +1, -1). Measured on the truck on 10 Nov 2026 against the old sensor: pitch correlation +0.997, gyro x / z -0.995 / -0.999 before sign correction.
  - pitch = atan2(ay, az): STANDING about -43.6 deg, DRIVING (tipped back onto the wheels) about -1.2 deg
  - pitch rate = gx (+ towards DRIVING), yaw rate = gz
  - **If the board is remounted:** stream `I S`, tilt STANDING to DRIVING and back, and flip the signs in `board.h`.
- Robustness added: I2C bus clear (9 SCL pulses) at power-up, bus re-init after an I2C error, setup retried 3 times at boot then every 500 ms if it failed, and a "dead sensor" check (5 readings near 0 g in a row means the sensor lost its configuration after a brown-out, so it is reconfigured).
- **I2C pull-up warning.** The LCD's PCF8574 backpack shares the bus. If its pull-ups go to 5 V they over-drive the 3.3 V IMU. With power on, SDA and SCL must measure about 3.3 V to GND. Remove the backpack pull-ups or power it from 3V3 if not.

### 3.3 Handle (dead-man) switch

- Module: [`deadman.c`](../code/STM_RotaTruck/src/deadman.c); behaviour in [`assist_control.c`](../code/STM_RotaTruck/src/assist_control.c).
- Wiring as built: one terminal to A0, the other to GND. Pressed = LOW. Internal pull-up on A0, so a released switch, a broken wire or an unplugged connector all read RELEASED (fail-safe).
- Debounce: release accepted after 20 ms (safety side), press after 80 ms (so contact bounce cannot re-enable the assist). Starts RELEASED at boot; it must be seen pressed first.
- Part / location: `[FILL IN: switch part number, where it is mounted on the handle, how it is held (lever, push button, capacitive?), photo]`
- Options in `board.h`: `DEADMAN_ACTIVE_LOW` (set 0 for a switch to 3.3 V), `DEADMAN_ENABLE` (set 0 to ignore the switch, **bench testing only**, prints a warning at boot).

### 3.4 Load plate and strain gauges

- The toe (load) plate has three strips (left, middle, right) separated by two elliptical cut-outs. Six BF350 350-ohm gauges are bonded as three Poisson pairs (lengthwise + crosswise per strip), on one line at mid-length, same face.
- Two bridges, one HX711 each (channel A, gain 128, 10 Hz, powered from 3V3):

| Bridge | Gauges | Configuration | Arms |
|---|---|---|---|
| CH1 middle | middle pair | Poisson half bridge | R1 crosswise, R3 lengthwise, R2 / R4 350-ohm completion resistors |
| CH2 left + right | both side pairs | Poisson full bridge | R1 L crosswise, R2 R lengthwise, R3 L lengthwise, R4 R crosswise |

- Total load (view "All 6") = CH1 + g x CH2, where g (side weight) is calibrated by putting the same mass on the middle strip and then on a side strip.
- CAD for the plates: `../../Weighting Plate - Left.STL`, `../../Weighting Plate - Right.STL` (in the parent DMMS folder). `[FILL IN: confirm these are the fitted plates, material, and how they are attached to the toe plate]`
- Photos of the gauge installation: `../../SG/` (28 Sep 2026).

### 3.5 Pin documentation that is now out of date

- [`../docs/07_handover/ODrive_S1_Setup_Guide.pdf`](../docs/07_handover/ODrive_S1_Setup_Guide.pdf) lists CAN on PB9/PB8 and the IMU on I2C2 (PB10/PB11). **That is wrong for this firmware.** CAN is PA12/PA11 and the IMU is on I2C1 PB8/PB9 (D15/D14). Use the table in 3.1. The ODrive Web GUI steps in that guide are still valid.
- [`../docs/04_hardware/wiring.md`](../docs/04_hardware/wiring.md) is correct for CAN, buttons and UART but does not show A0, D2 to D5 or the new IMU.

### 3.6 Parts added this phase (bill of materials)

| Item | Qty | Part / supplier | Unit cost (AUD inc GST) | Fitted on the truck? |
|---|---|---|---|---|
| ISM330DHCX IMU breakout | 1 | SparkFun SEN-19764 | 44.85 | Yes |
| HX711 load-cell amplifier | 2 | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` |
| BF350 strain gauge (350 ohm) | 6 (+ spares) | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` |
| 350-ohm completion resistors | 2 | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` |
| Handle switch | 1 | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` |
| Weighing plates (printed / machined) | 2 | In-house | `[FILL IN]` | `[FILL IN]` |
| IMU test rig items (ESP32-S3 bench, fixtures) | | See `../../DMMS_BOM.xlsx` | | Bench only |
| **Total this phase** | | | `[FILL IN]` | |

The previous team's BOM is in [`../docs/04_hardware/bill-of-materials.md`](../docs/04_hardware/bill-of-materials.md). The MPU6050 and the DFRobot compression load cells listed there are no longer used.

### 3.7 Wiring, connector, mounting and enclosure record (Scope B robustness)

Under the approved scope, Scope B owns wiring, connector, mounting and enclosure robustness. This team's own prototype audit (photos in `../../Evid/`) found sealed connectors at the wheel motors but unsleeved, unsecured wiring elsewhere, in-line spliced connectors, and no panel-mounted power input jack.

| Harness / item | Before (audit) | Change made | Connector / part | Strain relief / routing | Date / by |
|---|---|---|---|---|---|
| Power input (24 V) | Spliced in-line connectors, no panel jack | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | |
| CAN bus (STM32 to both ODrives) | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | |
| I2C (IMU + LCD) | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | |
| Handle switch (A0, GND) | New | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | |
| HX711 x2 and gauge leads | New | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | |
| Front-panel buttons | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | `[FILL IN]` | |
| Electronics bracket / enclosure | 3D-printed PLA lower bracket, components on M3 screws | `[FILL IN]` | | | |
| IMU mounting | GY-521 MPU6050 | SparkFun ISM330DHCX `[FILL IN: mount, standoffs]` | | | |

`[FILL IN: wiring diagram reference, IP target agreed with the client, interface agreements with Team 5 (connector standard, power budget).]` Test procedure: [06](06_Testing_and_Commissioning.md), section 7.

---

## 4. Firmware structure

Single cooperative super-loop in [`main.c`](../code/STM_RotaTruck/src/main.c). Each stage paces itself; nothing blocks except the I2C reads (about 1.5 ms) and the `LD` console calibration commands (see 6.6).

```c
while (1) {
    commandPoll();      // serial console (motor console or LD console)
    buttonPoll();       // B1 toggles the assist
    observerUpdate();   // reads the IMU once, force + slope estimate (~100 Hz)
    loadcellUpdate();   // HX711s when ready (10 Hz) + load pose gating      NEW
    deadmanPoll();      // handle switch, debounced                          NEW
    assistUpdate();     // pose, brake/release, handle-release stop ramp     CHANGED
    autoseqUpdate();    // drive-entry calibrate (+ auto-arm in AUTO mode)
    modeInputUpdate();  // D8 assist toggle, +/- manual payload              CHANGED
    payloadUpdate();    // latch load on STANDING -> DRIVE, push to observer NEW
    admittanceStep();   // assist torque loop (~100 Hz, gated)               CHANGED
    displayUpdate();    // LCD, one byte per pass                            CHANGED
}
```

### 4.1 Module map

| Module | Role | Status this phase |
|---|---|---|
| `main.c` | Init order and super-loop | Changed: new modules wired in |
| `board.h` | Pins, IMU axis signs, switch / HX711 options | Changed |
| `assist_tuning.h` | All tuning constants | Changed: slope estimator, dead-man ramp, gravity comp auto-on |
| `deadman.c/.h` | Handle switch reading, fail-safe debounce | **New** |
| `ism330.c/.h` | ISM330DHCX register driver | **New** |
| `imu.c/.h` | IMU layer: bus, cart frame, cache, recovery | Rewritten (MPU6050 removed) |
| `loadcell.c/.h` | Load detection (port of bench rig v4.2, changes marked `[RT]`) | **New** (about 1,350 lines) |
| `payload.c/.h` | Chooses the assist payload: latched load or manual | **New** |
| `observer.c/.h` | Momentum observer; **slope estimator** added; reseed after a release stop | Changed |
| `assist_control.c/.h` | Pose machine; **release-stop ramp** (MM_STOP_RAMP); handle gate | Changed |
| `admittance.c/.h` | Assist loop; refuses to arm while the `LD` console is open | Changed |
| `autoseq.c/.h` | Drive-entry calibration; also resets the slope estimate | Changed (message / slope) |
| `motor_control.c/.h` | ODrive wrapper; `motorControlEnterVelocityHold()` added for the ramp | Changed |
| `command.c/.h` | Console; `W`, `W <kg>`, `W A`, `LD`, `I2C`, `I B`; junk-byte filter | Changed |
| `display.c/.h`, `lcd_i2c.c/.h` | LCD layout and driver | Changed |
| `mode_input.c/.h` | Front-panel buttons; +/- now go to `payload.c` | Changed |
| `odrive_can.c/.h`, `button.c`, `pid.c` | CAN protocol, B1 button, unused PID | Unchanged |

A file-by-file list is in [07 Change Log](07_Change_Log.md).

---

## 5. Control changes

### 5.1 Slope-based gravity compensation (fix for the tilt / incline limitation)

**Problem found.** The previous firmware computed gravity compensation from the frame pitch (relative to the pitch at drive entry). A frame-mounted IMU cannot tell "handle held higher" from "cart on a ramp": both rotate the frame against gravity the same way. So every change of handle angle on flat floor was read as a slope, and the gravity term produced a phantom push or pull. We believe this is the cause of the previous team's Limitation 1 ("bidirectional assist does not engage correctly when the cart is held at an angle"). `[VERIFY: confirm with the test record that this was the observed symptom.]`

**Fix.** The slope is now estimated from **acceleration**, in [`observer.c`](../code/STM_RotaTruck/src/observer.c) (`slope_estimator_update`):

1. A complementary filter (gyro + accelerometer) tracks the frame tilt, trusting the accelerometer only when the cart is not accelerating (|a_wheel| < 0.08 m/s^2).
2. The IMU specific force is projected onto true vertical: f_v. At rest on any slope f_v = 1 g.
3. When the cart speeds up or slows down along the road, the vertical part of that acceleration is a_wheel x sin(slope), so
   `sin(slope) = (f_v - f_v_rest) x g / a_wheel`, with a_wheel = d/dt of the wheel speed from the ODrive encoders.
4. Handle angle drops out: it changes the frame tilt (tracked by the filter) but not the vertical acceleration.
5. The estimate only updates while |a_wheel| > 0.25 m/s^2, the frame is not swinging (pitch rate < 5 deg/s) and the cart is rolling (> 0.05 m/s). It holds its value between pushes and resets to 0 on every drive entry.
6. Gravity force = M_trans x g x sin(slope beyond a +/-2.5 deg deadband), soft (no step at the deadband edge), slope clamped to +/-12 deg.

Tuning constants: `SLOPE_*` and `GRAVITY_SLOPE_DEADBAND_DEG` in `assist_tuning.h`. Gravity compensation is switched on automatically at every drive entry (`GRAVITY_COMP_AUTO_ENABLE 1`).

**Evidence so far.** Flat-floor test on 10 Nov 2026: the estimate stayed within about +/-1 deg in steady pushes and reached +/-2 to 3 deg at direction reversals (which is why `SLOPE_MAX_PITCH_RATE_DPS` was lowered from 10 to 5 and the deadband set at 2.5 deg). No phantom push on flat floor. **Ramp test not done yet.** Use `OB S` on a ramp: the slope field should read close to the ramp angle.

Limits of the method: the slope is only learnt while the cart is accelerating, so the first push on a ramp starts with slope = 0 (the estimate converges with a 0.6 s time constant once the cart accelerates). Gentle ramps under 2.5 deg get no gravity help by design.

### 5.2 Handle-release stop

When the switch is released while the assist is driving the wheels, `assist_control.c` enters `MM_STOP_RAMP`:

1. Both ODrives switch to velocity control **at their current speed** (`motorControlEnterVelocityHold`), so there is no jolt.
2. Both setpoints ramp to 0 at `DEADMAN_DECEL_REV_PER_S2` = 1.5 rev/s^2 (about 0.78 m/s^2, about 1.3 s from 1 m/s). This is a controlled stop, not a freewheel and not a slam.
3. The wheels are then held at 0 (`Pose: stopped - holding at 0`).
4. If the cart was already braked (STANDING, tilting, anti-rollback), it stays braked.
5. Pressing the switch again hands back to the normal pose logic. The observer momentum is re-seeded from the current motion so the stop itself is not read as an operator push.

`assistDriveActive()` (the gate on the assist loop) now also requires the switch pressed and no stop ramp in progress. In dev mode (no IMU) the switch still gates the assist.

### 5.3 Pose detection with the new IMU

Same state machine and thresholds as before (`POSE_DRIVE_ENTER_DEG -5`, `POSE_STANDING_ENTER_DEG -15`, 200 ms debounce, 1 s anti-rollback, tilt protection at 15 deg/s with 250 ms settle). The thresholds still sit between the newly measured STANDING (-43.6 deg) and DRIVING (-1.2 deg) angles. `[VERIFY: DRIVING at -1.2 deg leaves only 3.8 deg of margin to the -5 deg threshold; check with M S that a low handle position does not drop the cart out of DRIVE.]`

### 5.4 Payload latch (load detection into the assist)

Implemented in [`payload.c`](../code/STM_RotaTruck/src/payload.c). The payload sets the observer's translational mass and friction scale (not a speed target).

1. **Arm:** switch the assist ON (D8, B1 or `AS 1`) while the cart is STANDING.
2. While standing, the last **valid** live reading (status OK or OVERLOAD) is remembered, as 0.9 x measured, capped at 150 kg.
3. **Latch:** on STANDING to DRIVE, that reading becomes the assist payload and stays fixed for the push. Each later STANDING to DRIVE in the same armed session re-latches.
4. **Manual fallback:** assist switched on outside STANDING, or no valid reading while standing, means the manual value is used (buttons or `W <kg>`, starts at 0 kg, not saved). Pressing +/- during a push switches to manual starting from the value in use. `W A` returns to the latched value.
5. Changes are pushed to the observer only when the value changes (each push re-seeds the momentum estimate).

**Interaction to be aware of:** in autonomous mode (`R A`) the assist arms itself in DRIVE, which is "outside STANDING", so the latch never arms and the manual value is used. See Limitations.

---

## 6. Load detection

Full engineering note: [`../code/STM_RotaTruck/LOAD_DETECTION.md`](../code/STM_RotaTruck/LOAD_DETECTION.md). Summary:

### 6.1 Signal chain (per 10 Hz sample)

1. Read both HX711s when ready (bit-banged, interrupts off for about 60 us, DWT cycle counter for timing). The main loop never waits.
2. Per channel: median of 5, zero, sign.
3. View value: CH1 only (`x 1`), CH2 only (`x 2`) or CH1 + g x CH2 (`x 0`, the view the assist uses).
4. Filter: stable-hold (default, like a kitchen scale) or smooth low-pass.
5. Rest zero: while the IMU says the cart is tipped back and still, the zero follows the reading, using samples 1.1 to 1.8 s old so the moment someone grabs the cart never reaches the zero. A contact on the plate is refused, not absorbed.
6. Load: `N = (reading - offset) / slope`, per view.
7. Status and assist value: live only when standing (within 5 deg of the standing pose) and untouched for 1 s; HELD while tilted or driven; 0 on any fault.

### 6.2 Why the calibration has an offset

The cart is back-heavy. Empty, it rests tipped back and only stands on its toe once the load outweighs the back (or someone holds it). So the zero is taken tipped back and empty, and the standing readings form a straight line that does not pass through that zero. Bench result (middle strip): about 3.1 microstrain per kg with an offset of about -39 microstrain (about 12.6 kg). With slope only, 15 kg reads about 2.4 kg; with slope and offset both read correctly. The firmware stores slope and offset per view (`s <counts/N> <offset>`).

### 6.3 Status codes

| Code | Name | LCD | Meaning |
|---|---|---|---|
| 0 | OK | `OK` | Valid live reading |
| 1 | SETTLING | `SETL` | Moved in the last second, or not settled |
| 2 | NO_CAL | `NOCAL` | View not calibrated, payload 0 |
| 3 | RAILED | `RAIL` | HX711 at full scale |
| 4 | FROZEN | `FROZ` | 20 identical raw samples |
| 5 | NO_DATA | `NODAT` | An HX711 silent for 500 ms (loose DT wire) |
| 6 | IMPLAUSIBLE | `IMPL` | Below -5 kg or above 1.5 x rated |
| 7 | OVERLOAD | `OVER` | Above rated (valid, capped) |
| 8 | HELD | `HELD` | Not standing, value held from the last standing reading |
| 9 | IMU_FAULT | `IMU!` | No IMU, payload 0 |

### 6.4 Calibration routine (summary)

Done in `host/stm_dashboard.py` with **Capture zero + load** (paired capture). Capture the IMU poses first in the `LD` console: `q s` standing with at least 15 kg on, hands off; `q r` empty and tipped back.

1. Tip back, empty, hands off: capture the zero (refused unless tipped back and still).
2. Stand it up, place 15 kg, let go, wait about 10 s: continue with the **total** mass entered (refused unless standing).
3. Repeat for 30 kg, 40 kg, ... each with a fresh tipped-back zero.
4. Repeat steps 1 to 3 two or three times.
5. Check the fit: R^2 about 0.999 or better, no curve in the residuals, a sensible offset.
6. APPLY: sends `s <slope> <offset>` and saves to flash.

Rules: every calibration load must stand the cart by itself (15 kg minimum); never capture while someone holds the cart upright; do not use `c <N>` (one-point, forces offset 0). Order of views: 2 SG, 4 SG, balance g, then All 6.

**Recalibrate whenever the battery, motors, enclosure or handle are moved, added or removed.** The offset is a property of the cart's own weight and balance.

### 6.5 Settings storage

Last flash page (0x0807F800), layout version 6 (a v4.1 layout is migrated). Holds zeros, per-view slope and offset, signs, side weight g, completion resistors, rated payload, pose references. Saving stalls the CPU for about 20 ms; it only happens from console commands while the assist is off.

### 6.6 The `LD` console

`LD` toggles a maintenance console (`LD 1` / `LD 0` force it). Inside it all bench-rig commands work unchanged (`d b v x t z c s u q g i n f k m h a p r`) and a 10 Hz CSV streams:

```
t_ms,raw,offset_raw,hx_mV,bridge_uV,microstrain,newtons,valid,status,assist_kg,assist_pct,pose,tilt_deg
```

**The assist is switched off on entry and cannot be armed while the console is open**, because `z`, `c` and `d` block the loop for several seconds. Outside the console load detection keeps running silently and only prints faults and recovery (`# LOAD NO_DATA: ...`).

---

## 7. Console commands added or changed

The full list is printed by `H`. The previous reference is [`../docs/03_software/command-reference.md`](../docs/03_software/command-reference.md); these are the differences:

| Command | Action |
|---|---|
| `W` | Payload in use (LATCHED or MANUAL), last latch, manual value, latch armed?, load status |
| `W <kg>` | Set the manual payload (same as the buttons), not saved |
| `W A` | Back to this session's latched value |
| `LD` / `LD 1` / `LD 0` | Toggle / open / close the load-detect console |
| `I S` | Toggle 10 Hz raw 6-DOF IMU stream in the cart frame (g, dps, tilt) |
| `I2C` | Scan the I2C bus (lists IMU / LCD addresses) |
| `I B` | 2 s still test: gyro bias and noise, |a| (assist must be off) |
| `M` | Now also shows the handle switch state (PRESSED / RELEASED) |
| `OB S` | Observer stream; includes the estimated slope |

The console now ignores non-printable bytes (some USB serial ports send a junk byte on open).

---

## 8. Host tools (`code/STM_RotaTruck/host/`)

| File | Purpose |
|---|---|
| `stm_dashboard.py` | Calibration dashboard launcher (views 2 SG / 4 SG / All 6). Sends `LD 1` on connect and `LD 0` on close |
| `stm_calib_gui.py` | Tkinter + matplotlib GUI: live trace, health banner, paired capture, least-squares fit with residuals, APPLY |
| `stm_dual_test.py` | Drift / stability test of both bridges (`x 3` stream), with markers; writes CSV, summary and plot |
| `stm_bench_common.py` | Serial reader and CSV logger shared by the GUIs |

Run with the PlatformIO serial monitor closed:

```
cd code/STM_RotaTruck/host
py -3.13 stm_dashboard.py
py -3.13 stm_dual_test.py
```

Data is written to `code/STM_RotaTruck/data/strain_gauge/`. Unlike the bench-rig version, the dashboard **keeps** the board's saved calibration on launch (type `u` in its command box to clear it on purpose). Requires Python 3 with `pyserial`, `matplotlib`, `numpy` `[VERIFY: package list]`.

**Note on `host/stm_calibration_combined.json`:** this record (9 Nov 2026) has two points that disagree in sign and magnitude (15 kg = +98,516 counts paired, 30 kg = -2,552,086 counts single). Treat it as invalid; recalibrate on the truck. The JSON files are records only and are never loaded by the board.

---

## 9. Tuning constants added or changed (`assist_tuning.h`)

| Constant | Value | Purpose / which way to move it |
|---|---|---|
| `SLOPE_LPF_ALPHA` | 0.10 | LPF on f_vertical and a_wheel (about 95 ms) |
| `SLOPE_MIN_ACCEL_MPS2` | 0.25 | Minimum wheel acceleration before the slope updates |
| `SLOPE_MAX_PITCH_RATE_DPS` | 5.0 | Frame must swing slower than this (10 let reversal rocking leak in) |
| `SLOPE_MIN_SPEED_MPS` | 0.05 | Must be rolling |
| `SLOPE_TAU_S` | 0.6 | Slope estimate time constant while updating |
| `SLOPE_MAX_DEG` | 12.0 | Clamp |
| `SLOPE_TILT_CF_K` | 0.02 | Accelerometer weight in the tilt filter (about 0.5 s) |
| `SLOPE_CF_MAX_ACCEL_MPS2` | 0.08 | Trust accelerometer tilt only below this |
| `GRAVITY_SLOPE_DEADBAND_DEG` | 2.5 | Raise if flat floor shows phantom push; lower if gentle ramps get no help |
| `GRAVITY_COMP_AUTO_ENABLE` | 1 | Gravity comp on at every drive entry |
| `DEADMAN_DECEL_REV_PER_S2` | 1.5 | Release stop rate. Lower if abrupt or regen trips the bus; raise if it rolls too long |

Unchanged and **still not measured on this cart**: `TROLLEY_MASS_EMPTY_KG` 15, `TROLLEY_WHEELBASE_M` 0.35, `WHEEL_INERTIA_KGM2` 0.007, `YAW_INERTIA_KGM2` 0.7, friction constants, `MOTOR_KT_NM_PER_A` 0.951. See Limitations.

Board-level options (`board.h`): `IMU_AXIS_SIGN_*`, `DEADMAN_*`, `LCD_ROW1_SHOW_HANDLE` (1 = show handle state on LCD row 1, 0 = show load status), `UI_PAYLOAD_STEP_KG` 5, `UI_PAYLOAD_MAX_KG` 250.

---

## 10. Technical debt and code notes

- **Stale comments.** `assist_control.c` header still says pose = atan2(ax, az) with STANDING about -40 deg and DRIVING about +10 deg; the code uses atan2(ay, az) in the cart frame with -43.6 / -1.2 deg. The pose section of `assist_tuning.h` says DRIVING reads about +5 deg. `LOAD_DETECTION.md` describes LCD row 1 as showing load status, but `LCD_ROW1_SHOW_HANDLE` is 1 (handle state). `board.h`'s header still says "ODrive S1 branch".
- **Requirement numbers in code comments** (for example "spec FR5" for load detection, "FR1 / FR3" for the dead-man) refer to Project Specification v1.0 (approved 2 Sep 2026), the same numbering used in [04](04_Status_and_Limitations.md). The older Sprint 1 draft (`Project Specification.docx`) numbers them differently; ignore it.
- **Safety gaps in code** (CAN loss, IMU loss while driving, one-axis fault, no watchdogs): see [08 Safety Analysis](08_Safety_Analysis.md), section 2.
- **Yaw channel unused.** The observer computes a twist (steering) estimate but `admittance.c` sends the same torque to both wheels. Differential steering assist is not implemented.
- **No cross-axis fault handling** (if one ODrive faults the other is not stopped automatically), **ODrive CAN watchdog not enabled**, **hard fault handler is a bare loop** with no LED or UART output. Inherited, still open.
- **`pid.c`** is unused.
- **Repository.** The working copy has no git history (`master` has no commits) and the bench-rig firmware (`../../Code_STM/P102_LoadDetect_Main`, `P102_LoadDetect_WTests`, `IMU`, `Switch`) lives outside the repository. Commit the repository and move the bench projects under `archive/` before handing over.

---

## 11. Where things are

| Item | Location |
|---|---|
| Delivered firmware | `code/STM_RotaTruck/` |
| Load-detection engineering note | `code/STM_RotaTruck/LOAD_DETECTION.md` |
| Calibration / test GUIs | `code/STM_RotaTruck/host/` |
| Load-detect bench rig (Nucleo, v4.2) and its data | `../../Code_STM/P102_LoadDetect_Main/` (outside the repo) |
| Bench rig with tests | `../../Code_STM/P102_LoadDetect_WTests/` (outside the repo) |
| IMU and switch bench code | `../../Code_STM/IMU/`, `../../Code_STM/Switch/` (outside the repo) |
| Weighing plate CAD | `../../Weighting Plate - Left.STL`, `../../Weighting Plate - Right.STL` |
| Free-body diagrams (flat, incline) | `../../FBD_Flat.png`, `../../FBD_Incline.png` (+ `.drawio`) |
| IMU procurement plan / BOM | `../../DMMS_BOM.xlsx` |
| ESP32-S3 IMU bench pinout | `../../DMMS_Pinout.xlsx` |
| Prototype audit photos / videos | `../../Evid/`, strain-gauge photos `../../SG/` |
| Previous team's handover | `docs/07_handover/` |
| This team's review of the previous handover | `RotaTruck_Handover_Summary.pdf` |
| Project specification (approved v1.0) | `code/STM_RotaTruck/Team 4 Rotatruck Project Specification.pdf` (`Project Specification.docx` at the root is the superseded Sprint 1 draft) |
| Client next-phase scope (Scope A / B split) | `../../Rotatruck_NextPhase_Scope.pdf` |
| `[FILL IN: design report, test report, meeting minutes, CAD of any new brackets]` | `[FILL IN]` |
