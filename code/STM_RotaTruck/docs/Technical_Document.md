# Technical Document: RotaTruck on the STM32F407 Discovery

This document is for the engineer who will maintain or extend the cart. It describes the system **as it is now**, after the move from the Nucleo-G491RE to the **STM32F407G-DISC1**, including the handle-driven load workflow and the LCD.

It follows the structure of [`handover/02_Technical_Handover.md`](../../../handover/02_Technical_Handover.md). That document is still the reference for what this port didn't change: the momentum observer, slope-based gravity compensation, the release-stop ramp, the strain-gauge plate design and the ODrive setup.

The authoritative sources are the firmware files:

- [`board.h`](../include/board.h): pins, wiring options, payload, handle and calibration settings.
- [`assist_tuning.h`](../include/assist_tuning.h): tuning constants.

Where this document and the code disagree, the code wins. Operator instructions: [User_Manual.md](User_Manual.md). Load-detection details: [`LOAD_DETECTION.md`](../LOAD_DETECTION.md).

Status: as of 12 Nov 2026. GitHub `main` is at `8b5dca9` (11 Nov). **The 12 Nov changes listed in section 2 are on the board but not yet pushed.** `[FILL IN: ...]` marks input still needed; `[VERIFY: ...]` marks facts not confirmed on the cart.

---

## 1. System at a glance

```
                      +------- STM32F407G-DISC1 (168 MHz, supervisor) -------+
 Handle switch PE7 -->| deadman: hold / click / triple-click                 |
                      |   -> assist_control (arm, pose, stop ramp), payload  |
 ISM330DHCX  I2C1 --->| imu -> observer (momentum + slope) -> admittance     |
  (PB8/PB9, 3V)       |                                                      |
 HX711 CH1 PE11/12 -->| loadcell (final calib locked, zero, status)          |
                      |   -> payload (AUTO 0.7x / MAN 1.0x) -> observer mass |
 16x2 LCD  I2C3 <-----| display (own bus PA8/PC9, 5V backpack)               |
 LED LD4 PD12 <-------| load status                                          |
 ST-Link mini-USB <-->| SWD flash + RTT console (command.c, PL log, LD, LCD) |
                      +--------------------------+---------------------------+
                                                 | CAN2 PB12 / PB13, 250 kbit/s
                                     TJA transceiver (5 V)
                                      |                     |
                         ODrive S1 #0 (LEFT)      ODrive S1 #1 (RIGHT)

 Not fitted: HX711 CH2 (PE13/PE14), front-panel buttons
 Laptop: tools/rtt_server.sh (OpenOCD, socket :9090) -> host/live_load_log.py, GUIs
```

**Unchanged from the Nucleo build:**

- The supervisor plus two smart drives.
- The momentum observer and the admittance (direct torque) assist loop.
- Slope-based gravity compensation and the handle-release stop ramp.
- The load-detection signal chain, status codes and settings layout v6.

**New on the F407:**

- **Handle-only operation**: hold arms the assist, a click accepts the load, a triple-click zeroes.
- **One pose definition**: the captured `q s` / `q r` references are used by both the scale and the motors. The assist drives only when **fully** tipped back.
- **Load**: auto-read when exactly upright; AUTO 0.7× / MANUAL 1.0×; final calibration locked; zero → MANUAL.
- **LCD on its own I2C bus.**
- **Robustness**: IMU recovery, zero refused on a stuck HX711, steady-reading requirement for the auto-read.

---

## 2. What changed from the Nucleo-G491RE build

| Area | Nucleo-G491RE | STM32F407G-DISC1 (now) |
|---|---|---|
| MCU / clock | G491RE, 170 MHz, FDCAN | F407VG, 168 MHz from 8 MHz HSE ([`clock_f4.c`](../src/clock_f4.c)), bxCAN |
| CAN | FDCAN1 PA11/PA12 | **bxCAN CAN2 PB12 (RX) / PB13 (TX)**, filter bank 14 |
| Console | USART2 via ST-Link VCP | **SEGGER RTT** over SWD ([`rtt.c`](../src/rtt.c)). USART2 still initialised. |
| Handle switch | A0 (PA0) | **PE7** (PA0 is the USER button on the DISC1) |
| HX711 CH1 / CH2 | PA10/PB3, PB5/PB4 | **PE11/PE12**, PE13/PE14 (CH2 not fitted) |
| Load LED | LD2 (PA5) | **LD4 (PD12)** |
| LCD | Shared I2C1 with the IMU | **Own bus I2C3, SCL PA8 / SDA PC9, backpack on 5V.** No level shifter, IMU bus stays 3.3 V. |
| Settings flash | Last page 0x0807F800 | **Sector 11, 0x080E0000** (128 KB) |
| HX711 gain | 128 | **64** (±22.66 mV) |
| Calibration | From flash, changeable | **Final bench calibration locked in firmware** (`LC_FINAL_*`) |
| Buttons | D8/D9/D10 + B1 | Disabled (`UI_BUTTONS_ENABLE 0`); B1 (PA0) still toggles the assist |
| Arming | Button [A] / `AS 1` | **Hold the handle** (`DEADMAN_ARMS_ASSIST 1`) |
| Payload | Latched STANDING → DRIVE, 0.9× | **Auto-read when upright (steady 1.5 s) or 1 click** → AUTO 0.7×; MANUAL (default 15 kg, `W`, after a zero) 1.0× |
| Motor pose | Fixed pitch thresholds | **Captured references**; DRIVE only within 5° of `q r` |
| LCD layout | `ASSIST:ON 30kg` / `DRIVE FWD PRESS` | `UPRIGHT 30.2kg` / `AUTO 21.0kg OFF` (section 10) |

**12 Nov 2026 changes (on the board, not yet pushed):**

1. LCD on its own bus I2C3 (PA8/PC9), with BUSY recovery and the `LCD` diagnostic command.
2. New LCD layout. Out-of-range readings show the actual value instead of IMPL.
3. MANUAL load gives 1.0× assist (AUTO stays 0.7×).
4. Final calibration locked; calibration-changing commands refused.
5. Zeroing refused while the HX711 is at full scale.
6. Auto-read safety: only a steady reading (±0.5 kg / 1.5 s); a `W <kg>` manual load pauses the auto-read.
7. A click takes the most recent valid upright reading. No upper weight limit.
8. A triple-click zero sets the load back to MANUAL.
9. `q s` re-captured on the trolley after the IMU moved.

---

## 3. Build, flash, console

| Env | Builds | Use |
|---|---|---|
| `disco_f407vg` | Full firmware | The cart |
| `imu_test` | [`imu_test_main.c`](../src/imu_test_main.c) | IMU, handle switch, CAN listen-only bring-up |
| `loadcell_test` | [`loadcell_test_main.c`](../src/loadcell_test_main.c) + `loadcell.c` | Bench load detection only (IMU off, pose simulator `q f s/r/t/0`) |

```sh
pio run -e disco_f407vg                 # build
pio run -e disco_f407vg -t upload       # flash over the ST-Link (mini-USB)
pio device monitor -e disco_f407vg      # console over RTT (socket://localhost:9090)
```

Build checked on 12 Nov 2026: **success**, flash **95,804 B (9.1 %)**, RAM 7,132 B (5.4 %). Keep the image below 896 KB (sector 11 holds the load settings).

**One program at a time owns the ST-Link.** Close the PlatformIO monitor or `tools/rtt_server.sh` before uploading. After a USB drop, the OpenOCD session goes stale and must be restarted.

Expected boot log:
```
ODrive CAN init OK — waiting for first heartbeat...
IMU: using ISM330DHCX for pose + observer.
LCD: 16x2 I2C display detected (own bus I2C3, SCL PA8 / SDA PC9).
SAFETY: handle switch on PE7, active-low - RELEASED now.
LOAD: HX711 CH1 ok, CH2 MISSING | zero saved | view x 1 | pose gating OFF
LOAD: FINAL calibration locked: view x 1, s 220.3704 -2086.4, zero -3028324, gain 64 (board.h LC_FINAL_*)
PAYLOAD: initialised load 15.0 kg at power-up ... -> assist payload 15.0 kg (1.00 x).
```

---

## 4. Hardware

### 4.1 Pin map (STM32F407G-DISC1)

| Function | Pin | Notes |
|---|---|---|
| CAN2 RX / TX | **PB12 / PB13** | TJA transceiver (VCC on **5V**): red = RX, brown = TX. 250 kbit/s. |
| IMU I2C1 SCL / SDA | **PB8 / PB9** | ISM330DHCX 0x6B, 100 kHz, **3V** |
| LCD I2C3 SCL / SDA | **PA8 / PC9** | PCF8574 backpack 0x27 (0x20–0x27 / 0x38–0x3F probed), VCC **5V**. Pins are 5 V tolerant, no internal pull-ups. |
| Handle switch | **PE7** | To GND, internal pull-up, active-low |
| HX711 #1 (middle) DT / SCK | **PE11 / PE12** | VCC 3V |
| HX711 #2 (sides) DT / SCK | PE13 / PE14 | Not fitted |
| Load LED | PD12 (LD4) | |
| USER button | PA0 | Toggles the assist |
| SWD | PA13 / PA14 | Flash + RTT. Keep the handle wiring away from it. |

### 4.2 Power

| Rail | Source | Loads |
|---|---|---|
| DISC1 | Laptop over the ST-Link mini-USB | MCU; **3V**: IMU, HX711; **5V**: CAN transceiver, LCD |
| ODrives | 24 V battery / supply | Motors |

All grounds must be common.

- **The 3V/GND rail to the breadboard is marginal.** On 12 Nov the IMU repeatedly reset ("reads ~0 g") while the HX711 returned −1 at the same time. Solder these, or use one short, solid wire per rail.
- **USB drops** (cable or socket) restart the board. Each restart loses the zero and the accepted load. `[FILL IN: on-cart controller supply.]`
- The ODrive CAN watchdog isn't configured. `[VERIFY: axes go IDLE when the STM32 stops.]`

### 4.3 CAN (bxCAN CAN2)

[`odrive_can.c`](../src/odrive_can.c):

- PCLK1 42 MHz, prescaler 12, BS1 11, BS2 2 (85.7 % sample point).
- Filter bank 14; the CAN1 clock is required.
- Node 0 = LEFT, 1 = RIGHT.

To diagnose, use `OD` and the ESR register. **LEC 3 (ACK error)** = TX works, nobody answers (ODrives off, or CAN H/L open). **LEC 5** = TX broken (the brown TX wire, 11 Nov).

### 4.4 IMU (ISM330DHCX)

- Driver: [`ism330.c`](../src/ism330.c), [`imu.c`](../src/imu.c), with cart-frame signs (−1, +1, −1) in `board.h`.
- **Recovery:**
    - Full I2C bus recovery on every retry (every 500 ms).
    - A "dead sensor" check: about 0 g triggers a reconfiguration.
    - The IMU comes back without a reset once its wiring is good.
- **IMU lost:** the load status is `IMU_FAULT`, the pose drops to STANDING (wheels held), and load acceptance and zeroing are refused.
- **The IMU board must be rigidly mounted.** On 12 Nov it moved while its wires were re-seated: upright then read 81° from the reference, and nothing could reach AUTO. Re-capture `q s` / `q r` after any change (section 7).

### 4.5 Handle switch

- Wiring: PE7 to GND, active-low. A broken wire reads RELEASED (fail-safe).
- Debounce: release 20 ms, press 80 ms.
- `[FILL IN: part, mounting, photo.]`

### 4.6 Load plate, bridge and HX711

- **Middle strip only** (CH1, Poisson half bridge: 2 gauges + 2 × 350 Ω completion resistors). The assist uses view `x 1`.
- **Gain 64**: input range ±22.66 mV. Sensitivity: **1 kg ≈ 2,160 counts ≈ 6 µV**, so **1 Ω of contact resistance in one arm ≈ 330 kg**.
- **Bridge balance** (A+ to A− at rest) must stay well inside ±22.66 mV:
    - Measured about −8 mV at the bench calibration.
    - **20–26 mV** on 12 Nov (rail and huge offsets).
    - Later 0.2 / +1.9 / −7.9 mV as the wiring was reworked.
- **Observed faults and causes** (details in `LOAD_DETECTION.md`):
    - RAIL / thousands of kg: imbalance or an open arm.
    - Jumps of 100–1000 kg: intermittent joints.
    - Smooth runaway: a floating input or creep.
    - A one-time step of 100–400 kg on the first load: the plate seating on its mounting.
- **Solder every bridge connection, add strain relief, fix the plate mounting.**

### 4.7 Wiring record and open hardware faults

| Item | State (12 Nov 2026) | Action |
|---|---|---|
| Strain-gauge bridge and E±/A± wires | Intermittent, unbalanced at times, drifts | Solder, strain relief, check A+/A− under ±5 mV |
| Plate mounting | Shifts on the first load after a rest | Tighten / shim |
| 3V / GND rail (IMU + HX711) | Marginal, brown-outs | Solder or replace the jumpers |
| IMU wires and mounting | SDA and SCL came loose, the board moved | Qwiic cable or solder; mount rigidly |
| LCD SCL wire | Came off twice | Secure it |
| USB cable | Frequent drops, which restart the board | Replace; independent controller supply |
| CAN TX (brown) | Intermittent (11 Nov) | Solder or crimp |
| First LCD / backpack | Held SDA low when its GND was connected | Suspect, don't use; the second LCD works at 0x27 |

---

## 5. Firmware structure

The main loop in [`main.c`](../src/main.c) is unchanged: `commandPoll → buttonPoll → observerUpdate → loadcellUpdate → deadmanPoll → assistUpdate → autoseqUpdate → modeInputUpdate → payloadUpdate → admittanceStep → displayUpdate`.

| Module | Change on the F407 |
|---|---|
| `clock_f4.c`, `stm32f4xx_it.c`, `stm32f4xx_hal_msp.c` | New (F4 clock, interrupts, MSP including I2C3 for the LCD) |
| `rtt.c`, `bringup_io.c`, `*_test_main.c` | New (RTT console, bring-up programs) |
| `odrive_can.c` | bxCAN CAN2 |
| `deadman.c` | Gestures: hold, click, triple-click |
| `assist_control.c` | Arm on hold, disarm after the stop; pose from the captured references |
| `payload.c` | Auto-read (steady window), click accept, manual lock, zero → MANUAL, AUTO 0.7× / MAN 1.0× |
| `loadcell.c` | Gain 64; F4 flash; **final-calibration lock**; **zero refused at the rail**; 3 s / ±3 kg auto-zero; handle zero; pose getters |
| `imu.c` | Full bus recovery per retry; `IMU_ENABLE` |
| `lcd_i2c.c` | **Own I2C3 bus**, bus clear and BUSY-erratum recovery, address search, `LCD` diagnostics (line pull-up test, software scan) |
| `display.c` | New two-line layout (section 10) |
| `command.c` | RTT input; `PL`, `W`, `LCD` |

---

## 6. Handle switch gestures

| Gesture | Rule (board.h) | Effect |
|---|---|---|
| **Hold** | ≥ `DEADMAN_HOLD_MS` 400 ms | Arms the assist (closed loop, friction and gravity compensation). Refused if no ODrive answers. |
| **Release** | 20 ms debounce | Ramp to 0 at 1.5 rev/s². Once stopped for `DEADMAN_DISARM_HOLD_MS` 500 ms: **disarm, axes IDLE, wheels free**. (Original: wheels held at 0.) |
| **1 click** | ≤ 350 ms, no 2nd click within 500 ms | Accept the load (section 8) |
| **3 clicks** | Gaps ≤ 500 ms | Zero (tipped back, IMU OK, HX711 not at its limit) and **load → MANUAL** |

---

## 7. Pose: one definition for the whole system

**References** (saved in flash):

- `q s` = upright, at least 15 kg on, hands off.
- `q r` = empty, fully tipped back.

| Pose | Rule | Used for |
|---|---|---|
| STANDING | Within 5° of `q s` | Live scale reading |
| Exactly upright | Within 2° of `q s` | Auto-read and click accept |
| TIPPED BACK | At least 15° back | Zeroing |
| DRIVE (motors) | Within 5° of `q r` (leaves beyond 10°) | Assist allowed; otherwise wheels held |

**State on 12 Nov:**

- `q s` re-captured on the trolley (upright reads 0°, tipped back about 42°).
- **`q r` is still the default and must be captured on the trolley.**
- Pose gating is `q 0` (left from the bench). Set `q 1` once both references are captured.

---

## 8. Load to assist payload ([`payload.c`](../src/payload.c))

| Source | How it is set | Assist |
|---|---|---|
| **MANUAL** | Power-up default `PAYLOAD_DEFAULT_KG` 15 kg; `W <kg>`; **after a triple-click zero** | **1.0 ×** load (`MANUAL_ASSIST_SCALE`) |
| **AUTO** | **Auto-read:** exactly upright, status OK/OVERLOAD, reading within ±`PAYLOAD_AUTO_STABLE_KG` 0.5 kg for `PAYLOAD_AUTO_STABLE_MS` 1.5 s (updates on a change > 0.5 kg) | **0.7 ×** load (`LOAD_ASSIST_SCALE`) |
| | **Click:** exactly upright, the most recent valid upright reading within `PAYLOAD_ACCEPT_MAX_AGE_MS` 3 s before the press | 0.7 × |

**Rules:**

- Frozen while not upright.
- **No upper limit**: readings over 150 kg are shown as OVERLOAD but can still be accepted.
- `W <kg>` **pauses the auto-read** until `W A`, a click or a restart, so the operator can always override a bad scale.
- The observer mass = 15 kg trolley + assist weight + wheel inertia. It is pushed only when it changes.

**Safety note:** a faulty bridge that settles at a wrong but steady value would be accepted. The 0.7× factor limits, but doesn't remove, over-assist. Check the LCD weight before pushing. `[VERIFY: consider an auto-read ceiling or a jump filter once the bridge is fixed.]`

---

## 9. Load detection and calibration

Full details are in [`LOAD_DETECTION.md`](../LOAD_DETECTION.md).

**Final calibration (locked):**

| Item | Value |
|---|---|
| View | `x 1` (middle strip, CH1) |
| Slope / offset | `s 220.3704 -2086.4` |
| Power-up zero | −3,028,324 |
| Gain | 64 |
| Fit | R² 0.989, ±1.7 kg (bench) |

- `LC_FINAL_CAL_LOCK 1` applies these at every boot and refuses `s/c/u/x/i`. Re-calibration procedure: `LOAD_DETECTION.md`.
- **Verified on the trolley (12 Nov):** with the plate seated and zeroed, 15 kg read about 15 kg and 30 kg read 29–33 kg.
- The **power-up zero no longer matches the reworked bridge**, so zero after every restart.

**Zeroing:**

- Triple-click (RAM, also → MANUAL).
- Auto-zero within ±3 kg / 3 s.
- `t` / `z` in the `LD` console.
- **All refused while the HX711 is at full scale.**

**Status codes:** 0 OK, 1 SETTLING, 2 NO_CAL, 3 RAILED, 4 FROZEN, 5 NO_DATA, 6 IMPLAUSIBLE (< −5 / > 225 kg), 7 OVERLOAD (> 150 kg), 8 HELD, 9 IMU_FAULT.

---

## 10. LCD ([`display.c`](../src/display.c), [`lcd_i2c.c`](../src/lcd_i2c.c))

```
UPRIGHT  30.2kg      line 1: UPRIGHT / DRIVE (motor pose) + live scale reading
AUTO  21.0kg OFF     line 2: AUTO / MAN + weight the assist uses + ON / OFF / ERR
```

- **Line 1 reading:**
    - `30.2kg` settled, `~` not settled or held, `!` over 150 kg.
    - Out of range shows **the actual value** (no IMPL).
    - `RAIL` / `FROZ` / `NODAT` / `NOCAL` / `IMU!` for faults.
- **Line 2:** in the `LD` console it reads `LD CONSOLE <status>`.
- **Bus:** own I2C3 (PA8/PC9), `LCD_I2C_SEPARATE_BUS 1`. The backpack provides 4.7 kΩ pull-ups to 5V. A bus clear and the STM32F4 BUSY-erratum workaround run at init.
- **`LCD` console command:**
    - Pull-up test per line (reports which wire is off).
    - Hardware and software scan.
    - Re-init without a reset. The LCD is otherwise only probed at boot.
- **Known risk:** with the I2C3 controller stuck BUSY, every LCD write waits about 25 ms, which can slow the main loop to about 10 Hz while the screen content changes. `[TODO: move the LCD to software I2C, or drop it after a failed write.]`

---

## 11. Console commands added or changed

The console runs over RTT. `H` lists them all.

| Command | Action |
|---|---|
| `PL` / `PL 1` / `PL 0` / `PL S` | Load and payload log: one line, 5 Hz stream on/off, toggle |
| `W` | Load in use (ACCEPTED/MANUAL), assist weight and factor, pose, live status |
| `W <kg>` / `W A` | Manual load (pauses the auto-read) / back to the accepted value |
| `LD` → `q s`, `q r`, `q 1`, `z`, `b`, `d` … | Pose capture, zero, diagnostics. `s/c/u/x/i` are refused while the lock is on. |
| `LCD` | LCD bus check and restart |
| `M`, `I`, `I2C`, `OD` / `OD C` | Pose / IMU / I2C1 scan / ODrive status |

---

## 12. Host tools

| File | Purpose |
|---|---|
| `tools/rtt_server.sh [--attach]`, `tools/rtt_monitor.sh` | RTT server (socket 9090) / RTT terminal |
| `host/live_load_log.py` | Plain-language live status (`--tech` for every value), logs to `data/strain_gauge/live_load_*.csv`. Translates accept, zero, back-to-manual and refusal messages. |
| `host/stm_dashboard.py`, `stm_calib_gui.py`, `stm_dual_test.py` | Calibration and drift GUIs (load the FINAL calibration on launch) |
| `host/middle_raw_check.py` | Raw HX711 / rail check |
| `host/stm_calibration_middle_FINAL.json` | The locked calibration record |

---

## 13. Verification status (12 Nov 2026)

| Item | Result |
|---|---|
| Full firmware builds, boots, all devices detected at boot | Verified |
| IMU, recovery without a reset | Verified; wiring and power marginal |
| Handle: hold arms, click accepts (AUTO), triple-click zeroes → MANUAL | Verified on the trolley (live log) |
| LCD on I2C3, layout, `LCD` command | Verified |
| Load: 15 kg / 30 kg after seating and zero | About 15 kg / 29–33 kg; drift a few kg/min |
| Final calibration lock, refused commands | Verified |
| Zero refused at full scale | Verified |
| Auto-read steady window, `q s` re-capture → AUTO | Verified |
| CAN TX | Verified (ACK error with the ODrives off) |
| Assist driving the wheels on the cart | **Not tested** (ODrives off during these tests) |
| `q r` on the trolley, `q 1` | **To do** |
| Ramp / outdoor | Not tested |

---

## 14. Technical debt and known issues

- **Hardware:** see 4.7. The bridge and the supply wiring are the main sources of bad readings.
- **Power-up zero mismatch:** consider a power-up auto-zero when tipped back and still (currently the recorded zero is forced).
- **Auto-read can accept a steady wrong value** (section 8).
- **LCD I2C3 BUSY stall risk** (section 10).
- **The wheels roll freely after a release-stop on a slope** (section 6). Decide whether to hold at 0 instead.
- **Stale comments:** the `board.h` header still mentions the Nucleo; the `W` help text describes the old latch rule.
- **Inherited issues:** no ODrive CAN watchdog, no cross-axis fault handling, minimal hard-fault handler, yaw unused, `pid.c` unused.

---

## 15. Where things are

| Item | Location (in `code/STM_RotaTruck/`) |
|---|---|
| Firmware | `src/`, `include/`, `platformio.ini` |
| Pins, options, calibration lock | `include/board.h` |
| Tuning | `include/assist_tuning.h` |
| Load detection | `LOAD_DETECTION.md` |
| Host tools / RTT scripts | `host/`, `tools/` |
| User manual / this document | `docs/` (`.md`, plus `.pdf` / `.docx` copies) |
| Previous handover pack (Nucleo) | `../../handover/` |
