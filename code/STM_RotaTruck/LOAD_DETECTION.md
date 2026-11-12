# Load detection in the RotaTruck firmware (STM32F407 DISC1)

The trolley weighs its load with strain gauges on the toe plate, read by an HX711. The load is read while the trolley stands upright, frozen when it tips back, and handed to the power assist. This note covers how it works, how to wire, zero and calibrate it, and what the faults look like.

**Status (12 Nov 2026):**

- Runs in the full firmware on the DISC1, on the trolley.
- Only the **middle strip** (2 gauges, HX711 CH1) is fitted and used.
- The final bench calibration is **locked in the firmware**.
- With the plate seated and zeroed, 15 kg read about 15 kg and 30 kg read 29–33 kg.
- Open issues:
    - The bridge drifts by a few kg per minute.
    - The bridge shifts on the first load after a rest.
    - Several push-on wires come loose (see [Known limits](#known-limits)).

## How it works in one table

| Step | What happens |
|---|---|
| **Weigh** | HX711 at 10 Hz → median → zero → final calibration → kg. The trolley must be **exactly upright** (within 2° of the captured upright pose `q s`), still, and not touched for about 1 s. |
| **AUTO read** | Upright, and the reading holds within ±0.5 kg for 1.5 s: that reading becomes the load (**AUTO**). Standing up again re-reads the total. |
| **1 click** | Upright: the most recent valid upright reading (up to 3 s before the press) becomes the load (**AUTO**). |
| **Freeze** | Tipped back or driving: the load stays at its last value |
| **Assist** | AUTO load × **0.7** (`LOAD_ASSIST_SCALE`). MANUAL load × **1.0** (`MANUAL_ASSIST_SCALE`). |
| **MANUAL** | At power-up: 15 kg (`PAYLOAD_DEFAULT_KG`). Also after `W <kg>` and after a triple-click zero. |
| **Zero** | Triple-click while tipped back and empty. Small drift is also corrected automatically within ±3 kg. |

## Wiring (STM32F407G-DISC1)

| Signal | DISC1 pin | Notes |
|---|---|---|
| HX711 #1 (CH1, middle strip) DT / SCK | **PE11 / PE12** | Fitted |
| HX711 #2 (CH2, left + right strips) DT / SCK | PE13 / PE14 | Not fitted (the boot log says `CH2 MISSING`) |
| HX711 VCC / GND | **3V** / GND | The DISC1 3V rail is about 3.0 V, so HX711 AVDD is about 2.9 V |
| HX711 E+ / E− / A+ / A− | Middle-strip half bridge | 2 gauges + 2 × 350 Ω completion resistors. **Solder all of them.** |
| Status LED | LD4 green (PD12) | Solid = valid; slow blink = settling, held or no calibration; fast blink = fault |
| IMU ISM330DHCX | PB8 SCL / PB9 SDA, 3V | Pose (upright / tipped back) |
| LCD (PCF8574 backpack) | **PA8 SCL / PC9 SDA, 5V** | Its own I2C bus (I2C3), separate from the IMU |

The IMU and both HX711s share the board's **3V** rail. A poor 3V or GND contact makes them fail together: the IMU "reads ~0 g", and the HX711 returns −1 or NO_DATA.

## Bridge health (read this first)

At HX711 gain 64 and AVDD 2.9 V:

| Quantity | Value |
|---|---|
| HX711 input range | **±22.66 mV** (±8,388,608 counts) |
| 1 kg on the plate | about **2,160 counts** ≈ **6 µV** |
| Resistance change in one 350 Ω arm that looks like 1 kg | about 0.003 Ω |
| 1 Ω of contact resistance in one arm | **about 330 kg** |

So the bridge must be **balanced** (A+ to A− at rest well inside ±22.66 mV, ideally under ±5 mV), and every joint must be solid. A loose joint changing by an ohm or two looks like hundreds of kilograms.

| Symptom | Raw data | Cause |
|---|---|---|
| `RAIL`, about 2,480 kg, raw stuck at −8,388,608 | A+/A− beyond ±22.66 mV | Bridge badly unbalanced (20–26 mV was measured on 12 Nov), or an arm open |
| Hundreds to thousands of kg, jumping, sometimes RAIL | Raw jumping by 10⁵–10⁶ counts | Intermittent joint or wire (E±, A±, a gauge or resistor leg) |
| Empty plate creeps smoothly by kg/s, never returns | Raw ramps smoothly | An input nearly open (floating), or creep in the plate or glue |
| A large step on the **first** load after a rest, then correct steps | One step of 100–400 kg, later loads correct | The plate seats on its mounting. Seat it (load once, unload) before zeroing. |
| Huge offset straight after a restart | Normal noise, wrong level | The power-up zero no longer matches the bridge. Triple-click to zero. |
| −1 samples, NO_DATA | — | HX711 power or DT line dropping. Check the 3V/GND rail and DT/SCK. |

Diagnose from the console: `LD` opens the CSV stream (column `raw`), `b` gives the bridge report, `d` the diagnostic.

## Calibration (locked)

The board always uses the **final bench calibration**, recorded in [`host/stm_calibration_middle_FINAL.json`](host/stm_calibration_middle_FINAL.json):

| Item | Value |
|---|---|
| Strain gauges used | `x 1`: CH1, middle strip, 2 gauges |
| Slope / offset | `s 220.3704 -2086.4` (counts/N, counts) |
| Power-up zero | `z -3028324 0` |
| HX711 gain | 64 |
| Fit | R² 0.989, ±1.7 kg over 0–32 kg (bench) |

`board.h` → `LC_FINAL_CAL_LOCK 1` with `LC_FINAL_VIEW / _COUNTS_PER_N / _OFFSET_COUNTS / _ZERO_CH1 / _SIGN_CH1`:

- These values are applied at **every power-up**, whatever is in the flash settings.
- In the `LD` console, `s`, `c`, `u`, `x` and `i` are **refused**. The exception is `s 220.3704 -2086.4` / `x 1` themselves (what the dashboard sends), which are accepted as a no-op.
- **To recalibrate** (for example on the trolley, after the bridge is soldered): set `LC_FINAL_CAL_LOCK 0`, flash, and follow the [routine](#calibration-routine). Then put the new values into `LC_FINAL_*` and the FINAL JSON, set the lock back to 1, and flash.

The calibration converts the **change** in counts to kg. Whether the readings are right depends on the zero and on a stable bridge, not on the calibration.

## Zeroing

| Zero | When | Kept |
|---|---|---|
| **Power-up zero** | Every restart, including every USB drop: the recorded −3,028,324 | Always this value. **It no longer matches the bridge, so zero after every restart.** |
| **Triple-click** | Tipped back (IMU required), plate empty, hands off. Uses the samples from just before the first click. | Until restart. **Also sets the load back to MANUAL.** |
| **Auto-zero** | The reading stays within ±3 kg for 3 s and is stable (with pose gating off, `q 0`, in any pose) | Until restart. Does **not** change the load source. |
| **Console `t` / `z`** | In the `LD` console, tipped back and empty | `z` saves to flash (the lock still uses the recorded zero at power-up) |

**Zeroing is refused while the HX711 is stuck at its limit** (`REFUSED: HX711 at full scale`), so a disconnected bridge can't be hidden by zeroing onto the limit.

**Routine before a load test:**

1. Restart if needed, then **seat the plate**: put a load on once and take it off.
2. **Tip back, empty, hands off: triple-click.** It should read about 1 kg (the 0.97 kg calibration offset).
3. Load, stand it **exactly upright**, hands off for about 1.5 s, and it switches to AUTO.
4. Drift adds up over minutes, so zero again before each test.

## Operator workflow (handle switch)

| Handle | When | Result |
|---|---|---|
| **1 click** (< 0.35 s) | Trolley exactly upright | Takes the most recent valid upright reading as the load: **AUTO**, assist 0.7× |
| **3 quick clicks** | Tipped back, plate empty | Zero the scale, and the load goes back to **MANUAL** (15 kg / last `W` value, assist 1.0×) |
| **hold** (≥ 0.4 s) | Any time | Assist on (when the ODrives answer). It drives only when **fully** tipped back. Release → ramp to a stop → assist off after 0.5 s (wheels then roll freely). |

Refusals are printed (and translated by `host/live_load_log.py`), for example `not exactly at the standing angle (6.0 deg ...)`, `no settled reading`, `no IMU`.

There is no upper weight limit. A reading over 150 kg is shown as OVERLOAD, and the click and the auto-read still accept it.

## Pose references (IMU)

Upright, tipped back and "fully tipped back for the assist" are all measured against two captured poses:

- **`q s`**: upright, at least 15 kg on, hands off.
- **`q r`**: empty, fully tipped back at the normal pushing angle.

Both are saved in flash. **The IMU board must be fixed rigidly.** If it moves, every angle is wrong.

- On 12 Nov the IMU moved while its wires were re-seated. Upright then read 81° from the old reference, so nothing could reach AUTO.
- `q s` was re-captured on the trolley (upright now reads 0°).
- **`q r` is still the old default and must be captured on the trolley.**
- After capturing both, turn pose gating back on with **`q 1`**. It was left at `q 0` from the bench calibration.

```
LD      q s  (upright, ≥15 kg, hands off)      q r  (empty, fully tipped back)      q 1      LD
```

## Status codes and the LCD

Status codes (CSV, `W`, `PL`): 0 OK, 1 SETTLING, 2 NO_CAL, 3 RAILED, 4 FROZEN, 5 NO_DATA, 6 IMPLAUSIBLE (< −5 or > 225 kg), 7 OVERLOAD (> 150 kg), 8 HELD, 9 IMU_FAULT.

**LCD line 1:** `UPRIGHT` / `DRIVE` + scale reading:

- `30.2kg` settled
- `~30.2kg` not settled, or not weighing
- `!160kg` over 150 kg
- **Out of range shows the actual value** (`-12.1kg`, `2480kg`), not IMPL
- `RAIL` / `FROZ` / `NODAT` / `NOCAL` / `IMU!` faults

**LCD line 2:** `AUTO` or `MAN` + the weight the assist uses (AUTO 0.7×, MAN 1.0×) + `ON` / `OFF` / `ERR`. In the `LD` console, line 2 reads `LD CONSOLE <status>`.

## Serial commands (RTT console)

| Command | Action |
|---|---|
| `W` | Load in use (AUTO = accepted from the scale, or MANUAL), assist weight and factor, pose, live status |
| `W <kg>` | Manual load (assist 1.0×). **Pauses the auto-read until `W A`, a click or a restart.** |
| `W A` | Back to the last accepted (AUTO) value, auto-read resumes |
| `PL` / `PL 1` / `PL 0` | Load and payload log (5 Hz stream) used by `host/live_load_log.py` |
| `LD` | Load-detect console (calibration and diagnostics; the assist is off while it's open) |
| `LCD` | Scan the LCD bus, report a missing pull-up (wire off) and restart the display |

**Load-detect console:**

- Commands `d b v x t z c s u q g i n f k m h a p r` (see the bench rig's `RUN_GUIDE.md`; calibration-changing ones are refused while the lock is on).
- 10 Hz CSV: `t_ms,raw,offset_raw,hx_mV,bridge_uV,microstrain,newtons,valid,status,assist_kg,assist_pct,pose,tilt_deg`.

## Live view and host tools

The DISC1 has no serial port to the PC: the console runs over **RTT** through the ST-Link.

```
tools/rtt_server.sh --attach          # terminal 1 (or the PlatformIO serial monitor)
python3 host/live_load_log.py         # terminal 2: plain-language status (--tech: every value)
```

Logs go to `data/strain_gauge/live_load_*.csv`.

**Calibration GUIs** (`host/stm_dashboard.py`, `host/stm_calib_gui.py`, `host/stm_dual_test.py`):

- Connect to `socket://localhost:9090` and send `LD 1` / `LD 0`.
- The dashboard loads the FINAL calibration on launch (`--no-final` to skip it).
- Close them before uploading.

**Every USB drop restarts the board and kills the RTT link.** Restart `rtt_server.sh` (or the monitor), and re-zero.

## Calibration routine

Only needed with `LC_FINAL_CAL_LOCK 0`. The trolley is back-heavy: empty, it rests tipped back. So:

- the **zero is taken tipped back, empty, hands off**,
- standing readings have an **offset** (`s <slope> <offset>`, `N = (reading − offset) / slope`).

1. Solder the bridge. Check that A+ to A− at rest is under ±5 mV and that the empty reading holds steady for a few minutes.
2. Capture `q s` and `q r` on the trolley, then `q 1`.
3. Seat the plate (load once, unload).
4. In `host/stm_dashboard.py` → **Capture zero + load**: zero (tipped back, empty) → stand it, 15 kg, hands off about 10 s → **Continue** with the **total** mass. Repeat for 30 kg, 40 kg …, each with a fresh zero. Do 2–3 rounds.
5. Check: R² ≥ 0.999, no curve in the residuals, a sensible offset. Then **APPLY**.
6. Copy the result into `LC_FINAL_*` and the FINAL JSON, and set the lock back to 1.

Rules:

- Every calibration load must stand the trolley by itself (15 kg minimum).
- Never capture while the handle is held.
- Recalibrate after moving the battery, motors or handle.
- Don't use `c <N>` (it forces offset 0).

## Known limits

- **Bridge stability:**
    - Push-on and unsoldered connections cause offsets, jumps and RAIL (see [Bridge health](#bridge-health-read-this-first)).
    - The empty plate drifts by a few kg per minute.
    - The plate shifts on the first load after a rest.
    - Fix: solder everything, add strain relief, and fix the plate mounting.
- **The power-up zero doesn't match the bridge** since it was rewired, so zero after every restart.
- **USB drops** (laptop-powered board) restart the board, which loses the zero and the accepted load.
- **The 3V/GND rail** shared by the IMU and the HX711 is marginal: the IMU resets and reads 0 g, the HX711 returns −1.
- **IMU and LCD wires** have come loose repeatedly, and the IMU board moved, so mount and secure them.
- **`q r` is the old default** and needs capturing on the trolley. Pose gating is off (`q 0`).
- **Bench calibration only** (R² 0.989, ±1.7 kg). The middle strip only; the side strips (CH2) aren't fitted.

## Files

| File | Role |
|---|---|
| `src/loadcell.c`, `include/loadcell.h` | Load detection: HX711, filters, zero (rail guard), final-calibration lock, pose, status codes, `LD` console |
| `src/payload.c`, `include/payload.h` | Load in use: AUTO read (steady window), click accept, manual lock, zero → MANUAL, AUTO 0.7× / MAN 1.0× |
| `src/deadman.c` | Handle gestures (click, triple-click, hold) |
| `src/display.c`, `src/lcd_i2c.c` | LCD layout, own I2C3 bus, `LCD` diagnostics |
| `src/command.c` | `W`, `PL`, `LD`, `LCD` commands |
| `include/board.h` | Pins, `LC_FINAL_*`, `PAYLOAD_*`, `LOAD_ASSIST_SCALE`, `MANUAL_ASSIST_SCALE`, LCD bus |
| `host/` | `live_load_log.py`, calibration GUIs, `stm_calibration_middle_FINAL.json` |
