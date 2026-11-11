# Load detection in the RotaTruck firmware

Automatic payload detection (spec FR5) from the P102 bench rig (`Code_STM/P102_LoadDetect_Main`, firmware v4.2) now runs inside the RotaTruck motor firmware. The detected load is latched into the assist when the truck is tipped back, and the +5 / -5 kg buttons stay as a manual fallback.

**Status (10 Nov 2026):** builds with no warnings (`-Wall -Wextra`), about 89 KB flash. **It has not been run on the truck yet.** Everything below the "Bring-up" heading is the plan, not verified results.

## What changed

| Area | Before | Now |
|---|---|---|
| Payload | Entered with the +5 / -5 kg buttons (PB6, PC7) or `W <kg>` | Measured by 2 x HX711 strain-gauge bridges. The buttons stay as the manual fallback |
| Assist payload | Whatever was entered | **Latched** at STANDING -> tipped back: 0.9 x the last valid standing reading, if the assist was switched on at STANDING. Otherwise the manual (button) value |
| LCD row 0 | `ASSIST:ON   15kg` (entered value) | `ASSIST:ON   22kg` (latched value) or `ASSIST:ON  15kgM` (manual) |
| LCD row 1 | `DRIVE      FWD` | `DRIVE FWD  HELD` = pose, assist state, load status |
| IMU | ISM330DHCX with an MPU6050 fallback | ISM330DHCX only (MPU6050 removed); also tries 0x6A, clears a stuck I2C bus at power-up, reconfigures itself if it reads ~0 g |
| Serial | Motor console only | Motor console plus a load-detect console (`LD`) |

The bench rig's signal chain, filters, gauge views, rest zero, status codes, flash settings layout (v6) and command letters are unchanged.

## Wiring (STM32F407G-DISC1)

| Signal | DISC1 pin |
|---|---|
| HX711 #1 (CH1, middle strip) DT | PE11 |
| HX711 #1 SCK | PE12 |
| HX711 #2 (CH2, left + right strips) DT | PE13 |
| HX711 #2 SCK | PE14 |
| Both HX711 VCC | **3V** (the DISC1 rail is 3.0 V; not 5V) |
| Both HX711 GND | GND |
| Status LED | green LD4 / PD12 (on the board) |
| IMU ISM330DHCX (3V only) | PB9 SDA / PB8 SCL, shared with the LCD |

Both HX711s are on the PE header (PE11-PE14) rather than the Nucleo's PA10/PB3 and PB5/PB4 (PA10 is the DISC1's micro-USB ID line, PB3 the ST-Link SWO line). The bridge wiring is the same as on the bench rig.

The Nucleo-G491RE pins (D2-D5, LD2/PA5) are in git history (commit eb04ba3). The payload buttons and assist button still use the Nucleo pins in `board.h` and still need moving for the DISC1.

## Operator workflow (F407, handle switch)

One pose definition drives everything (board.h `POSE_FROM_LOAD_REFS`): the captured standing (`q s`) and tipped-back (`q r`) poses decide where the load is accepted **and** what the motors do. Upright = wheels held; tipped back = assist drives; IMU lost = wheels held.

| Handle | When | Result |
|---|---|---|
| **1 click** | trolley **exactly upright** (within 2 deg of `q s`), hands off the load ~1 s first | the most recent settled reading is **accepted and frozen** as the load; the assist is set for **0.7 x** that load (`LOAD_ASSIST_SCALE`) |
| **3 quick clicks** | **tipped back**, plate empty | scale **zeroed** (RAM; `LD`, `z`, `LD` saves it) |
| **hold** (>= 0.4 s) | any time | **assist on**; upright = wheels held, tip back = assist drives; **release = ramp to a stop, then off** |

The load the assist uses is 15 kg at power-up (`PAYLOAD_DEFAULT_KG`, assist 10.5 kg) until a load is accepted, or `W <kg>` sets one. While resting tipped back, the scale re-zeroes itself if it stays within +/-3 kg of zero for 3 s.

Live view for anyone running a test: `tools/rtt_server.sh --attach` in one terminal, `python3 host/live_load_log.py` in another (plain language; `--tech` for every value). Data is logged to `data/strain_gauge/live_load_*.csv`.

## How the payload reaches the assist

**Measuring (loadcell.c)**

1. The HX711s convert at 10 Hz. Each conversion is read when it is ready (interrupts off for about 60 us), so the main loop never waits.
2. The IMU pose decides when a reading counts, as on the bench rig:
   - **Standing on the toe, hands off for 1 s:** live reading. When it is settled, the status is OK.
   - **Tipped back or driving:** status HELD.
   - **Tipped back, empty and still:** the zero follows the reading (rest zero).
3. The value offered to the assist is 0.9 x measured load, capped at the rated payload (150 kg), and 0 on any fault. Under-assisting is the safe side.

**Latching into the assist (payload.c)**

1. **Switch the assist ON (D8 or B1) while the truck is STANDING.** This arms the latch. The console prints `PAYLOAD: assist ON at STANDING - load will be latched when you tip back.`
2. While it stands, the firmware remembers the **last valid live reading** (status OK or OVERLOAD).
3. **Tip it back into DRIVE.** That final standing reading becomes the assist payload and stays fixed for the whole push: `PAYLOAD: latched 22.5 kg for the assist (load detection measured 25.0 kg).`
4. Standing it up and tipping it back again in the same session (assist still on) re-latches from the new standing period.

**Manual fallback (buttons)**

| Situation | Assist payload |
|---|---|
| Assist switched ON outside STANDING (in DRIVE) | Manual value, no latch |
| No valid reading while standing (NO_CAL, NO_DATA, IMU_FAULT, never settled) | Manual value, with a message giving the load status |
| A +/- button pressed (or `W <kg>`) | Manual, starting from the value in use, so a press nudges a latched value by 5 kg |
| Next armed STANDING -> DRIVE | Load detection latches again |

The manual value starts at `PAYLOAD_DEFAULT_KG` (board.h, 15 kg for the DISC1 bring-up; originally 0) at power-up and is not saved. The payload sets the observer's mass and friction model, and is only pushed when it changes.

Status codes (CSV and `W`): 0 OK, 1 SETTLING, 2 NO_CAL, 3 RAILED, 4 FROZEN, 5 NO_DATA, 6 IMPLAUSIBLE, 7 OVERLOAD, 8 HELD, 9 IMU_FAULT.

LCD short forms: `OK`, `SETL`, `NOCAL`, `RAIL`, `FROZ`, `NODAT`, `IMPL`, `OVER`, `HELD`, `IMU!`.

## Serial commands

The motor console (115200, ST-LINK COM port) gains:

| Command | Action |
|---|---|
| `W` | Payload in use (LATCHED or MANUAL), last latch, manual value, whether the latch is armed, load status |
| `W <kg>` | Set the manual payload (same as the buttons). Not saved |
| `W A` | Back to this session's latched value |
| `LD` | Enter or leave the load-detect console (`LD 1` / `LD 0` force it) |

**The load-detect console** is a maintenance mode for calibration:

- All the bench-rig commands work exactly as before: `d b v x t z c s u q g i n f k m h a p r`. See the bench rig's `RUN_GUIDE.md`, section 8.
- The 10 Hz CSV streams: `t_ms,raw,offset_raw,hx_mV,bridge_uV,microstrain,newtons,valid,status,assist_kg,assist_pct,pose,tilt_deg`.
- **The assist is switched off on entry and cannot be armed while the console is open.** Calibration commands such as `z`, `c` and `d` block the loop for several seconds.
- Outside the console, load detection keeps running and only prints faults and recovery (for example `# LOAD NO_DATA: ...`).

## Host GUIs

Copies of the bench-rig GUIs are in `host/`. They now send `LD 1` on connect and `LD 0` on close.

On the DISC1 the ST-Link's USB COM port is not wired to the F407, so the GUIs talk over RTT through the debugger instead, on `socket://localhost:9090` (listed first in the GUIs' port box). One-time setup on Linux: `sudo apt install python3-serial`.

1. Upload the firmware (`pio run -e loadcell_test -t upload` for bench testing) and close the PlatformIO serial monitor.
2. Terminal 1, leave it running: `tools/rtt_server.sh`
3. Terminal 2:

```
cd host
python3 stm_dashboard.py      # calibration dashboard
python3 stm_dual_test.py      # drift / stability test
python3 stm_dashboard.py socket://localhost:9090 test_01   # straight in
```

4. Close the GUI, then Ctrl-C in terminal 1 (frees the ST-Link for uploads).

Data is written to `data/strain_gauge/` in this project. The earlier bench data stays in `Code_STM/P102_LoadDetect_Main/data/`.

The board **keeps** its saved slope and offset when the dashboard opens (the bench-rig version cleared them with `u` on every launch; on the truck that would leave the assist uncalibrated). To clear every view on purpose, type `u` in the dashboard's command box.

## Calibration routine

The trolley is heavy at the back. Empty, it cannot stand on its own: it rests tipped back, and only stands on its toe once the payload outweighs the back, or when someone holds it upright. Two things follow:

- **The zero is always taken tipped back, empty and hands off.** That is the only empty state the trolley has.
- **Standing readings do not pass through that zero.** The toe only starts carrying load above a threshold, so the standing line has an **offset** (on the bench rig, middle strip: about 3.1 microstrain per kg and about -39 microstrain offset, about 12.6 kg). With slope only, 15 kg would read about 2.4 kg and 30 kg about 17 kg. With slope **and** offset, both read correctly. The firmware stores both per view (`s <counts/N> <offset>`) and uses `N = (reading - offset) / slope`.

Steps, in `host/stm_dashboard.py` with **Capture zero + load** (paired capture):

1. **Tip it back, empty, hands off.** Click **Capture zero + load**: it records the zero. It refuses unless the pose is TIPPED BACK and still.
2. **Stand it up, place 15 kg, let go, wait about 10 s**, then click **Continue: load placed**, with the **total** mass on the plate entered. It refuses unless the pose is STANDING.
3. Do the same for **30 kg, 40 kg, ...**, each starting with a fresh tipped-back zero, always entering the total.
4. **Repeat steps 1 to 3 two or three times.** The spread between rounds shows how well the rest zero repeats.
5. Check the fit: R^2 about 0.999 or better, no curve in the residuals, and a "zero reads as" offset in kg that makes sense for the trolley's balance.
6. Click **APPLY**. This sends `s <slope> <offset>` and saves both in flash.

Rules:

- **Every calibration load must stand the trolley by itself.** 15 kg is the lowest step. 2.5 kg does not stand it, so don't use it.
- **Never capture while someone holds the trolley upright.** A hand on the handle adds force the plate does not see. The firmware also treats any movement in the last second as "being handled" (status SETTLING), and the dashboard refuses a standing capture in the wrong pose.
- Capture the IMU poses first: **`q s` standing with at least 15 kg on, hands off**, and **`q r` empty, resting tipped back**.
- In use it works the same way: the zero follows by itself while the trolley rests tipped back (rest zero), and readings count when it stands.
- **The offset is a property of the trolley's own weight and balance.** It stays valid until something moves. **Recalibrate if the battery, motors or handle are moved, added or removed.**
- `c <N>` (one-point calibration) sets offset 0. Don't use it for this trolley.

## Bring-up on the truck (to do)

1. Wire the HX711s (table above). Measure 3V3 to GND for a short before powering up.
2. Flash the firmware (`loadcell_test` for bench work: load detection + IMU only, LD console on from boot, no CAN or motors). The boot log should show `LOAD: HX711 CH1 ok, CH2 ok | ...`. On a fresh board it also shows `view not calibrated -> assist payload 0 kg (NO_CAL)`.
3. `LD`, then `b` and `d`: check that both channels respond.
4. Capture the IMU poses for the load plate: standing with at least 15 kg on and hands off, `q s`; tipped back and empty, `q r`. The defaults were measured on the truck (standing about -43.6 deg, tipped back about -1.2 deg pitch), but capturing them is more accurate.
5. Calibrate with `host/stm_dashboard.py`: save zero (tipped back, empty), check signs, then follow the **Calibration routine** above for 2 SG and 4 SG, balance g, and finally All 6 (the view the assist uses).
6. `LD` to leave. Stand it loaded, switch the assist ON, wait for `W` to show a valid reading, tip back: the console prints `PAYLOAD: latched ... kg` and the LCD shows it without `M`.
7. Safety checks (FR1 to FR3, keep the logs):

| Check | Expected |
|---|---|
| Unplug one HX711 DT wire, then arm at STANDING and tip back | `LOAD NO_DATA` within 0.5 s, LCD `NODAT`; on tipping back `no valid load reading` and the manual value is used |
| Switch the assist on in DRIVE | `assist ON outside STANDING`, manual value, no latch |
| Press + / - during a push | Payload changes by 5 kg, LCD shows `M` |
| Unplug the IMU | About 0.1 s later: `LOAD: IMU lost`, LCD `IMU!` |
| Open `LD` with the assist on | Assist switches off; D8 / B1 / `AS 1` are refused until `LD` is closed |
| Power-cycle | Zero, scales, g, signs and pose references are kept (`LD`, `b`) |

## Known limits

- **Not run on the F407 yet.** HX711 timing uses the DWT cycle counter, so it should carry over at 168 MHz. Settings are saved in flash sector 11 (0x080E0000): a save stalls the CPU for about 1-2 s (it was about 20 ms on the G4's 2 KB page). It only happens from console commands, while the assist is off.
- There is no backup IMU. If the ISM330DHCX fails, the status goes to IMU_FAULT and the payload drops to 0 kg. The pose machine also loses its pitch reading.
- The LCD backpack shares the I2C bus. If its SDA / SCL pull-ups go to 5 V, they over-drive the 3.3 V-only ISM330DHCX. Measure SDA and SCL to GND with power on: they should read about 3.3 V.
- Front-to-back load placement error and toe friction (bench-rig `CONTEXT.md`) are unchanged.

## Files

| File | Role |
|---|---|
| `src/loadcell.c`, `include/loadcell.h` | Load-detection subsystem (bench-rig v4.2 port, changes marked `[RT]`) |
| `src/imu.c`, `src/ism330.c` | IMU layer (ISM330DHCX only): `imuGetLatest()`, 0x6A fallback, bus clear, cart-frame signs in `board.h` |
| `src/payload.c`, `include/payload.h` | Latch on STANDING -> DRIVE (if armed at STANDING), manual fallback, sends the payload to the observer |
| `src/main.c` | `loadcellInit()` / `loadcellUpdate()`, `payloadInit()` / `payloadUpdate()` |
| `src/command.c` | `LD` console routing, `W` / `W <kg>` / `W A`, junk-byte filter |
| `src/admittance.c` | Assist refused while the console is open |
| `src/display.c` | New LCD layout |
| `src/mode_input.c`, `include/board.h` | Payload buttons now set the manual payload in payload.c, HX711 pins added |
| `host/` | Bench-rig GUIs with the `LD` handshake; the dashboard no longer clears the calibration on launch |
