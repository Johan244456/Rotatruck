# 03 User Manual: Operating the Power-Assisted RotaTruck

This manual is for the person operating the cart. It replaces the previous team's [`RotaTruck User Manual.pdf`](../docs/07_handover/RotaTruck%20User%20Manual.pdf) for the parts that changed: the **handle safety switch**, **automatic load detection** and the **new LCD screen**. Power connection and shutdown are unchanged.

> **Current limits of the prototype.** Validated on flat indoor floor only. Ramp, outdoor and kerb operation have not been tested yet. Automatic load detection has not yet been run on the truck; if the screen shows `M` after the payload, the weight you entered with the buttons is being used. Always keep a hand on the handle switch.

`[FILL IN: photos for each step, as in the previous manual (right-hand column). The steps that need new photos are marked with a camera note.]`

---

## 1. Know the controls

| Control | Where | What it does |
|---|---|---|
| **Handle switch** | On the handle `[FILL IN: exact position, photo]` | Must be held for the motors to help. Let go and the cart slows to a stop and holds. |
| **[A] button** (D8) | Front panel `[FILL IN: confirm location]` | Assist ON / OFF. The blue USER button (B1) on the Nucleo does the same. |
| **[+] button** (D10) | Front panel | Manual payload +5 kg |
| **[-] button** (D9) | Front panel | Manual payload -5 kg |
| **LD2 green LED** | On the Nucleo board | Load detection: solid = valid reading, slow blink = settling / not calibrated / held, fast blink = fault |
| PSU ON/OFF | Bench supply | Main power (24 V) |

**Poses.** STANDING (`STAND`) = the cart upright on its toe plate, the way it is loaded. DRIVE = the cart tipped back onto its wheels, the way it is pushed.

## 2. Reading the screen

```
+----------------+
|ASSIST:ON   22kg|   row 0: assist ON/OFF, payload in use
|DRIVE FWD  PRESS|   row 1: pose, what the motors are doing, handle switch
+----------------+
```

| Row 0, right | Meaning |
|---|---|
| `22kg` | Payload measured automatically (load detection) and latched |
| `15kgM` | Payload **entered by hand** with the +/- buttons (M = manual) |
| `ERR` | A motor controller reports a fault (see Troubleshooting) |

| Row 1 field | Values |
|---|---|
| Pose | `STAND` / `DRIVE` |
| Motors | `IDLE` (no help, or assist OFF), `FWD` (helping you push), `BACK` (helping you pull), `HOLD` (wheels held still: standing, tilting, or the 1 s hold after tipping back), `STOP` (assist ON but handle switch released: slowing to a stop or holding) |
| Handle switch | `PRESS` (held) / `RELSD` (released) |

In the load-detect maintenance console the second row reads `LD CONSOLE <status>`, and the assist cannot be used.

---

## 3. Operating procedure

| Step | Procedure | Screen should show |
|---|---|---|
| **1. Connect power** | Same as the previous manual: PSU off at the wall, PSU close to the base, red lead to red terminal, black to black, lock the terminals, plug into the wall, PSU set to **24 V, 5 A**. `[FILL IN: replace with the battery procedure if the Scope A battery is fitted.]` | |
| **2. Power on** | Turn the PSU on. Check the motor controllers flash blue and the PSU shows 24 V at about 0.1 to 0.2 A. The screen shows `RotaTruck / Starting...` for a moment. | `ASSIST:OFF  0kgM` / `STAND IDLE RELSD` |
| **3. Stand the cart** | Stand the cart upright (STANDING) on level floor. | `STAND IDLE RELSD` |
| **4. Load it** | Place the load on the toe plate, **stable and centred** (left-right and front-back). Nobody should stand on, lean on or touch the plate. | |
| **5. Let it settle** | Take your hands off the cart for **2 seconds**. The cart must stand on its own. The LD2 LED on the Nucleo goes solid when the load reading is valid. (Camera: LED photo) | |
| **6. Switch the assist ON while STANDING** | Press **[A]**. Doing this while the cart is standing is what lets it measure the load. The wheels are held still while standing. With your hand off the handle switch the screen shows `STOP`; that is normal. | `ASSIST:ON ...` / `STAND STOP RELSD` |
| **7. Grip and tip back** | Hold the handle **with the handle switch pressed** and tip the cart back onto its wheels (DRIVE). Hold it still for about 1.5 s: the wheels stay braked for 1 s so the cart cannot roll back (`HOLD`), and the system zeroes itself. The measured payload appears without `M`. (Camera: screen photo) | `ASSIST:ON   22kg` / `DRIVE HOLD PRESS`, then `DRIVE IDLE PRESS` |
| **7a. If the payload shows `M`** | No valid load reading was available (see Troubleshooting). Set the weight of the load with **[+] / [-]** (5 kg steps). The PSU weight is already accounted for. | `ASSIST:ON  45kgM` |
| **8. Push or pull** | Start the cart moving yourself. The motors help **once it is rolling**: push and it helps forward, pull and it helps backward, ease off and the help fades. It will not help you break away from a dead stop. The cart turns normally. Help fades out by fast walking pace. | `DRIVE FWD  PRESS` or `DRIVE BACK PRESS` |
| **9. To stop** | Slow the cart with your hands as you would a manual trolley, or **let go of the handle switch**: the cart slows to a stop over about 1 to 1.5 s and holds. Press the switch again to carry on. | `DRIVE STOP RELSD`, then `DRIVE IDLE PRESS` |
| **10. At the destination** | Bring the cart to rest, then stand it upright. The wheels are held. | `STAND HOLD PRESS` (or `STAND STOP RELSD` once you let go) |
| **11. Unload** | Remove the load safely. The next time you load and tip back (assist still ON), the new load is measured again. If you used the buttons, set the weight back to 0 kg or to the new load. | |
| **12. Switch off** | Press **[A]** to turn the assist OFF. Then, as in the previous manual: PSU off with the ON/OFF button, unplug from the wall, disconnect red and black leads, store the PSU and cart safely. | `ASSIST:OFF ...` |

### Things that change the payload

- Pressing **[+] / [-]** at any time switches to the manual value (`M`), starting from the value in use. So a press during a push nudges the measured value by 5 kg.
- Switching the assist ON while the cart is already tipped back (DRIVE) uses the manual value. To use load detection, switch it on while standing (step 6).
- The manual value starts at 0 kg each time the cart is powered on. It is not saved.

### Autonomous mode (engineers only)

From the serial console, `R A` makes the assist switch itself on each time the cart is tipped back, and `R M` returns to the normal (manual) mode, which is the power-up default. In autonomous mode the assist is armed in DRIVE, so the **load is not measured** and the manual payload is used. Set it with the buttons.

---

## 4. Safety rules

1. Keep a hand on the handle switch whenever the cart is moving. Letting go is the way to stop the motors.
2. Do not exceed **150 kg** (RotaTruck LITE rating). The assist is sized for 60 to 90 kg.
3. Use on **flat, level, dry floor** until ramp and outdoor testing has been completed and recorded in [04 Status](04_Status_and_Limitations.md).
4. Never let anyone stand on the toe plate or ride the cart.
5. Tip the cart slowly. Fast tilting holds the wheels (by design) and very fast tilting has tripped the bench supply in the past.
6. Do not open the load-detect console (`LD`) or change settings while the cart is loaded and someone is operating it. The assist switches off when the console opens.
7. If anything behaves unexpectedly: let go of the handle switch, stand the cart upright, press [A] to switch the assist off. `[FILL IN: physical emergency stop / power isolation if one is fitted by Scope A.]`

---

## 5. Troubleshooting

| What you see | Likely cause | What to do |
|---|---|---|
| Random characters on the screen | Loose LCD wiring or I2C glitch | Check the connections, then press the black RESET button on the Nucleo |
| `ERR` on the screen, motor controller LEDs red | ODrive fault | Stand the cart upright and tip it back again (this clears the faults). If it returns, check the supply and wiring, or use `OD` / `OD C` on the console |
| Pushing gives no help | Assist OFF, handle switch not pressed, still in the 1 s hold after tipping back, or cart not yet rolling | Check `ASSIST:ON` and `PRESS`, wait 1.5 s after tipping back, then push to get it rolling |
| `STOP` on the screen | Handle switch released (or a broken switch wire, which reads as released) | Press the switch. If `RELSD` stays with the switch held, check the switch wiring to A0 |
| Payload shows `M` after tipping back | Assist was switched on in DRIVE, the cart was touched while standing, load not calibrated, or a sensor fault | Set the weight with the buttons. For load detection: stand the cart, hands off 2 s, assist ON while standing, tip back |
| Measured payload clearly wrong | Load not centred, plate touched, calibration out of date (for example after the battery or handle was moved) | Re-centre and repeat steps 3 to 7. If still wrong, the plate needs recalibration (see [02 Technical Handover](02_Technical_Handover.md), section 6.4) |
| LD2 LED blinking fast | Load sensor fault (unplugged HX711, frozen or railed reading) | Use the buttons; report it for maintenance |
| Cart creeps, twitches or pushes on its own | Tuning issue, or a slope estimate on uneven floor | Let go of the handle switch, stand the cart up, switch the assist off. Report it with the console log if possible |
| Pose does not change to DRIVE | IMU not detected, or the cart not tipped back far enough | Tip back further. If the console shows `IMU: none detected`, check the IMU wiring |
| PSU cuts out when tilting | Wheels drawing current during a fast tilt | Tilt slowly |
| `[A]` does nothing, console says `refused - load-detect console open` | The `LD` maintenance console is open | Close it with `LD` on the console |
