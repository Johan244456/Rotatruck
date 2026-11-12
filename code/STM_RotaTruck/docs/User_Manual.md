# User Manual: Operating the Power-Assisted RotaTruck (STM32F407 controller)

This manual is for the person operating the cart. It replaces [`handover/03_User_Manual.md`](../../../handover/03_User_Manual.md) for the cart fitted with the **STM32F407 Discovery (DISC1) controller board**.

**Everything is done with the handle switch.** The cart's status is shown on the **16x2 LCD**. For more detail, a laptop can show a plain-language live status.

> **Current limits of the prototype (12 Nov 2026):**
>
> - The load scale reads within a few kg once it has been **zeroed just before use**, but it drifts over minutes, so zero again before each load (section 3).
> - After any restart the scale must be zeroed again.
> - Use the cart on flat, dry, indoor floor only.
> - Always keep a hand on the handle switch while the cart is moving.

`[FILL IN: photos for each step (handle switch, LCD, cart upright / fully tipped back), as in the previous manual.]`

---

## 1. Know the controls

| Control | Where | What it does |
|---|---|---|
| **Handle switch** | On the handle `[FILL IN: exact position, photo]` | The only control (see the gestures below) |
| **LCD screen** | `[FILL IN: position]` | Position, scale reading, the weight the assist is set for, assist on/off (section 2) |
| **Green LED (LD4)** | On the controller board | Scale: **solid** = good reading; **slow blink** = settling or held; **fast blink** = scale fault |
| **Black RESET button** | On the controller board | Restarts the controller. The assist goes off and the scale must be zeroed again. |
| Laptop live status (optional) | Laptop on the board's USB | Same information in words, plus every event |
| Main power | ODrive battery / bench supply `[FILL IN: battery or PSU procedure]` | Powers the two wheel motors |

**The two positions of the cart:**

- **UPRIGHT:** standing on the toe plate. This is how it is loaded, and where it weighs the load. The wheels are held.
- **FULLY TIPPED BACK (`DRIVE`):** resting back on its wheels at the normal pushing angle. **This is the only position where the motors help.** Partly tipped = wheels held.

**Handle switch gestures:**

| Gesture | How | When | Result |
|---|---|---|---|
| **Hold** | Keep it pressed (longer than about 0.4 s) | Any time | Assist ON. It helps only when fully tipped back and rolling. |
| **Let go** | Release | Assist on | The cart slows to a stop in about 1–1.5 s, then the assist switches off and the wheels roll freely |
| **1 click** | One quick press | Cart **exactly upright** | Accepts the weight now on the plate (**AUTO**) |
| **3 clicks** | Three quick presses | Cart **tipped back**, plate **empty** | Zeroes the scale. The assist weight goes back to **MAN** 15 kg. |

Quick clicks never switch the motors on. Only a hold does.

---

## 2. Reading the LCD

```
+----------------+
|UPRIGHT  30.2kg |   line 1: position + what the scale reads now
|AUTO  21.0kg OFF|   line 2: weight the assist is set for, AUTO/MAN, assist ON/OFF
+----------------+
```

**Line 1:**

| Shows | Meaning |
|---|---|
| `UPRIGHT` / `DRIVE` | Standing (wheels held) / fully tipped back (the assist may drive) |
| `30.2kg` | Steady scale reading |
| `~30.2kg` | Not steady yet, or not weighing (moving, tipped back) |
| `!160kg` | Over 150 kg |
| `-12.1kg`, `2480kg` … | A reading that can't be right. It's shown as it is: zero the scale; if it stays, report a scale fault. |
| `RAIL`, `NODAT`, `FROZ`, `NOCAL`, `IMU!` | Scale or tilt-sensor fault (section 6) |

**Line 2:**

| Shows | Meaning |
|---|---|
| `AUTO  21.0kg` | The weight was **read from the scale**. The assist is set for **0.7 ×** that weight (30 kg → 21 kg), so it never pushes harder than the load needs. |
| `MAN   15.0kg` | **Manual** weight: the 15 kg default at power-up, after a zero, or set by an engineer. The assist is set for exactly that weight (1.0 ×). |
| `ON` / `OFF` / `ERR` | Assist on / off / a motor controller reports a fault |

If line 2 reads `LD CONSOLE …`, an engineer has the maintenance console open and the assist is off.

---

## 3. Operating procedure

| Step | Procedure | LCD should show |
|---|---|---|
| **1. Connect power** | Laptop or controller supply to the board's mini-USB, then the motor battery or supply (**24 V**) to the ODrives. Check the motor controllers light up. `[FILL IN: battery steps]` | `RotaTruck` / `Starting...`, then the status |
| **2. Seat and zero the scale** | Put the load on the plate once and take it off (this seats the plate). Then **rest the cart tipped back, plate empty, hands off, and click three times.** | Line 1 about `1.0kg`, line 2 `MAN   15.0kg` |
| **3. Stand the cart** | Stand it **fully upright** on level floor | `UPRIGHT …` |
| **4. Load it** | Place the load **stable and centred** on the toe plate. Nobody touches the plate. | Line 1 rises (`~…kg`) |
| **5. Hands off** | Let go for about **1–2 s**. The weight is read **automatically** once it's steady. | Line 2 changes to `AUTO` with 0.7 × the weight |
| **5a. Or accept it yourself** | Still upright, **click the handle once** | Line 2 `AUTO …` |
| **6. Tip back fully** | Grip the handle and tip the cart back **all the way** to the pushing angle. The weight is now frozen. | Line 1 `DRIVE` |
| **7. Hold and push** | **Hold** the handle switch and start the cart moving. The motors help **once it rolls**: forward when you push, backward when you pull. The help fades by fast walking pace and brakes gently above 1 m/s. | Line 2 `… ON` |
| **8. To stop** | Slow the cart with your hands, or **let go of the switch**. It slows to a stop, then the assist switches off and the wheels roll freely. | Line 2 `… OFF` |
| **9. Unload or add load** | Stand it upright, change the load, hands off. The new **total** is read automatically. | Line 2 updates |
| **10. Switch off** | Disconnect the motor power, then the controller USB. Store the cart safely. | |

**If the board restarts** (power or USB interruption), the screen shows `Starting...` again, the weight goes back to `MAN 15.0kg`, and **the scale must be zeroed again** (step 2).

---

## 4. Safety rules

1. Keep a hand on the handle switch whenever the cart is moving. Letting go stops the motors. **Afterwards the wheels roll freely, so don't leave the cart on a slope.**
2. Do not exceed **150 kg** (RotaTruck LITE rating). The assist is sized for 60–90 kg.
3. Use only on flat, level, dry, indoor floor until ramp and outdoor tests are recorded.
4. Never stand on the toe plate or ride the cart.
5. Tip the cart slowly. Fast tilting holds the wheels (by design).
6. **Check line 2 before pushing.** If the weight looks wrong (for example `AUTO 110kg` with a light load), zero and re-read it, or ask an engineer to set it. An over-sized weight makes the assist push too hard.
7. **The controller's power comes from its USB cable.** Don't pull it while the motors are powered. `[VERIFY: ODrive behaviour on loss of CAN; fit a separate controller supply.]`
8. If anything behaves unexpectedly: **let go of the handle switch**, stand the cart upright, and disconnect the motor power. `[FILL IN: physical emergency stop, if fitted.]`

---

## 5. Before first use on a cart (engineer)

The cart's upright and fully tipped-back angles must be captured **on the cart**, with the tilt sensor (IMU) **firmly fixed**. Re-capture them whenever the IMU or the controller is moved, even slightly.

1. Open the console (`tools/rtt_monitor.sh` or the PlatformIO serial monitor), type `LD` and press Enter.
2. Cart **upright**, at least 15 kg on, hands off: `q s`.
3. Plate **empty**, cart **fully tipped back**: `q r`.
4. `q 1` (tilt-based scale handling on), then `LD` to leave.
5. Check that the LCD shows `UPRIGHT` when upright and `DRIVE` when fully tipped back.

`[STATUS 12 Nov 2026: q s re-captured on the cart; q r still to be captured.]`

---

## 6. Troubleshooting

| What you see | Likely cause | What to do |
|---|---|---|
| Line 2 stays `MAN` when upright and loaded | Not exactly upright (it must be within 2°), still moving, or the tilt sensor's angles are wrong (IMU moved) | Stand it fully up, hands off 2 s, or click once. If it never works, an engineer re-captures the angles (section 5). |
| Line 1 shows a big or negative number (for example `-999kg`, `850kg`) | The scale isn't zeroed (after a restart), or a scale wire is loose | Tip back, empty, triple-click. If it comes back, report the scale wiring. |
| Line 1 `RAIL` (or about `2480kg`) | A strain-gauge wire or joint is open | Report it for repair. The weight stays as it was (or `MAN`). |
| Reading creeps up or down by itself | Scale drift | Zero (triple-click) right before loading |
| First load reads far too high, later ones fine | The plate seated on the first load | Load once, unload, zero, then load again |
| `IMU!`, or the screen always says `UPRIGHT` | Tilt sensor (IMU) not answering. Its wires come loose easily. | Report it. Wheels are held and loads can't be accepted until it works. |
| LCD blank or frozen | LCD wire (SCL/SDA) loose | Re-seat the LCD wires; an engineer types `LCD` on the console to restart it |
| `ERR` on line 2 | A motor controller reports a fault | Stand the cart up and tip back again. If it stays, check the supply and wiring. |
| Holding the handle gives no help | Not fully tipped back, not yet rolling, or the motor controllers are off | Tip back fully (`DRIVE`), push to get it rolling, check the motor power |
| Screen shows `Starting...` by itself | The controller restarted (USB or power interruption) | Zero again (step 2). Report it if it happens often. |
