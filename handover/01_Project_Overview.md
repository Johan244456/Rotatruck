# 01 Project Overview

## The product

The RotaTruck LITE is Omnia Wheel's omni-wheeled hand trolley. The Power-Assisted RotaTruck adds two BLDC hub motors on the rear wheels, driven by two ODrive S1 controllers and supervised by an STM32 Nucleo-G491RE, so the motors amplify the operator's own push and pull. There is no throttle or lever: the cart infers the operator's effort from how it moves (a momentum observer) and adds torque in the same direction. The front omni-wheels stay unpowered, so the cart steers like the manual RotaTruck.

## Project history

| Phase | Team | Outcome |
|---|---|---|
| Previous DMMS cohort | Power Assist Rangers | Working proof of concept on flat ground: ODrive S1 torque control over CAN, momentum-observer assist, pose state machine, manual payload entry, bench power supply. Demonstrated to the client at about 45 to 60 kg. Their handover is in [`../docs/07_handover/`](../docs/07_handover/) and is the reference this pack is built on. |
| This cohort (Spring 2026), Scope B | **OmniPros, DMMS Team 4 (P102)** | Controls, safety, load detection, and wiring / mounting / enclosure robustness. Covered in this pack. |
| This cohort (Spring 2026), Scope A | DMMS Team 5 | Power and battery integration, broader mechanical concept, system weight, daily operating lifecycle, and the full real-life field-test program. Same physical cart. `[FILL IN: Team 5 name and contact]` |

## Client and team

| Role | Name |
|---|---|
| Client | Carsten Borwieck, Engineering Manager, Omnia Wheel |
| Client contact | Mike, Omnia Wheel `[FILL IN: surname / role]` |
| Course | UTS 43019 Design in Mechanical and Mechatronic Systems (DMMS) |
| Project specification | Version 1.0, 28 Aug 2026, scope approved by C. Borwieck on 2 Sep 2026 ([`../code/STM_RotaTruck/Team 4 Rotatruck Project Specification.pdf`](../code/STM_RotaTruck/Team%204%20Rotatruck%20Project%20Specification.pdf)) |

| Member | Team role (spec) | Main technical contribution | Contact |
|---|---|---|---|
| Steven Lilin | Team Lead | `[FILL IN]` | `[FILL IN]` |
| Dang Khoa Tran | Documentation and Compliance Officer | Automatic load detection (strain-gauge load plate, HX711 firmware, calibration GUIs) and its integration into the RotaTruck firmware `[VERIFY]` | `[FILL IN]` |
| Hayden Choo | Stakeholder Engagement Officer | `[FILL IN]` | `[FILL IN]` |
| Ethan Mam | Procurement Officer | `[FILL IN]` | `[FILL IN]` |
| Kaung Myat Thu | Financial Officer | `[FILL IN]` | `[FILL IN]` |
| Declan Wijaya | Health and Safety Officer | `[FILL IN]` | `[FILL IN]` |

## Problem being solved (from the specification)

The inherited RotaTruck is a working proof of concept but not ready for reliable real-world use. The assist loop had not been reviewed against uphill, downhill, turning, kerb and fault conditions; payload is entered by hand; and the prototype needs clearer fail-safe behaviour and more robust wiring and mounting. Pushing and pulling loaded trolleys is a hazardous manual task (SafeWork NSW Hazardous Manual Tasks Code of Practice, ISO 11228-2), so the project must reduce operator effort **without creating new hazards** such as unintended motion, sudden acceleration, loss of control on a slope, sensor failure or wiring failure.

## Scope B responsibilities (approved)

| Design objective | Evaluation criterion | Timeframe |
|---|---|---|
| Review and improve the assist loop so commanded motor output stays predictable across the agreed operating envelope | Code review, bench tests and shared field-test logs show no unstable or uncommanded assist | Sprint 2 to 3 |
| A clear safety architecture covering operator release, emergency stop, invalid sensor data and controller faults | Fail-scenario table completed, each safety response demonstrated (reaction-time targets TBC with client) | Sprint 2 |
| Automatic payload detection, or a validated workaround that removes manual entry | Repeated tests against known loads: accuracy, repeatability, failure behaviour (tolerance TBC) | Sprint 2 to 3 |
| Wiring, connector, mounting and enclosure robustness | Inspection and shake / handling tests: secured wiring, strain relief, protected connections, no intermittent faults (IP target TBC) | Sprint 2 to 3 |
| Controls data, telemetry and interface support for Scope A's field testing | Data / interface handoff agreed with Scope A before field testing | Before Sprint 3 tests |
| Updated software and clear engineering documentation for Omnia Wheel | Final report, code, test evidence, wiring records and open-issue list reviewed against the approved scope | End of semester |

**Deliverables:** Control and Safety Software; Safety Analysis (fail-scenario analysis, adopted controls, verification evidence); Load Detection Outcome; Updated Shared Prototype; Final Design Report and Handover.
**Stretch goals:** telemetry or an additional software feature where it supports a clear safety or software need; integration with Scope A's battery on a single power-independent prototype.
**Out of scope:** commercial production, regulatory certification, mass manufacturing, commercial deployment, marketing.

## What this team has delivered so far

| Area | Delivered | Verified on the truck? |
|---|---|---|
| Assist loop review: incline / tilt | Gravity compensation now uses an **estimated road slope** (from acceleration) instead of the frame tilt, so holding the handle higher or lower no longer reads as a slope. Targets the previous team's top limitation. | Flat floor: yes (10 Nov 2026, no phantom push). Ramp: **not yet**. |
| Safety: operator release (dead-man) | **Handle safety switch** on A0. Released = both wheels ramp down to a stop and are held. Fail-safe wiring (open circuit = stop). | `[FILL IN: tested on the truck? date, result]` |
| Safety: sensor | MPU6050 (obsolete) replaced by **ISM330DHCX** (SparkFun SEN-19764). Axis signs and pose references re-measured on the truck. I2C bus recovery and automatic sensor re-configuration added. | Yes, axis mapping measured 10 Nov 2026 |
| Automatic payload detection | **Strain-gauge load plate** (6 x BF350, 2 x HX711) read by the motor firmware. Load latched into the assist when the cart is tipped back; buttons kept as manual fallback. | Bench rig: yes. In the truck firmware: **not run on hardware yet** |
| Calibration tools | Python GUIs (`host/`): calibration dashboard with paired capture and slope + offset fit, drift / stability test | Bench rig: yes. With the truck firmware: `[FILL IN]` |
| Safety analysis | Fail-scenario table drafted in [08 Safety Analysis](08_Safety_Analysis.md) | Evidence to be collected |
| Wiring / mounting / enclosure robustness | `[FILL IN: what has been changed]` | `[FILL IN]` |
| Data for Scope A field testing | Existing serial streams (`OB S`, `AS S`, `M S`) and the load-detect CSV. No dedicated logger yet. | `[FILL IN: interface agreed with Team 5?]` |

Full detail: [02 Technical Handover](02_Technical_Handover.md) and [04 Status and Limitations](04_Status_and_Limitations.md).

## Design targets

| Parameter | Value |
|---|---|
| Operational payload | 60 kg (lab-tested up to about 90 kg by the previous team) |
| Rated payload (load detection cap) | 150 kg (RotaTruck LITE rating) |
| Assist payload safety factor | 0.9 x measured load (under-assisting is the safe side) |
| Assisted top speed | Assist fades out by 0.7 m/s; overspeed brake above 1.0 m/s |
| Handle-release stop | Ramp at 1.5 rev/s^2 (about 0.78 m/s^2), about 1.3 s from 1 m/s |
| Slope handled by the estimator | Clamped to +/-12 deg; deadband +/-2.5 deg |
| Reaction times, acceleration / slew limits, payload accuracy, IP rating | **TBC with the client** (spec v1.0). `[FILL IN: agreed values]` |

## Decisions with the client

| Date | Decision / outcome | Source |
|---|---|---|
| 28 Aug 2026 | Project specification v1.0 issued | Spec document |
| 2 Sep 2026 | Scope deliverables, Scope B allocation (controls, user safety, robustness, load detection, additional software features) and the team interface approved by C. Borwieck | Spec section 10 |
| `[FILL IN]` | Agreed operating envelope (speed, acceleration, load, incline limits) | `[FILL IN]` |
| `[FILL IN]` | Reaction-time targets, payload accuracy tolerance, IP target | `[FILL IN]` |
| `[FILL IN]` | Choice of presence-sensing method (handle switch) | `[FILL IN]` |

## Interfaces with Team 5 (Scope A)

The cart is shared, so changes on either side affect the other (spec NFR7):

- **Field testing is run by Team 5.** Scope B supplies the controls data, telemetry and technical support. `[FILL IN: agreed signal list, log format, who operates the console during tests]`
- **Load detection calibration depends on the cart's weight and balance.** If the battery, motors, enclosure or handle are moved, added or removed, the load plate must be recalibrated (see 02, section 6.4).
- **Tuning constants** (`TROLLEY_MASS_EMPTY_KG` etc.) change when the battery is fitted.
- **Supply voltage.** Battery voltage range must stay inside the ODrive under- and over-voltage trips. `[FILL IN: agreed range]`
- **I2C bus** is shared by the IMU and LCD and is 3.3 V only; any new device on it must not pull up to 5 V.
- `[FILL IN: agreed connector standard, controller-rail power budget, mounting space for the HX711 boards and IMU, integration dates.]`
