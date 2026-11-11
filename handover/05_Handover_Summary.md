# 05 Handover Summary

**Project:** P102 Power-Assisted RotaTruck LITE
**Client:** Carsten Borwieck, Engineering Manager, Omnia Wheel (with Mike, Omnia Wheel)
**From:** OmniPros, DMMS Team 4, Scope B (controls, safety, load detection, wiring and enclosure robustness), UTS 43019, Spring 2026
**Team:** Steven Lilin (Team Lead), Dang Khoa Tran (Documentation and Compliance), Hayden Choo (Stakeholder Engagement), Ethan Mam (Procurement), Kaung Myat Thu (Finance), Declan Wijaya (Health and Safety)
**Shared prototype with:** DMMS Team 5, Scope A (power, battery, mechanical concept, field testing)
**To:** `[FILL IN: next cohort / Omnia Wheel]`
**Date:** `[FILL IN: issue date]` (draft prepared 11 Nov 2026)
**Governing document:** Project Specification v1.0 (28 Aug 2026, scope approved 2 Sep 2026)

---

## Where we started

The previous cohort (Power Assist Rangers) delivered a working proof of concept: two ODrive S1 controllers driving the rear hub motors with direct torque control over CAN, supervised by an STM32 Nucleo-G491RE that infers the operator's push from motion (momentum observer) and amplifies it. It was demonstrated on flat ground at 45 to 60 kg from a bench supply. Their open issues: tilt / incline assist not working when the handle is held at an angle, no incline testing, manual payload entry, no emergency stop or dead-man, and tuning constants from an earlier cart. Our audit also found weak wiring and connectors.

## What we did

| Spec item | Outcome | Status |
|---|---|---|
| Assist-loop review (objective 1, FR2, NFR4) | Found that gravity compensation read the handle angle as a slope. Replaced with a road-slope estimate from acceleration, which handle angle does not affect. | Flat floor verified 10 Nov 2026. **Ramp untested.** |
| Operator release (FR1, FR3) | Handle switch on the handle. Let go and the wheels ramp to a stop in about 1.3 s and hold. A broken wire reads as released. | `[FILL IN: verified?]` |
| Invalid sensor data (FR3) | Replaced the obsolete MPU6050 with an ISM330DHCX; axes re-measured; automatic recovery from I2C errors and brown-outs. Load-sensor faults drop the payload to the manual value or 0. | IMU verified on truck 10 Nov 2026 |
| Automatic payload detection (FR5) | Six strain gauges on the toe plate, two HX711s, read by the motor firmware. The load measured while standing is latched into the assist when the cart is tipped back (90 % of measured, capped at 150 kg). Buttons remain as fallback. Calibration GUIs included. | Bench verified; in the truck firmware **built but not yet run** |
| Safety analysis | 26 fail scenarios with current controls and test references ([08](08_Safety_Analysis.md)) | Drafted; evidence mostly open |
| Wiring / enclosure robustness | `[FILL IN]` | `[FILL IN]` |
| Data support for Team 5's field tests | Serial streams available (`OB S`, `AS S`, `M S`, load CSV) | `[FILL IN: interface agreed?]` |

The firmware builds cleanly (11 Nov 2026, 90.5 KB flash). All changes are in `code/STM_RotaTruck/`; limits and tuning in `assist_tuning.h`, wiring options in `board.h`.

## What the next team needs to know first

1. **Load detection has not been run on the truck.** Do the bring-up and safety checks in [06](06_Testing_and_Commissioning.md) before relying on it. Until then the cart works with the manual payload buttons as before.
2. **Known safety gaps remain** ([08](08_Safety_Analysis.md), section 2): if the CAN bus is lost or the STM32 hangs, the ODrives keep their last torque (watchdog not enabled); if the IMU is lost while driving the assist carries on; a one-ODrive fault is not acted on; there is no dedicated e-stop yet.
3. **The ramp test is the most important open test** for the control algorithm.
4. **The mechanical tuning constants are still from an earlier cart.** Re-measure them, especially after Team 5 fits the battery.
5. **Any change in the cart's mass or balance needs a load-plate recalibration.** Coordinate with Team 5.
6. **Autonomous mode does not use load detection.** Use manual mode (the default) with the assist switched on while standing.
7. **Several targets are still "TBC with the client"** in the spec: reaction times, acceleration / slew limits, payload accuracy, incline limits, IP rating.
8. **Some older documents are now wrong:** the STM32 pin table in the ODrive setup guide, and the IMU / payload / LCD descriptions in `docs/`. This pack and the firmware take precedence. The Sprint 1 `Project Specification.docx` is superseded by v1.0.

## Recommended next steps

1. Close the safety gaps: ODrive CAN watchdog, STM32 watchdog, stop on IMU loss, stop both axes on a one-axis fault, accessible e-stop.
2. Bring up load detection on the truck; run the accuracy and failure tests.
3. Controlled ramp tests at 2 to 3 angles, uphill and downhill, empty and loaded.
4. Re-measure mass, wheelbase, inertia, friction and Kt.
5. Agree the open "TBC" limits with the client and set them in `assist_tuning.h`.
6. A single FR6 log record (command, speed, payload, safety state, faults) for Team 5's field tests.
7. Consider a handle force sensor for help from a standstill.

## Handover checklist

| Item | Done? |
|---|---|
| Firmware flashed on the cart matches `code/STM_RotaTruck/` (record the build) | `[FILL IN]` |
| Repository committed (currently no history) and pushed to the shared remote | `[FILL IN]` |
| Bench-rig code (`Code_STM/`) and test data moved into the repository | `[FILL IN]` |
| Load plate calibrated on the truck and calibration recorded | `[FILL IN]` |
| Test results entered in 06; evidence in 08; status in 04 | `[FILL IN]` |
| Wiring record (02, section 3.7) complete | `[FILL IN]` |
| Physical cart, PSU, spares and tools handed over; storage location: `[FILL IN]` | `[FILL IN]` |
| Accounts / access (GitHub, shared drive, ODrive GUI) transferred | `[FILL IN]` |
| Final design report submitted | `[FILL IN]` |
| Reviewed against the client-approved scope; client sign-off | `[FILL IN]` |

## Documents in this pack

[01 Project Overview](01_Project_Overview.md) · [02 Technical Handover](02_Technical_Handover.md) · [03 User Manual](03_User_Manual.md) · [04 Status, Limitations and Notes](04_Status_and_Limitations.md) · [06 Testing and Commissioning](06_Testing_and_Commissioning.md) · [07 Change Log](07_Change_Log.md) · [08 Safety Analysis](08_Safety_Analysis.md)

## Contacts

| Name | Role | Email |
|---|---|---|
| Steven Lilin | Team Lead | `[FILL IN]` |
| Dang Khoa Tran | Documentation and Compliance Officer | `[FILL IN]` |
| Hayden Choo | Stakeholder Engagement Officer | `[FILL IN]` |
| Ethan Mam | Procurement Officer | `[FILL IN]` |
| Kaung Myat Thu | Financial Officer | `[FILL IN]` |
| Declan Wijaya | Health and Safety Officer | `[FILL IN]` |
| Carsten Borwieck | Client, Omnia Wheel | `[FILL IN]` |
