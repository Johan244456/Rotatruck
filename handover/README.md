# Handover Pack: Power-Assisted RotaTruck (OmniPros, DMMS Team 4, Scope B)

This folder is the handover from **OmniPros (DMMS Team 4, project P102)** for the Power-Assisted RotaTruck, for Omnia Wheel and the next team. It documents what this team added to the system it inherited from the previous cohort (Power Assist Rangers): automatic load detection, the IMU replacement, the handle dead-man switch, the slope-based gravity compensation, safety analysis, and wiring robustness. The previous team's handover in [`../docs/07_handover/`](../docs/07_handover/) was used as the reference for structure and content.

The governing document is the **Project Specification v1.0** (28 Aug 2026, scope approved by Carsten Borwieck, Omnia Wheel, on 2 Sep 2026): [`../code/STM_RotaTruck/Team 4 Rotatruck Project Specification.pdf`](../code/STM_RotaTruck/Team%204%20Rotatruck%20Project%20Specification.pdf). Requirement IDs (FR1 to FR7, NFR1 to NFR8) in this pack and in the code refer to it.

**Status of this pack:** draft, prepared 11 Nov 2026 from the firmware, the repository, the specification and the team's working files. `[FILL IN: ...]` marks input the team still has to provide. `[VERIFY: ...]` marks facts inferred from the code or files that should be confirmed.

## Documents

| # | Document | Read it if you want to... | Spec link |
|---|---|---|---|
| 01 | [Project Overview](01_Project_Overview.md) | Understand the project, the team, Scope B's responsibilities and the interface with Team 5 | Sections 1 to 5 |
| 02 | [Technical Handover](02_Technical_Handover.md) | Maintain or extend the firmware and electronics: architecture, new modules, pin map, wiring record, tuning, calibration, tools | Control and Safety Software; Updated Shared Prototype |
| 03 | [User Manual](03_User_Manual.md) | Operate the cart: power-up, loading, driving, handle switch, LCD states, troubleshooting | |
| 04 | [Status, Limitations and Notes](04_Status_and_Limitations.md) | Know what works, what is verified, requirements traceability, open issues, next steps | Final Design Report and Handover |
| 05 | [Handover Summary](05_Handover_Summary.md) | Get the whole picture in two pages, plus the handover checklist | |
| 06 | [Testing and Commissioning](06_Testing_and_Commissioning.md) | Bring up the new hardware and run the verification tests (blank results tables) | Verification evidence |
| 07 | [Change Log](07_Change_Log.md) | See exactly what changed relative to the previous team's delivery | |
| 08 | [Safety Analysis](08_Safety_Analysis.md) | Fail scenarios, adopted controls, gaps and evidence | Safety Analysis deliverable |

## How this pack relates to the existing documentation

- The previous team's documentation is still in [`../docs/`](../docs/) (start at [`../docs/README.md`](../docs/README.md)). It is still correct for the parts this team did not change (ODrive setup, CAN bus, observer theory, motor harness).
- Where this pack and `../docs/` disagree, **this pack and the firmware (`board.h`, `assist_tuning.h`) win.** The main differences are the IMU (ISM330DHCX, not MPU6050), the handle switch, automatic load detection, the slope-based gravity compensation and the LCD layout.
- [`../RotaTruck_Handover_Summary.pdf`](../RotaTruck_Handover_Summary.pdf) is this team's review of the previous handover, written at the start of the project.
- [`../code/STM_RotaTruck/LOAD_DETECTION.md`](../code/STM_RotaTruck/LOAD_DETECTION.md) is the detailed engineering note for the load-detection integration.
- `../Project Specification.docx` is the superseded Sprint 1 draft (different client contact and requirement numbering). Use v1.0.

## Open fill-in items (collected)

1. Each member's technical contribution and contact details (01, 05).
2. Client decisions on the "TBC" items: operating envelope, reaction times, acceleration / slew limits, payload accuracy, incline limit, IP target (01, 04, 08).
3. Handle switch part number, mounting position and photo (02, 03).
4. Wiring, connector, mounting and enclosure changes (02 section 3.7, 06 section 7).
5. Updated bill of materials and costs (02).
6. On-truck bring-up, load-detection accuracy and safety test results (06, then 04, 05 and the evidence log in 08).
7. Controlled ramp test results and ramp angles (06, 04).
8. Interface agreement with Team 5: data / log format for field testing, connectors, power budget, battery voltage (01, 04).
9. Decision on a physical emergency stop (04, 08).
10. Photos for the user manual steps that changed (03).
