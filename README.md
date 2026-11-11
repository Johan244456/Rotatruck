# Power-Assisted RotaTruck

A manual RotaTruck LITE hand trolley fitted with two BLDC hub motors and a smart controller, so the motors amplify whatever the operator is already doing. Push and it helps you push; ease off and it eases off; pull back and it slows the cart down. There's no throttle and no lever — the cart follows your hands, and the amount of help scales with the load so a heavy cart feels close to an empty one.

Built by the **Power Assist Rangers** for **Omnia Wheel** as a UTS capstone project.

This repository holds the firmware, the documentation, and the supporting bench projects. The delivered firmware lives on the **`main`** branch.

---

## What's here

```
Power_Assisted_Rotatruck/
├── README.md                     you are here
├── CLAUDE.md                     developer/AI handoff notes (internal)
│
├── docs/                         all documentation — start at docs/README.md
│   ├── 01_overview/              what the project is and why
│   ├── 02_branch/                the ODrive + CAN integration reference (the deep dive)
│   ├── 03_software/              firmware architecture + UART command reference
│   ├── 04_hardware/              bill of materials, wiring, battery sizing
│   ├── 05_subsystems/            strain-gauge bench rig notes
│   ├── 06_operation/             how to drive it: user manual + mode guides + maintenance
│   └── 07_handover/              status, results, limitations, future work
│
├── code/
│   ├── STM_RotaTruck/            the production firmware (STM32 Nucleo-G491RE, PlatformIO)
│   └── low_fidelity/             the original Arduino proof-of-concept
│
└── Sensor_gauge_testing/         standalone HX711 + strain-gauge bench rig
```

## The short version of how it works

A single **STM32 Nucleo-G491RE** is the brain. It doesn't drive the motors itself — two **ODrive S1** controllers do that, one per rear wheel, talking to the STM32 over a single **CAN bus**. The STM32 reads the cart's tilt and motion from an **IMU**, works out how hard the operator is pushing, and tells each ODrive how much torque to add.

The clever part is that there's no force sensor on the handle. The controller runs a **momentum observer**: it knows the torque the motors are applying, so it knows how fast the cart *should* be accelerating; the difference between that and how the cart actually moves is the operator's push. Amplify that, send it to the wheels as torque, and the load gets lighter. The full explanation is in [docs/03_software/architecture.md](docs/03_software/architecture.md).

The front omni-wheels stay unpowered, so the cart still turns and manoeuvres exactly like the manual RotaTruck it's built on.

## Quick links

| If you want to… | Read |
|---|---|
| Actually drive the cart | [docs/06_operation/user-manual.md](docs/06_operation/user-manual.md) |
| Understand how the control works | [docs/03_software/architecture.md](docs/03_software/architecture.md) |
| Look up a UART command | [docs/03_software/command-reference.md](docs/03_software/command-reference.md) |
| Wire it up / find a part | [docs/04_hardware/wiring.md](docs/04_hardware/wiring.md), [docs/04_hardware/bill-of-materials.md](docs/04_hardware/bill-of-materials.md) |
| Re-flash or service it | [docs/06_operation/maintenance-and-updates.md](docs/06_operation/maintenance-and-updates.md) |
| Know what works and what's left | [docs/07_handover/status-and-limitations.md](docs/07_handover/status-and-limitations.md) |
| Read the ODrive/CAN deep dive | [docs/02_branch/ODRIVE_INTEGRATION.md](docs/02_branch/ODRIVE_INTEGRATION.md) |

## Build it

The firmware is a [PlatformIO](https://platformio.org/) project. The STM32 toolchain downloads itself on the first build.

```sh
# Build
pio run -d code/STM_RotaTruck

# Build and flash over the Nucleo's built-in ST-Link (USB)
pio run -d code/STM_RotaTruck -t upload

# Open the serial console (115200 8N1, same USB cable)
pio device monitor -d code/STM_RotaTruck
```

## A note on the branches

`main` is the delivered firmware. The other branches (`assist_test`, `State_Observer_Control`, and the now-merged `ODrive_S1`) are earlier development stages — the move from the original MakerBotics BLD300B speed drivers to the ODrive S1 torque controllers, and the control-loop experiments that led to the momentum observer. They're kept for history; you don't need them to build, run, or maintain the cart.

## Project

- **Client:** Omnia Wheel
- **Team:** Power Assist Rangers — Aldorino Gladwyn, Thomas Hoang, Mikhail Lazarev, Melroop Nijjar, Patipol Poungjun, Lachlan Torres
- **Course:** University of Technology Sydney (UTS), DMMS capstone
