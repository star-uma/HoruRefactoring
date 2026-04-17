# HoruRefactoring

Refactoring of the **Horu** robot control system — migrating from Simulink + Arduino IDE to a fully version-controlled Python + VS Code environment.

## What is Horu?

Horu is a tracked robot with four independently articulated crawler arms. Each arm is driven by a stepper motor (elevation) and a brushless motor (traction), giving the robot the ability to traverse obstacles and self-level on uneven terrain.

## Repository structure

```
HoruRefactoring/
├── control/
│   └── horu_control.py         # PC controller — reads Xbox gamepad, sends UDP
├── firmware/
│   └── horu_mkr_traccion/
│       └── horu_mkr_traccion.ino  # Arduino MKR WiFi 1010 firmware
└── docs/
    └── TFG_GIEI_Mario_Garcia_Jimenez_O1JUN_25.pdf
```

## System architecture

```
[PC — horu_control.py]
        |
        |  UDP (18 bytes, 9 × int16 LE) @ 50 pkt/s
        |  WiFi "Horu" — 192.168.10.101:8885
        ▼
[Arduino MKR WiFi 1010]
        |
        |  CAN bus @ 1 Mbps
        ▼
[RMD-X8 traction motors]  FL(0x141)  FR(0x142)  RR(0x143)  RL(0x144)
        |
        |  UART
        ▼
[ESP32]
        |
        |  I2C (TCA9548A mux + AS5600 encoders)
        ▼
[DM542 stepper drivers — elevation motors]
```

## UDP datagram format

| Index | Field | Description |
|---|---|---|
| 0 | `id` | Always 1 (PC origin) |
| 1 | `action_left_train` | Left traction speed (dps × 100) |
| 2 | `action_right_train` | Right traction speed (dps × 100) |
| 3–6 | `o0..o3` | Crawler target angles / directions (modes 2–4) |
| 7 | `mode` | Active control mode (1..5) |
| 8 | `code_error` | 0 = no error |

## Control modes

| Mode | Name | Description |
|---|---|---|
| 1 | Traction | Differential drive — left stick (speed) + right stick (turn) |
| 2 | Absolute position | Set all crawlers to a fixed angle via A/B/X/Y buttons |
| 3 | Incremental × 4 | Tilt all crawlers together via D-pad |
| 4 | Incremental × 2 | Control crawler pairs independently via D-pad + triggers |
| 5 | Auto-levelling | IMU-based automatic horizontal levelling (hold Start) |

## Xbox controller mapping

| Input | Action |
|---|---|
| Left stick (vertical) | Forward / backward |
| Right stick (horizontal) | Turn |
| RB | Cycle mode: 1 → 2 → 3 → 4 → 5 → 1 |
| A / B / X / Y | Absolute position (Mode 2): 225° / 180° / 135° / 90° |
| D-pad + triggers | Crawler control (Modes 3 / 4) |
| Start (hold) | Auto-levelling (Mode 5) |

## Requirements

**PC (horu_control.py)**
- Python 3.8+
- `pygame`

```bash
pip install pygame
```

**Arduino MKR WiFi 1010**
- `WiFiNINA`
- `CAN`

## Running

```bash
# Normal operation
python control/horu_control.py

# Test mode — verify gamepad axes without sending to robot
python control/horu_control.py --test
```

## Branch strategy

| Branch | Purpose |
|---|---|
| `main` | Stable, tested code |
| `dev` | Integration branch |
| `feature/*` | New features or modes |
| `fix/*` | Bug fixes |

## Safety

The Arduino firmware includes a **500 ms watchdog**: if no UDP packet is received within that window (connection lost, script stopped), all traction motors are released immediately.
