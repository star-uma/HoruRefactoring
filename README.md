# StarCrawler

Refactoring of the **Horu** robot control system — renamed **StarCrawler**. Migrates from Simulink + Arduino IDE to a fully version-controlled Python + VS Code environment.

## What is StarCrawler?

StarCrawler is a tracked robot with four independently articulated crawler arms. Each arm is driven by a stepper motor (elevation) and a brushless motor (traction), giving the robot the ability to traverse obstacles and self-level on uneven terrain.

## Repository structure

```
StarCrawler/
├── .vscode/
│   └── arduino.json                          # VS Code Arduino configuration
├── control/
│   └── StarCrawlerXbox.py                    # PC controller — reads Xbox gamepad, sends UDP
├── firmware/
│   └── starcrawler_mkr_traccion/
│       └── starcrawler_mkr_traccion.ino      # Arduino MKR WiFi 1010 firmware
└── docs/
    └── TFG_GIEI_Mario_Garcia_Jimenez_O1JUN_25.pdf
```

## System architecture

```
[PC — StarCrawlerXbox.py]
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

---

## Development environment setup

### Requirements

| Tool | Version | Notes |
|---|---|---|
| Python | 3.12 (recommended) | 3.15 has compatibility issues with pygame |
| Git | 2.53+ | |
| VS Code | 1.116+ | |
| Arduino CLI | 1.4.1 | Manually installed at `C:\arduino-cli\` |

### VS Code extensions

| Extension | ID |
|---|---|
| Arduino Community Edition | `vscode-arduino` |
| Python | `ms-python.python` |
| Pylance | `ms-python.vscode-pylance` |
| GitLens | `eamodio.gitlens` |
| Error Lens | `usernamehakobyan.error-lens` |
| Serial Monitor | `ms-vscode.serial-monitor` |

---

## Installation from scratch (Windows)

### 1. Python

Download Python 3.12 from https://www.python.org/downloads/release/python-3128/
During installation mark: ☑️ **Add python.exe to PATH**

Install dependencies:
```powershell
python -m pip install pygame
```

### 2. Arduino CLI

Download the Windows 64-bit ZIP from https://arduino.github.io/arduino-cli/latest/installation/

```powershell
mkdir C:\arduino-cli
Copy-Item "path\to\arduino-cli.exe" "C:\arduino-cli\arduino-cli.exe"
$env:Path += ";C:\arduino-cli"
[Environment]::SetEnvironmentVariable("Path", [Environment]::GetEnvironmentVariable("Path","User") + ";C:\arduino-cli", "User")
```

Verify:
```powershell
arduino-cli version
```

### 3. Arduino boards and libraries

```powershell
arduino-cli core update-index
arduino-cli core install arduino:samd
arduino-cli lib install "WiFiNINA"
arduino-cli lib install "CAN"
```

Verify board is detected (connect Arduino via USB first):
```powershell
arduino-cli board list
```
Expected output: Arduino MKR WiFi 1010 on COM3.

### 4. SSH key for GitHub

```powershell
ssh-keygen -t ed25519 -C "your_email@gmail.com"
type $env:USERPROFILE\.ssh\id_ed25519.pub
```

Copy the output and add it to GitHub → Settings → SSH and GPG keys → New SSH key → **Authentication Key**.

Verify:
```powershell
ssh -T git@github.com
```

### 5. Git identity

```powershell
git config --global user.email "your_email@gmail.com"
git config --global user.name "your_username"
```

### 6. Clone the repository

```powershell
git clone -b main git@github.com:star-uma/StarCrawler.git
```

---

## Running

### PC controller

```powershell
cd control
python StarCrawlerXbox.py
```

Test mode (verify gamepad axes without sending to robot):
```powershell
python StarCrawlerXbox.py --test
```

### Compile firmware

```powershell
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 "firmware/starcrawler_mkr_traccion/starcrawler_mkr_traccion.ino"
```

### Flash firmware

```powershell
arduino-cli upload -p COM3 --fqbn arduino:samd:mkrwifi1010 "firmware/starcrawler_mkr_traccion/starcrawler_mkr_traccion.ino"
```

---

## Git workflow

### Create a new feature branch

```powershell
git checkout -b feature/branch-name
git push -u origin feature/branch-name
```

### Daily workflow

```powershell
git add .
git commit -m "type(scope): description"
git push
```

### Branch strategy

| Branch | Purpose |
|---|---|
| `main` | Stable, tested code |
| `dev` | Integration branch |
| `feature/*` | New features or modes |
| `fix/*` | Bug fixes |

---

## Safety

The Arduino firmware includes a **500 ms watchdog**: if no UDP packet is received within that window (connection lost, script stopped), all traction motors are released immediately.