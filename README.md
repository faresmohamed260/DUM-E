![DUM-E banner](./branding/banner-repo.svg)

## DUM-E

DUM-E is an ESP32-based robot arm system with three active layers:

- `robot_arm.ino`: the firmware that runs the arm, stores calibration in flash, handles Wi-Fi, and exposes the HTTP API.
- `streamlit_app.py`: the main control console for calibration, controller mapping, sequence recording, FK/IK path planning, and real robot operation.
- `ros2_dashboard/` + `ros2_mock/`: a portable ROS2-style presentation layer for the robotics assignment while DUM-E remains the real execution backend.

The active repository contains the production project only. Experimental work and unfinished prototypes live outside the tracked scope and are intentionally excluded from GitHub.

## Active Project Layout

- `robot_arm.ino`: ESP32 firmware
- `streamlit_app.py`: main DUM-E dashboard
- `ik_kinematics.py`: forward/inverse kinematics helpers
- `ik_path.py`: IK path construction and guarded pick-and-place execution
- `sequence_module.py`: saved sequence definitions and replay logic
- `sequence_live.py`: live controller-driven sequence recorder
- `dume_endpoint_discovery.py`: endpoint discovery and recovery helpers
- `config/arm_geometry.json`: arm geometry and mapping configuration for IK
- `ros2_dashboard/`: Windows browser panel for the ROS2-style assignment interface
- `ros2_mock/`: local ROS2-shaped compatibility service used by the portable panel
- `launcher.py`: desktop launcher entrypoint
- `launcher.spec`: PyInstaller spec for the portable executable
- `scripts/`: helper scripts for Python deps, Bluepad32 core install, and launcher builds
- `branding/`: icons, banner, and brand assets
- `requirements.txt`: Python dependencies for local development

## What The System Does

### Main DUM-E dashboard

The main dashboard supports:

- Wi-Fi setup and reconnect
- wireless controller pairing and mapping
- per-joint calibration with flash persistence
- mixed motor semantics (`positional_180` and `continuous_360`)
- live state inspection
- controller-driven sequence recording and replay
- FK/IK-assisted motion planning
- guarded pick-and-place execution with gripper timing

### Portable ROS2-style dashboard

The ROS2 panel supports:

- preparing the packaged ROS2 runtime
- starting the assignment-facing action server
- sending pick-and-place goals
- showing live nodes, actions, topics, logs, and feedback

This layer is presentation-focused. DUM-E still performs the real motion, FK/IK, gripper actuation, and HTTP execution.

## Requirements

For local source development:

- Windows 10/11
- Python `3.10`
- Arduino IDE or `arduino-cli`
- ESP32 Bluepad32 board package
- a USB connection to the ESP32 board for flashing
- optional: DualShock 4 controller for manual driving/recording

For the portable packaged flow:

- Windows 10/11
- browser available locally
- USB/serial driver support for the ESP32 board
- no internet required after the package is built

## Installation

Use the full setup guide here:

- [INSTALLATION.md](./INSTALLATION.md)

That guide covers:

- Python environment setup
- dependency installation
- Arduino/Bluepad32 installation
- firmware flashing
- first dashboard run
- ROS2 panel usage
- portable launcher creation

## Quick Start

### 1. Install Python dependencies

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\install_python_deps.ps1
```

### 2. Install the ESP32 Bluepad32 core

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\install_bluepad32_core.ps1
```

### 3. Compile and flash the firmware

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32-bluepad32:esp32:esp32 .
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' upload -p COM4 --fqbn esp32-bluepad32:esp32:esp32 .
```

Replace `COM4` with the actual board port.

### 4. Launch the main dashboard

```powershell
python -m streamlit run .\streamlit_app.py
```

### 5. Optional: build the portable launcher

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\build_launcher.ps1
```

That produces the packaged Windows launcher in `dist/`.

## First Run Checklist

1. Flash the ESP32 firmware.
2. Power the arm.
3. Connect to the setup AP if station Wi-Fi is not configured yet.
4. Open the main dashboard and connect to the DUM-E endpoint.
5. Restore or create joint calibration.
6. Pair the controller if you want live manual driving.
7. Save calibration to flash.
8. Test the main dashboard before using the ROS2 panel.

## Controller Pairing

1. Enable wireless pairing in the main dashboard.
2. Hold `SHARE + PS` on the DualShock 4 until the light flashes rapidly.
3. Refresh the dashboard state.
4. Turn pairing mode back off once the controller is remembered.

## Notes On Motor Types

- `positional_180`
  - command values represent angle/target semantics
  - ideal for `base`, `shoulder`, `elbow`, and `gripper`
- `continuous_360`
  - command values represent speed/direction semantics
  - should be handled conservatively and with feedback-aware logic

## ROS2 Assignment Layer

The repository includes:

- `ros2_mock/dume_ros2_mock/action/PickAndPlace.action`
- `ros2_mock/pick_and_place_server.py`
- FK/IK implementations used by the DUM-E backend
- assignment-facing state machine and feedback publishing

The ROS2 panel is meant to demonstrate those interfaces while keeping DUM-E as the real execution engine.

## Portable Launcher

The packaged launcher can:

- install/repair the Bluepad32 Arduino core in a portable app data directory
- flash firmware to the board
- open the main DUM-E dashboard
- open the ROS2 assignment dashboard

Important:

- the launcher is a Windows packaging target, not the source of truth
- the source files in this repository remain the authoritative implementation

## Ignored / Non-Tracked Content

The following are intentionally excluded from GitHub:

- `experiments/`
- generated `output/`
- `dist/`, `build/`
- portable launcher runtime state
- local device cache and temporary dashboard runtime files

## Validation Commands

Python syntax:

```powershell
python -m py_compile .\streamlit_app.py .\launcher.py .\ik_kinematics.py .\ik_path.py
```

Firmware compile:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32-bluepad32:esp32:esp32 .
```

Portable launcher build:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\build_launcher.ps1
```
