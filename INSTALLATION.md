# Installation Guide

This guide covers both supported ways to use DUM-E:

- local source development
- packaged portable launcher usage

The active GitHub repository contains only the production project. Experimental folders are intentionally excluded.

## 1. System Requirements

### Minimum host requirements

- Windows 10 or Windows 11
- USB data cable for the ESP32 board
- a modern web browser
- permission to use local loopback ports on the machine

### For source development

- Python `3.10.x`
- Arduino IDE installed, or `arduino-cli` available
- network access only for the initial dependency and board-package install

### For the portable package

- no Python install required
- no WSL required
- no ROS2 install required
- no internet required after the portable bundle is prepared

## 2. Repository Setup

Clone the repository:

```powershell
git clone https://github.com/faresmohamed260/DUM-E.git
cd DUM-E
```

## 3. Python Setup

Create and activate a virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

Install dependencies:

```powershell
pip install -r .\requirements.txt
```

Or use the helper script:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\install_python_deps.ps1
```

## 4. Arduino / ESP32 Bluepad32 Setup

The firmware uses the ESP32 Bluepad32 board package.

Install it with the helper script:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\install_bluepad32_core.ps1
```

If you prefer the CLI directly:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' core update-index --additional-urls https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' core install esp32-bluepad32:esp32@4.1.0 --additional-urls https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
```

## 5. Compile And Flash Firmware

Compile:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32-bluepad32:esp32:esp32 .
```

Upload:

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' upload -p COM4 --fqbn esp32-bluepad32:esp32:esp32 .
```

Replace `COM4` with the real port for the board.

If upload is unstable:

- lower the upload baud rate
- disconnect unnecessary hardware loads from the board during flashing
- manually use `BOOT` and `EN` if the board requires that sequence

## 6. First Firmware Boot

After flashing:

1. power the arm
2. connect to the DUM-E setup AP if station Wi-Fi is not configured
3. open the main dashboard
4. connect to the current DUM-E endpoint
5. calibrate or restore all joints
6. save calibration to flash

## 7. Run The Main Dashboard

Start the main dashboard:

```powershell
python -m streamlit run .\streamlit_app.py
```

The dashboard is the real operational interface for:

- calibration
- controller pairing
- sequence recording
- IK path building
- guarded pick-and-place execution

## 8. Controller Pairing

To pair a DualShock 4:

1. enable pairing mode in the dashboard
2. hold `SHARE + PS` until the controller LED flashes rapidly
3. wait for DUM-E to connect
4. disable pairing mode when done

## 9. Configure IK / Motion Planning

Before relying on IK:

- verify `config/arm_geometry.json`
- confirm the platform reference frame:
  - origin at platform-top center
  - `z = 0` at the platform top
  - negative `z` below that surface
- validate start/end capture and safe travel height from the main dashboard

## 10. ROS2 Assignment Panel

The repository includes a portable ROS2-style layer:

- `ros2_dashboard/`
- `ros2_mock/`

This layer is presentation-facing. DUM-E still does the real work underneath.

It can:

- prepare the packaged runtime
- start the assignment-facing action server
- send pick-and-place goals
- show live nodes, actions, topics, logs, and feedback

## 11. Build The Portable Launcher

Build with:

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\build_launcher.ps1
```

Or directly:

```powershell
.\.venv\Scripts\python.exe -m PyInstaller --clean --noconfirm .\launcher.spec
```

Output goes to:

- `dist\DUM-E Launcher.exe`

The launcher packages:

- the main DUM-E dashboard
- the ROS2 assignment panel
- firmware flashing support
- the portable local ROS2 compatibility layer

## 12. Portable Usage

On a target Windows PC:

1. copy the built `dist` contents
2. run the launcher executable
3. use the launcher to:
   - install/repair the Bluepad32 core locally if needed
   - flash firmware
   - launch the DUM-E dashboard
   - launch the ROS2 panel

The portable runtime stores its writable state beside the executable in a local data folder.

## 13. Recommended Validation

### Python

```powershell
python -m py_compile .\streamlit_app.py .\launcher.py .\ik_kinematics.py .\ik_path.py .\sequence_module.py .\sequence_live.py
```

### Firmware

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn esp32-bluepad32:esp32:esp32 .
```

### Launcher

```powershell
pwsh -ExecutionPolicy Bypass -File .\scripts\build_launcher.ps1
```

## 14. Troubleshooting

### Dashboard opens but returns 404

- ensure you are using the newest launcher build
- close all previous launcher copies before retrying
- verify the launcher log for the assigned local port

### ROS2 panel opens but actions do nothing

- make sure the main DUM-E dashboard is working first
- use `Prepare ROS2 Runtime`
- then `Start ROS2 Server`
- then send the goal

### Firmware flashing fails

- verify the COM port
- try a different USB cable or port
- use manual `BOOT/EN` timing if the board needs it

### No controller input appears

- re-enable pairing mode
- re-pair the controller
- confirm live `/api/state` is updating

## 15. Tracked vs Ignored Content

Tracked:

- firmware
- dashboards
- ROS2 compatibility layer
- launcher
- IK/FK logic
- install/build scripts
- configuration
- documentation

Ignored:

- `experiments/`
- `output/`
- `build/`
- `dist/`
- local runtime state
- local caches
