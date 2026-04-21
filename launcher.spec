# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path

from PyInstaller.utils.hooks import collect_data_files, collect_submodules, copy_metadata

datas = collect_data_files("streamlit")
datas += collect_data_files("ttkbootstrap")
datas += copy_metadata("streamlit")
datas += copy_metadata("ttkbootstrap")
datas += copy_metadata("requests")
datas += copy_metadata("pyserial")
datas += [
    ("robot_arm.ino", "."),
    ("streamlit_app.py", "."),
    ("dume_endpoint_discovery.py", "."),
    ("ik_kinematics.py", "."),
    ("ik_path.py", "."),
    ("sequence_live.py", "."),
    ("sequence_module.py", "."),
    ("requirements.txt", "."),
    ("README.md", "."),
    ("config\\arm_geometry.json", "config"),
    ("ros2_dashboard\\app.py", "ros2_dashboard"),
    ("ros2_dashboard\\index.html", "ros2_dashboard"),
    ("ros2_dashboard\\script.js", "ros2_dashboard"),
    ("ros2_dashboard\\styles.css", "ros2_dashboard"),
    ("ros2_dashboard\\demo_profile.json", "ros2_dashboard"),
    ("ros2_dashboard\\shared_profile.json", "ros2_dashboard"),
    ("ros2_mock\\build_workspace.py", "ros2_mock"),
    ("ros2_mock\\dume_bridge.py", "ros2_mock"),
    ("ros2_mock\\mock_ros2_cli.py", "ros2_mock"),
    ("ros2_mock\\pick_and_place_server.py", "ros2_mock"),
    ("ros2_mock\\runtime.py", "ros2_mock"),
    ("ros2_mock\\README.md", "ros2_mock"),
    ("ros2_mock\\DH_PARAMETERS_AND_IK.md", "ros2_mock"),
    ("ros2_mock\\dume_ros2_mock\\action\\PickAndPlace.action", "ros2_mock\\dume_ros2_mock\\action"),
    ("branding\\logo-horizontal.svg", "branding"),
    ("branding\\logo-mark.svg", "branding"),
    ("branding\\icon-app.svg", "branding"),
    ("branding\\icon-app.png", "branding"),
    ("branding\\icon-app.ico", "branding"),
    ("branding\\banner-repo.svg", "branding"),
    ("branding\\social-card.svg", "branding"),
    ("branding\\brand-guide.md", "branding"),
]

hiddenimports = collect_submodules("streamlit")
hiddenimports += collect_submodules("ttkbootstrap")
hiddenimports += [
    "serial.tools.list_ports",
    "ros2_dashboard.app",
    "ros2_mock.build_workspace",
    "ros2_mock.dume_bridge",
    "ros2_mock.mock_ros2_cli",
    "ros2_mock.pick_and_place_server",
    "ros2_mock.runtime",
]

arduino_cli = Path(r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe")
binaries = []
if arduino_cli.exists():
    binaries.append((str(arduino_cli), "."))

a = Analysis(
    ["launcher.py"],
    pathex=[],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    exclude_binaries=False,
    name="DUM-E Launcher",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    icon="branding\\icon-app.ico",
    version=None,
)
