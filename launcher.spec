# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import collect_data_files, collect_submodules, copy_metadata

datas = collect_data_files("streamlit")
datas += copy_metadata("streamlit")
datas += copy_metadata("requests")
datas += copy_metadata("pyserial")
datas += [
    ("robot_arm.ino", "."),
    ("streamlit_app.py", "."),
    ("requirements.txt", "."),
    ("README.md", "."),
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
hiddenimports += [
    "serial.tools.list_ports",
]

a = Analysis(
    ["launcher.py"],
    pathex=[],
    binaries=[],
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
    [],
    exclude_binaries=True,
    name="DUM-E Launcher",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    icon="branding\\icon-app.ico",
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="DUM-E Launcher",
)
