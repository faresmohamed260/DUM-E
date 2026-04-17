from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
import webbrowser
from pathlib import Path

import ttkbootstrap as tb
from serial.tools import list_ports
from ttkbootstrap.constants import BOTH, END, LEFT, X
from ttkbootstrap.scrolled import ScrolledText


APP_NAME = "DUM-E Launcher"
BOARD_FQBN = "esp32-bluepad32:esp32:esp32"
BLUEPAD_URL = "https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json"
DEFAULT_DASHBOARD_URL = "http://localhost:8501"


def is_frozen() -> bool:
    return getattr(sys, "frozen", False)


def bundle_root() -> Path:
    if is_frozen() and hasattr(sys, "_MEIPASS"):
        return Path(sys._MEIPASS)
    return Path(__file__).resolve().parent


def app_root() -> Path:
    if is_frozen():
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


def resource_path(name: str) -> Path:
    return bundle_root() / name


def portable_root() -> Path:
    path = app_root() / ".dume-portable"
    path.mkdir(parents=True, exist_ok=True)
    return path


def arduino_dirs() -> dict[str, Path]:
    root = portable_root()
    data = root / "arduino-data"
    downloads = root / "downloads"
    user = root / "arduino-user"
    for path in (data, downloads, user):
        path.mkdir(parents=True, exist_ok=True)
    return {"data": data, "downloads": downloads, "user": user}


def arduino_cli_config_path() -> Path:
    dirs = arduino_dirs()
    config_path = portable_root() / "arduino-cli.yaml"
    config_path.write_text(
        "\n".join(
            [
                "board_manager:",
                "  additional_urls:",
                f"    - {BLUEPAD_URL}",
                "directories:",
                f"  data: {dirs['data'].as_posix()}",
                f"  downloads: {dirs['downloads'].as_posix()}",
                f"  user: {dirs['user'].as_posix()}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return config_path


def arduino_cli_path() -> Path:
    bundled = resource_path("arduino-cli.exe")
    if bundled.exists():
        return bundled
    candidate = Path(r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe")
    if candidate.exists():
        return candidate
    return Path("arduino-cli.exe")


def run_dashboard_mode() -> None:
    from streamlit.web import bootstrap

    script_path = str(resource_path("streamlit_app.py"))
    flag_options = {
        "server.headless": True,
        "server.port": 8501,
        "browser.gatherUsageStats": False,
        "server.fileWatcherType": "none",
    }
    bootstrap.run(script_path, False, [], flag_options)


class LauncherApp:
    def __init__(self) -> None:
        self.root = tb.Window(themename="darkly")
        self.root.title(APP_NAME)
        self.root.geometry("1160x760")
        self.root.minsize(1080, 700)
        icon_path = resource_path("branding/icon-app.ico")
        if icon_path.exists():
            self.root.iconbitmap(str(icon_path))

        self.log_queue: queue.Queue[str] = queue.Queue()
        self.dashboard_process: subprocess.Popen[str] | None = None
        self.brand_icon: tk.PhotoImage | None = None

        self.cli_path = arduino_cli_path()
        self.cli_config = arduino_cli_config_path()
        self.bundle_dir = bundle_root()
        self.sketch_dir = bundle_root()
        self.port_var = tb.StringVar()
        self.url_var = tb.StringVar(value=DEFAULT_DASHBOARD_URL)
        self.status_var = tb.StringVar(value="Ready")

        self._build_ui()
        self.refresh_ports()
        self.root.after(150, self._drain_logs)

    def _build_ui(self) -> None:
        self.root.configure(padx=18, pady=18)

        hero = tb.Frame(self.root, bootstyle="dark")
        hero.pack(fill=X, pady=(0, 14))

        header_card = tb.Labelframe(hero, text=" DUM-E Control Hub ", bootstyle="info")
        header_card.pack(fill=X)

        brand_row = tb.Frame(header_card)
        brand_row.pack(fill=X, padx=16, pady=16)

        icon_column = tb.Frame(brand_row)
        icon_column.pack(side=LEFT, anchor="n", padx=(0, 18))

        icon_png = resource_path("branding/icon-app.png")
        if icon_png.exists():
            self.brand_icon = tk.PhotoImage(file=str(icon_png))
            tb.Label(icon_column, image=self.brand_icon, bootstyle="dark").pack(anchor="n")

        copy_column = tb.Frame(brand_row)
        copy_column.pack(side=LEFT, fill=X, expand=True)

        tb.Label(
            copy_column,
            text="DUM-E",
            font=("Segoe UI Semibold", 28),
            bootstyle="light",
        ).pack(anchor="w")
        tb.Label(
            copy_column,
            text="Portable launcher for flashing firmware and opening the control dashboard.",
            font=("Segoe UI", 11),
            bootstyle="secondary",
        ).pack(anchor="w", pady=(4, 0))
        tb.Label(
            copy_column,
            text=f"Bundled Arduino CLI: {'yes' if self.cli_path.exists() else 'no'}  |  Portable data: {portable_root()}",
            font=("Segoe UI", 10),
            bootstyle="info",
        ).pack(anchor="w", pady=(10, 0))

        content = tb.Frame(self.root)
        content.pack(fill=BOTH, expand=True)

        left = tb.Frame(content)
        left.pack(side=LEFT, fill=BOTH, expand=True, padx=(0, 12))

        right = tb.Frame(content)
        right.pack(side=LEFT, fill=BOTH, expand=True)

        device_card = tb.Labelframe(left, text=" Device ", bootstyle="secondary")
        device_card.pack(fill=X, pady=(0, 12))

        row1 = tb.Frame(device_card)
        row1.pack(fill=X, padx=12, pady=(12, 8))
        tb.Label(row1, text="COM port", width=14, anchor="w").pack(side=LEFT)
        self.port_menu = tb.Combobox(row1, textvariable=self.port_var, state="readonly")
        self.port_menu.pack(side=LEFT, fill=X, expand=True, padx=(8, 8))
        tb.Button(row1, text="Refresh", command=self.refresh_ports, bootstyle="secondary-outline").pack(side=LEFT)

        row2 = tb.Frame(device_card)
        row2.pack(fill=X, padx=12, pady=(0, 12))
        tb.Label(row2, text="Dashboard URL", width=14, anchor="w").pack(side=LEFT)
        tb.Entry(row2, textvariable=self.url_var).pack(side=LEFT, fill=X, expand=True, padx=(8, 8))
        tb.Button(row2, text="Open", command=self.open_dashboard_url, bootstyle="info-outline").pack(side=LEFT)

        actions_card = tb.Labelframe(left, text=" Actions ", bootstyle="secondary")
        actions_card.pack(fill=X, pady=(0, 12))

        button_grid = tb.Frame(actions_card)
        button_grid.pack(fill=X, padx=12, pady=12)
        button_grid.grid_columnconfigure(0, weight=1)
        button_grid.grid_columnconfigure(1, weight=1)

        tb.Button(
            button_grid,
            text="Install / Repair Core",
            command=lambda: self.run_async(self.install_core),
            bootstyle="info",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=6)
        tb.Button(
            button_grid,
            text="Flash Firmware",
            command=lambda: self.run_async(self.flash_firmware),
            bootstyle="success",
        ).grid(row=0, column=1, sticky="ew", padx=(6, 0), pady=6)
        tb.Button(
            button_grid,
            text="Launch Dashboard",
            command=self.launch_dashboard,
            bootstyle="primary",
        ).grid(row=1, column=0, sticky="ew", padx=(0, 6), pady=6)
        tb.Button(
            button_grid,
            text="Stop Dashboard",
            command=self.stop_dashboard,
            bootstyle="danger-outline",
        ).grid(row=1, column=1, sticky="ew", padx=(6, 0), pady=6)

        status_card = tb.Labelframe(left, text=" Status ", bootstyle="secondary")
        status_card.pack(fill=X)
        tb.Label(
            status_card,
            textvariable=self.status_var,
            font=("Segoe UI Semibold", 12),
            bootstyle="light",
        ).pack(anchor="w", padx=12, pady=12)

        log_card = tb.Labelframe(right, text=" Activity Log ", bootstyle="secondary")
        log_card.pack(fill=BOTH, expand=True)
        self.log = ScrolledText(log_card, autohide=True, font=("Cascadia Code", 10), padding=10)
        self.log.pack(fill=BOTH, expand=True, padx=12, pady=12)
        self.log.text.configure(state="disabled")

    def append_log(self, message: str) -> None:
        self.log_queue.put(message.rstrip() + "\n")

    def _drain_logs(self) -> None:
        while not self.log_queue.empty():
            chunk = self.log_queue.get_nowait()
            self.log.text.configure(state="normal")
            self.log.text.insert(END, chunk)
            self.log.text.see(END)
            self.log.text.configure(state="disabled")
        self.root.after(150, self._drain_logs)

    def run_async(self, fn) -> None:
        thread = threading.Thread(target=fn, daemon=True)
        thread.start()

    def command_prefix(self) -> list[str]:
        return [str(self.cli_path), "--config-file", str(self.cli_config)]

    def run_command(self, args: list[str], description: str) -> bool:
        self.status_var.set(description)
        self.append_log(f"\n== {description} ==")
        self.append_log(" ".join(args))
        env = os.environ.copy()
        env["PYTHONUTF8"] = "1"
        process = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=str(self.sketch_dir),
            env=env,
        )
        assert process.stdout is not None
        for line in process.stdout:
            self.append_log(line.rstrip())
        return_code = process.wait()
        if return_code == 0:
            self.append_log(f"{description} completed successfully.")
            self.status_var.set("Ready")
            return True
        self.append_log(f"{description} failed with exit code {return_code}.")
        self.status_var.set("Failed")
        return False

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_menu["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")
        self.append_log("Ports: " + (", ".join(ports) if ports else "none found"))

    def install_core(self) -> None:
        prefix = self.command_prefix()
        self.run_command(prefix + ["core", "update-index"], "Updating Arduino indexes")
        self.run_command(prefix + ["core", "install", "esp32-bluepad32:esp32@4.1.0"], "Installing ESP32 + Bluepad32 core")

    def flash_firmware(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            self.append_log("No COM port selected.")
            self.status_var.set("Choose a COM port")
            return

        prefix = self.command_prefix()
        compile_ok = self.run_command(prefix + ["compile", "--fqbn", BOARD_FQBN, str(self.sketch_dir)], "Compiling firmware")
        if not compile_ok:
            return
        self.run_command(prefix + ["upload", "-p", port, "--fqbn", BOARD_FQBN, str(self.sketch_dir)], "Uploading firmware")

    def launch_dashboard(self) -> None:
        if self.dashboard_process and self.dashboard_process.poll() is None:
            self.append_log("Dashboard already running.")
            webbrowser.open(self.url_var.get().strip())
            return

        self.status_var.set("Starting dashboard")
        args = [sys.executable, "--run-dashboard"] if is_frozen() else [sys.executable, str(resource_path("launcher.py")), "--run-dashboard"]
        self.dashboard_process = subprocess.Popen(
            args,
            cwd=str(self.sketch_dir),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.append_log("Dashboard started.")

        def open_later() -> None:
            time.sleep(3)
            webbrowser.open(self.url_var.get().strip())
            self.status_var.set("Ready")

        threading.Thread(target=open_later, daemon=True).start()

    def stop_dashboard(self) -> None:
        if self.dashboard_process and self.dashboard_process.poll() is None:
            self.dashboard_process.terminate()
            self.append_log("Dashboard stopped.")
        else:
            self.append_log("Dashboard is not running.")
        self.status_var.set("Ready")

    def open_dashboard_url(self) -> None:
        webbrowser.open(self.url_var.get().strip())

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    if "--run-dashboard" in sys.argv:
        run_dashboard_mode()
        return

    app = LauncherApp()
    app.run()


if __name__ == "__main__":
    main()
