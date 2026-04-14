from __future__ import annotations

import os
import queue
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path
from tkinter import BOTH, END, LEFT, Button, Entry, Frame, Label, PhotoImage, StringVar, Tk, ttk
from tkinter.scrolledtext import ScrolledText

from serial.tools import list_ports


APP_NAME = "DUM-E Launcher"
BOARD_FQBN = "esp32-bluepad32:esp32:esp32"
BLUEPAD_URL = "https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json"
DEFAULT_DASHBOARD_URL = "http://localhost:8501"
BG = "#0E1116"
PANEL = "#141B24"
PANEL_ALT = "#18212C"
LINE = "#273243"
INK = "#FFF6EF"
MUTED = "#A2AFBF"


def is_frozen() -> bool:
    return getattr(sys, "frozen", False)


def app_root() -> Path:
    if is_frozen():
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


def resource_path(name: str) -> Path:
    return app_root() / name


def arduino_cli_path() -> Path:
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
        self.root = Tk()
        self.root.title(APP_NAME)
        self.root.geometry("900x640")
        self.root.minsize(840, 560)
        self.root.configure(bg=BG)

        self.log_queue: queue.Queue[str] = queue.Queue()
        self.dashboard_process: subprocess.Popen[str] | None = None
        self.brand_image: PhotoImage | None = None

        self.cli_path = arduino_cli_path()
        self.sketch_dir = app_root()
        self.port_var = StringVar()
        self.url_var = StringVar(value=DEFAULT_DASHBOARD_URL)
        self.status_var = StringVar(value="Ready")

        self._load_brand_assets()
        self._build_ui()
        self.refresh_ports()
        self.root.after(150, self._drain_logs)

    def _load_brand_assets(self) -> None:
        png_path = resource_path("branding/icon-app.png")
        ico_path = resource_path("branding/icon-app.ico")
        if png_path.exists():
            self.brand_image = PhotoImage(file=str(png_path))
            self.root.iconphoto(True, self.brand_image)
        elif ico_path.exists():
            try:
                self.root.iconbitmap(default=str(ico_path))
            except Exception:
                pass

    def _build_ui(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("Dume.TFrame", background=BG)
        style.configure("DumePanel.TFrame", background=PANEL)
        style.configure("Dume.TLabel", background=BG, foreground=INK, font=("Segoe UI", 10))
        style.configure("DumeMuted.TLabel", background=BG, foreground=MUTED, font=("Segoe UI", 10))
        style.configure("DumeTitle.TLabel", background=BG, foreground=INK, font=("Segoe UI", 20, "bold"))
        style.configure("Dume.TButton", font=("Segoe UI", 10, "bold"))
        style.configure("Dume.TCombobox", fieldbackground=PANEL_ALT, background=PANEL_ALT, foreground=INK)

        top = ttk.Frame(self.root, padding=12, style="Dume.TFrame")
        top.pack(fill=BOTH)

        brand_row = ttk.Frame(top, style="Dume.TFrame")
        brand_row.pack(fill=BOTH)
        if self.brand_image is not None:
            Label(brand_row, image=self.brand_image, bg=BG).pack(side=LEFT, padx=(0, 10))
        ttk.Label(brand_row, text="DUM-E", style="DumeTitle.TLabel").pack(anchor="w")
        ttk.Label(
            top,
            text="Flash the ESP32, launch the dashboard, and manage setup from one place.",
            style="DumeMuted.TLabel",
        ).pack(anchor="w", pady=(2, 0))

        grid = ttk.Frame(self.root, padding=(12, 8), style="Dume.TFrame")
        grid.pack(fill=BOTH)

        ttk.Label(grid, text="COM port", style="Dume.TLabel").grid(row=0, column=0, sticky="w")
        self.port_menu = ttk.Combobox(grid, textvariable=self.port_var, state="readonly", width=24, style="Dume.TCombobox")
        self.port_menu.grid(row=0, column=1, sticky="we", padx=(8, 8))
        ttk.Button(grid, text="Refresh ports", command=self.refresh_ports, style="Dume.TButton").grid(row=0, column=2, sticky="we")

        ttk.Label(grid, text="Dashboard URL", style="Dume.TLabel").grid(row=1, column=0, sticky="w", pady=(8, 0))
        Entry(grid, textvariable=self.url_var, width=28, bg=PANEL_ALT, fg=INK, insertbackground=INK, relief="flat").grid(row=1, column=1, sticky="we", padx=(8, 8), pady=(8, 0))
        ttk.Button(grid, text="Open URL", command=self.open_dashboard_url, style="Dume.TButton").grid(row=1, column=2, sticky="we", pady=(8, 0))

        grid.grid_columnconfigure(1, weight=1)

        actions = ttk.Frame(self.root, padding=(12, 8), style="Dume.TFrame")
        actions.pack(fill=BOTH)

        ttk.Button(actions, text="Install / Repair ESP32 + Bluepad32 Core", command=lambda: self.run_async(self.install_core), style="Dume.TButton").pack(fill=BOTH, pady=4)
        ttk.Button(actions, text="Flash ESP32 Firmware", command=lambda: self.run_async(self.flash_firmware), style="Dume.TButton").pack(fill=BOTH, pady=4)
        ttk.Button(actions, text="Launch Dashboard", command=self.launch_dashboard, style="Dume.TButton").pack(fill=BOTH, pady=4)
        ttk.Button(actions, text="Stop Dashboard", command=self.stop_dashboard, style="Dume.TButton").pack(fill=BOTH, pady=4)

        status_row = ttk.Frame(self.root, padding=(12, 4), style="Dume.TFrame")
        status_row.pack(fill=BOTH)
        ttk.Label(status_row, text="Status:", style="Dume.TLabel").pack(side=LEFT)
        ttk.Label(status_row, textvariable=self.status_var, style="DumeMuted.TLabel").pack(side=LEFT)

        self.log = ScrolledText(self.root, wrap="word", height=20, bg=PANEL, fg=INK, insertbackground=INK, relief="flat", font=("Consolas", 10))
        self.log.pack(fill=BOTH, expand=True, padx=12, pady=12)
        self.log.configure(state="disabled")

    def append_log(self, message: str) -> None:
        self.log_queue.put(message.rstrip() + "\n")

    def _drain_logs(self) -> None:
        while not self.log_queue.empty():
            chunk = self.log_queue.get_nowait()
            self.log.configure(state="normal")
            self.log.insert(END, chunk)
            self.log.see(END)
            self.log.configure(state="disabled")
        self.root.after(150, self._drain_logs)

    def run_async(self, fn) -> None:
        thread = threading.Thread(target=fn, daemon=True)
        thread.start()

    def run_command(self, args: list[str], description: str) -> bool:
        self.status_var.set(description)
        self.append_log(f"\n== {description} ==")
        self.append_log(" ".join(args))
        process = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=str(self.sketch_dir),
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
        self.append_log("Refreshed COM ports: " + (", ".join(ports) if ports else "none found"))

    def install_core(self) -> None:
        cli = str(self.cli_path)
        self.run_command([cli, "core", "update-index", "--additional-urls", BLUEPAD_URL], "Updating Arduino core index")
        self.run_command([cli, "core", "install", "esp32-bluepad32:esp32@4.1.0", "--additional-urls", BLUEPAD_URL], "Installing ESP32 + Bluepad32 core")

    def flash_firmware(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            self.append_log("No COM port selected.")
            self.status_var.set("Choose a COM port")
            return

        cli = str(self.cli_path)
        compile_ok = self.run_command([cli, "compile", "--fqbn", BOARD_FQBN, str(self.sketch_dir)], "Compiling firmware")
        if not compile_ok:
            return
        self.run_command([cli, "upload", "-p", port, "--fqbn", BOARD_FQBN, str(self.sketch_dir)], "Uploading firmware")

    def launch_dashboard(self) -> None:
        if self.dashboard_process and self.dashboard_process.poll() is None:
            self.append_log("Dashboard is already running.")
            webbrowser.open(self.url_var.get().strip())
            return

        self.status_var.set("Starting dashboard")
        if is_frozen():
            args = [sys.executable, "--run-dashboard"]
        else:
            args = [sys.executable, str(resource_path("launcher.py")), "--run-dashboard"]

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
