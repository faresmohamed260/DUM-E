from __future__ import annotations

import ipaddress
import json
import socket
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List
from urllib.parse import urlparse

import requests
import streamlit as st


DEFAULT_DEVICE_URL = "http://192.168.4.1"
DEFAULT_MDNS_HOSTNAME = "robot-arm.local"
IDENTIFY_PATH = "/api/identify"
CONTROL_MODE_OPTIONS = [("none", 0), ("axis", 1), ("buttons", 2)]
AXIS_SOURCE_OPTIONS = [
    ("none", 0),
    ("left_stick_x", 1),
    ("left_stick_y", 2),
    ("right_stick_x", 3),
    ("right_stick_y", 4),
    ("dpad_x", 5),
    ("dpad_y", 6),
    ("triggers", 7),
]
BUTTON_OPTIONS = [
    ("none", 0),
    ("up", 1),
    ("down", 2),
    ("left", 3),
    ("right", 4),
    ("square", 5),
    ("cross", 6),
    ("circle", 7),
    ("triangle", 8),
    ("l1", 9),
    ("r1", 10),
    ("l2", 11),
    ("r2", 12),
    ("share", 13),
    ("options", 14),
    ("l3", 15),
    ("r3", 16),
    ("ps", 17),
    ("touchpad", 18),
]
MOTOR_TYPE_OPTIONS = [("positional_180", 0), ("continuous_360", 1)]
APP_ROOT = Path(__file__).resolve().parent
BRANDING_DIR = APP_ROOT / "branding"
DEVICE_CACHE_PATH = APP_ROOT / "device_cache.json"
DISCOVERY_TIMEOUT = 0.45
DISCOVERY_WORKERS = 32


@dataclass
class JointState:
    name: str
    coordinate_space: str
    pin: int
    motor_type: str
    min_angle: int
    max_angle: int
    home_angle: int
    step: int
    pulse_min: int
    pulse_max: int
    neutral_output: int
    stop_deadband: int
    max_speed_scale: int
    invert: bool
    position: int
    startup_target: int
    raw_output: int
    stored_min_angle: int
    stored_max_angle: int
    stored_home_angle: int
    stored_position: int
    attached: bool
    velocity: int
    control_mode: str
    axis_source: str
    positive_button: str
    negative_button: str
    input_invert: bool


@dataclass
class ControllerState:
    enabled: bool
    allow_new_connections: bool
    state: str
    status_text: str
    last_error: str
    scanning_in_progress: bool
    reconnect_in_progress: bool
    connected: bool
    esp32_bt_mac: str
    controller_name: str
    controller_type: str
    controller_bt_addr: str
    remembered_name: str
    remembered_type: str
    remembered_bt_addr: str
    led_r: int
    led_g: int
    led_b: int
    rumble_force: int
    rumble_duration: int
    axis_deadzone: int
    axis_center_lx: int
    axis_center_ly: int
    axis_center_rx: int
    axis_center_ry: int
    home_all_button: str
    battery: int
    battery_raw: int


@dataclass
class WifiState:
    hostname: str
    mdns_hostname: str
    mdns_active: bool
    ap_active: bool
    ap_ssid: str
    ap_ip: str
    sta_ssid: str
    sta_connected: bool
    sta_ip: str
    sta_status: str
    last_result: str
    last_failure: str


@dataclass
class DeviceInfo:
    base_url: str
    hostname: str
    mdns_hostname: str
    ip_address: str
    ap_ip: str
    mac: str
    firmware_version: str
    device_model: str


def option_index(options: List[tuple[str, int]], value: str) -> int:
    for index, (name, _) in enumerate(options):
        if name == value:
            return index
    return 0


def api_get(base_url: str, path: str, params: dict | None = None) -> dict:
    response = requests.get(f"{base_url.rstrip('/')}{path}", params=params, timeout=8)
    response.raise_for_status()
    payload = response.json()
    if not payload.get("ok", True):
        raise RuntimeError(payload.get("error", "Request failed"))
    return payload


def asset_text(name: str) -> str:
    return (BRANDING_DIR / name).read_text(encoding="utf-8")


def render_brand_header() -> None:
    st.image(
        str(BRANDING_DIR / "logo-horizontal.svg"),
        use_container_width=True,
    )


def render_sidebar_brand() -> None:
    st.sidebar.image(
        str(BRANDING_DIR / "logo-mark.svg"),
        use_container_width=True,
    )


def parse_joint_state(payload: dict) -> JointState:
    return JointState(
        name=payload["name"],
        coordinate_space=str(payload.get("coordinate_space", "")),
        pin=int(payload["pin"]),
        motor_type=str(payload["motor_type"]),
        min_angle=int(payload["min_angle"]),
        max_angle=int(payload["max_angle"]),
        home_angle=int(payload["home_angle"]),
        step=int(payload["step"]),
        pulse_min=int(payload["pulse_min"]),
        pulse_max=int(payload["pulse_max"]),
        neutral_output=int(payload.get("neutral_output", 90)),
        stop_deadband=int(payload.get("stop_deadband", 3)),
        max_speed_scale=int(payload.get("max_speed_scale", 100)),
        invert=bool(payload["invert"]),
        position=int(payload["position"]),
        startup_target=int(payload.get("startup_target", payload["home_angle"])),
        raw_output=int(payload.get("raw_output", payload["position"])),
        stored_min_angle=int(payload.get("stored_min_angle", payload["min_angle"])),
        stored_max_angle=int(payload.get("stored_max_angle", payload["max_angle"])),
        stored_home_angle=int(payload.get("stored_home_angle", payload["home_angle"])),
        stored_position=int(payload.get("stored_position", payload["position"])),
        attached=bool(payload["attached"]),
        velocity=int(payload["velocity"]),
        control_mode=str(payload["control_mode"]),
        axis_source=str(payload["axis_source"]),
        positive_button=str(payload["positive_button"]),
        negative_button=str(payload["negative_button"]),
        input_invert=bool(payload["input_invert"]),
    )


def parse_controller_state(payload: dict) -> ControllerState:
    return ControllerState(
        enabled=bool(payload["enabled"]),
        allow_new_connections=bool(payload["allow_new_connections"]),
        state=str(payload.get("state", "idle")),
        status_text=str(payload.get("status_text", "")),
        last_error=str(payload.get("last_error", "")),
        scanning_in_progress=bool(payload.get("scanning_in_progress", False)),
        reconnect_in_progress=bool(payload.get("reconnect_in_progress", False)),
        connected=bool(payload["connected"]),
        esp32_bt_mac=str(payload["esp32_bt_mac"]),
        controller_name=str(payload["controller_name"]),
        controller_type=str(payload["controller_type"]),
        controller_bt_addr=str(payload["controller_bt_addr"]),
        remembered_name=str(payload.get("remembered_name", "")),
        remembered_type=str(payload.get("remembered_type", "")),
        remembered_bt_addr=str(payload.get("remembered_bt_addr", "")),
        led_r=int(payload["led_r"]),
        led_g=int(payload["led_g"]),
        led_b=int(payload["led_b"]),
        rumble_force=int(payload["rumble_force"]),
        rumble_duration=int(payload["rumble_duration"]),
        axis_deadzone=int(payload["axis_deadzone"]),
        axis_center_lx=int(payload["axis_center_lx"]),
        axis_center_ly=int(payload["axis_center_ly"]),
        axis_center_rx=int(payload["axis_center_rx"]),
        axis_center_ry=int(payload["axis_center_ry"]),
        home_all_button=str(payload["home_all_button"]),
        battery=int(payload["battery"]),
        battery_raw=int(payload["battery_raw"]),
    )


def parse_wifi_state(payload: dict) -> WifiState:
    return WifiState(
        hostname=str(payload.get("hostname", "")),
        mdns_hostname=str(payload.get("mdns_hostname", "")),
        mdns_active=bool(payload.get("mdns_active", False)),
        ap_active=bool(payload.get("ap_active", True)),
        ap_ssid=str(payload.get("ap_ssid", "")),
        ap_ip=str(payload.get("ap_ip", "")),
        sta_ssid=str(payload.get("sta_ssid", "")),
        sta_connected=bool(payload.get("sta_connected", False)),
        sta_ip=str(payload.get("sta_ip", "")),
        sta_status=str(payload.get("sta_status", "unknown")),
        last_result=str(payload.get("last_result", "")),
        last_failure=str(payload.get("last_failure", "")),
    )


def fetch_state(base_url: str) -> dict:
    payload = api_get(base_url, "/api/state")
    joints = {joint["name"]: parse_joint_state(joint) for joint in payload["joints"]}
    return {
        "controller": parse_controller_state(payload["ps4"]),
        "wifi": parse_wifi_state(payload["wifi"]),
        "joints": joints,
    }


def sync_state(base_url: str) -> None:
    st.session_state.robot_state = fetch_state(base_url)


def normalize_base_url(value: str) -> str:
    candidate = value.strip()
    if not candidate:
        return ""
    if "://" not in candidate:
        candidate = f"http://{candidate}"
    return candidate.rstrip("/")


def local_cache() -> dict:
    if DEVICE_CACHE_PATH.exists():
        try:
            return json.loads(DEVICE_CACHE_PATH.read_text(encoding="utf-8"))
        except Exception:
            return {}
    return {}


def save_local_cache(data: dict) -> None:
    DEVICE_CACHE_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")


def parse_device_info(base_url: str, payload: dict) -> DeviceInfo:
    return DeviceInfo(
        base_url=normalize_base_url(base_url),
        hostname=str(payload.get("hostname", "")),
        mdns_hostname=str(payload.get("mdns_hostname", "")),
        ip_address=str(payload.get("ip_address", "")),
        ap_ip=str(payload.get("ap_ip", "")),
        mac=str(payload.get("mac", "")),
        firmware_version=str(payload.get("firmware_version", "")),
        device_model=str(payload.get("device_model", "")),
    )


def probe_device(base_url: str, timeout: float = DISCOVERY_TIMEOUT) -> DeviceInfo | None:
    candidate = normalize_base_url(base_url)
    if not candidate:
        return None
    try:
        response = requests.get(f"{candidate}{IDENTIFY_PATH}", timeout=timeout)
        response.raise_for_status()
        payload = response.json()
        if not payload.get("ok", True):
            return None
        if payload.get("device_type") != "robot_arm":
            return None
        return parse_device_info(candidate, payload)
    except Exception:
        return None


def cache_successful_device(base_url: str, device: DeviceInfo | None = None) -> None:
    cache = local_cache()
    cache["last_success_url"] = normalize_base_url(base_url)
    if device is not None:
        cache["last_hostname"] = device.hostname
        cache["last_mdns_hostname"] = device.mdns_hostname
        cache["last_ip"] = device.ip_address
    save_local_cache(cache)


def local_ipv4_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                addresses.add(ip)
    except Exception:
        pass
    return sorted(addresses)


def probable_subnets() -> list[ipaddress.IPv4Network]:
    subnets: list[ipaddress.IPv4Network] = []
    cache = local_cache()
    candidates = local_ipv4_addresses()
    if cache.get("last_ip"):
        candidates.insert(0, str(cache["last_ip"]))
    seen: set[str] = set()
    for ip in candidates:
        try:
            network = ipaddress.ip_network(f"{ip}/24", strict=False)
        except ValueError:
            continue
        if network.network_address.is_loopback:
            continue
        key = str(network)
        if key not in seen:
            seen.add(key)
            subnets.append(network)
    return subnets


def dedupe_devices(devices: list[DeviceInfo]) -> list[DeviceInfo]:
    deduped: dict[str, DeviceInfo] = {}
    for device in devices:
        deduped[device.base_url] = device
    return list(deduped.values())


def discovery_candidates() -> list[str]:
    cache = local_cache()
    candidates: list[str] = []
    if cache.get("last_success_url"):
        candidates.append(str(cache["last_success_url"]))
    for hostname in [cache.get("last_mdns_hostname"), cache.get("last_hostname"), DEFAULT_MDNS_HOSTNAME]:
        if hostname:
            host = str(hostname)
            if not host.endswith(".local") and "." not in host:
                host = f"{host}.local"
            candidates.append(f"http://{host}")
    seen: list[str] = []
    for item in candidates:
        normalized = normalize_base_url(item)
        if normalized and normalized not in seen:
            seen.append(normalized)
    return seen


def scan_subnet(network: ipaddress.IPv4Network) -> list[DeviceInfo]:
    devices: list[DeviceInfo] = []
    hosts = [str(host) for host in network.hosts()]
    with ThreadPoolExecutor(max_workers=DISCOVERY_WORKERS) as executor:
        futures = {
            executor.submit(probe_device, f"http://{host}", DISCOVERY_TIMEOUT): host
            for host in hosts
        }
        for future in as_completed(futures):
            device = future.result()
            if device is not None:
                devices.append(device)
    return devices


def discover_devices() -> tuple[list[DeviceInfo], list[str]]:
    results: list[DeviceInfo] = []
    log: list[str] = []
    for candidate in discovery_candidates():
        device = probe_device(candidate, timeout=1.0)
        if device is not None:
            results.append(device)
            log.append(f"Resolved {candidate}")
    if results:
        return dedupe_devices(results), log
    for network in probable_subnets():
        log.append(f"Scanning {network}")
        devices = scan_subnet(network)
        if devices:
            results.extend(devices)
            break
    return dedupe_devices(results), log


def connect_device(base_url: str, device: DeviceInfo | None = None) -> bool:
    target = normalize_base_url(base_url)
    try:
        sync_state(target)
        st.session_state.connected_base_url = target
        st.session_state.device_url = target
        st.session_state.pop("device_error", None)
        if device is None:
            device = probe_device(target, timeout=1.0)
        if device is not None:
            cache_successful_device(target, device)
        else:
            cache_successful_device(target)
        return True
    except Exception as exc:
        st.session_state.device_error = str(exc)
        return False


def auto_discovery_flow() -> None:
    signature = "|".join(local_ipv4_addresses())
    if st.session_state.get("discovery_signature") == signature and st.session_state.get("discovery_ran"):
        return
    st.session_state.discovery_signature = signature
    st.session_state.discovery_ran = True
    with st.spinner("Searching for DUM-E on your network..."):
        devices, log = discover_devices()
    st.session_state.discovery_log = log
    st.session_state.discovered_devices = devices
    if len(devices) == 1:
        connect_device(devices[0].base_url, devices[0])


def run_device_action(base_url: str, path: str, params: dict | None = None, refresh: bool = False, rerun: bool = False) -> bool:
    try:
        api_get(base_url, path, params=params)
        if refresh:
            sync_state(base_url)
        st.session_state.pop("device_error", None)
        if rerun:
            st.rerun()
        return True
    except Exception as exc:
        st.session_state.device_error = str(exc)
        return False


def device_label(device: DeviceInfo) -> str:
    mdns = device.mdns_hostname or "no-mdns"
    ip = device.ip_address or device.base_url
    return f"{device.device_model} | {mdns} | {ip}"


def render_connection_sidebar() -> str:
    cache = local_cache()
    st.sidebar.subheader("Connection")
    last_known = cache.get("last_success_url") or "none"
    st.sidebar.caption(f"Last successful device: `{last_known}`")

    if st.sidebar.button("Find DUM-E devices", use_container_width=True):
        with st.spinner("Searching for DUM-E devices..."):
            devices, log = discover_devices()
        st.session_state.discovered_devices = devices
        st.session_state.discovery_log = log
        if len(devices) == 1:
            connect_device(devices[0].base_url, devices[0])

    discovered_devices: list[DeviceInfo] = st.session_state.get("discovered_devices", [])
    if discovered_devices:
        if len(discovered_devices) == 1:
            device = discovered_devices[0]
            st.sidebar.success(f"Found `{device_label(device)}`")
        else:
            labels = [device_label(device) for device in discovered_devices]
            selected_label = st.sidebar.selectbox("Discovered devices", labels, key="device_picker")
            selected_device = discovered_devices[labels.index(selected_label)]
            if st.sidebar.button("Connect to selected device", use_container_width=True):
                connect_device(selected_device.base_url, selected_device)
    else:
        st.sidebar.info("Automatic discovery uses last known address, mDNS, then a local subnet scan.")

    if st.session_state.get("discovery_log"):
        with st.sidebar.expander("Discovery details"):
            for item in st.session_state["discovery_log"]:
                st.write(item)

    manual_default = st.session_state.get("device_url") or cache.get("last_success_url") or DEFAULT_DEVICE_URL
    with st.sidebar.expander("Manual URL override"):
        manual_url = st.text_input("Device URL", value=manual_default, key="manual_device_url").strip()
        st.caption("Use this only if automatic discovery fails or for advanced debugging.")
        if st.button("Connect manually", use_container_width=True):
            connect_device(manual_url)
    return normalize_base_url(manual_default)


def wifi_controls(base_url: str, wifi: WifiState) -> None:
    st.subheader("Wi-Fi")
    info_left, info_mid, info_right = st.columns(3)
    with info_left:
        st.write(f"AP SSID: `{wifi.ap_ssid}`")
        st.write(f"AP IP: `{wifi.ap_ip}`")
        st.write(f"AP active: `{wifi.ap_active}`")
    with info_mid:
        st.write(f"Station SSID: `{wifi.sta_ssid or 'not set'}`")
        st.write(f"Station connected: `{wifi.sta_connected}`")
        st.write(f"Station status: `{wifi.sta_status}`")
    with info_right:
        st.write(f"Station IP: `{wifi.sta_ip or 'not connected'}`")
        st.write(f"Hostname: `{wifi.hostname}`")
        st.write(f"mDNS: `{wifi.mdns_hostname or 'inactive'}`")

    status_left, status_mid = st.columns(2)
    with status_left:
        st.caption(f"Last Wi-Fi result: `{wifi.last_result or 'unknown'}`")
    with status_mid:
        st.caption(f"Last Wi-Fi failure: `{wifi.last_failure or 'none'}`")

    form_left, form_mid = st.columns(2)
    with form_left:
        hostname = st.text_input("Hostname", value=wifi.hostname, key="wifi_hostname")
        ap_ssid = st.text_input("AP SSID", value=wifi.ap_ssid, key="wifi_ap_ssid")
        ap_password = st.text_input("AP password", value="robotarm123", type="password", key="wifi_ap_password")
    with form_mid:
        sta_ssid = st.text_input("Station SSID", value=wifi.sta_ssid, key="wifi_sta_ssid")
        sta_password = st.text_input("Station password", value="", type="password", key="wifi_sta_password")

    button_left, button_mid, button_right = st.columns(3)
    with button_left:
        if st.button("Save Wi-Fi settings", use_container_width=True):
            run_device_action(
                base_url,
                "/api/wifi",
                params={
                    "cmd": "set",
                    "hostname": hostname,
                    "ap_ssid": ap_ssid,
                    "ap_password": ap_password,
                    "sta_ssid": sta_ssid,
                    "sta_password": sta_password,
                },
                refresh=True,
                rerun=True,
            )
    with button_mid:
        if st.button("Reconnect Wi-Fi", use_container_width=True):
            run_device_action(base_url, "/api/wifi", params={"cmd": "reconnect"}, refresh=True, rerun=True)
    with button_right:
        if st.button("Reboot ESP32", use_container_width=True):
            run_device_action(base_url, "/api/system", params={"cmd": "reboot"})
            st.info("Reboot requested. Wait a few seconds, then refresh.")


def controller_controls(base_url: str, controller: ControllerState) -> None:
    st.subheader("Wireless Controller")
    status_left, status_mid, status_right = st.columns(3)
    with status_left:
        st.write(f"Subsystem: `{controller.state}`")
        st.write(f"ESP32 Bluetooth MAC: `{controller.esp32_bt_mac}`")
    with status_mid:
        st.write(f"Connected: `{controller.connected}`")
        st.write(f"Battery: `{controller.battery}%`")
    with status_right:
        st.write(f"Pairing mode: `{controller.allow_new_connections}`")
        st.write(f"Reconnect active: `{controller.reconnect_in_progress}`")

    if controller.status_text:
        st.info(controller.status_text)
    if controller.last_error:
        st.warning(f"Last controller issue: `{controller.last_error}`")

    subsystem_left, subsystem_mid = st.columns(2)
    with subsystem_left:
        enabled = st.toggle(
            "Controller input enabled",
            value=controller.enabled,
            help="Disable this while tuning joints from the UI to avoid conflicting commands.",
        )
        if enabled != controller.enabled:
            run_device_action(base_url, "/api/ps4", params={"cmd": "enable", "value": 1 if enabled else 0}, refresh=True, rerun=True)
    with subsystem_mid:
        pair_mode = st.toggle(
            "Pairing / discovery mode",
            value=controller.allow_new_connections,
            help="Turn this on for first-time pairing, then hold SHARE + PS on the controller until it flashes quickly.",
        )
        if pair_mode != controller.allow_new_connections:
            run_device_action(base_url, "/api/ps4", params={"cmd": "pair_mode", "value": 1 if pair_mode else 0}, refresh=True, rerun=True)

    current_left, remembered_right = st.columns(2)
    with current_left:
        st.markdown("**Current controller**")
        st.write(f"Name: `{controller.controller_name or 'none'}`")
        st.write(f"Type: `{controller.controller_type or 'unknown'}`")
        st.write(f"Address: `{controller.controller_bt_addr or 'n/a'}`")
    with remembered_right:
        st.markdown("**Remembered controller**")
        st.write(f"Name: `{controller.remembered_name or 'none'}`")
        st.write(f"Type: `{controller.remembered_type or 'unknown'}`")
        st.write(f"Address: `{controller.remembered_bt_addr or 'n/a'}`")

    pair_left, pair_mid = st.columns(2)
    with pair_left:
        if st.button("Remember current controller", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "remember_current"}, refresh=True, rerun=True)
    with pair_mid:
        if controller.allow_new_connections:
            st.caption("Pairing mode is active. Put the controller in pairing mode now and the first successful connection will become the remembered target.")
        elif controller.remembered_bt_addr:
            st.caption("Pairing mode is off. Turning the remembered controller on should reconnect it automatically.")
        else:
            st.caption("No remembered controller yet. Enable pairing mode to bond one.")

    home_left, home_mid = st.columns(2)
    with home_left:
        home_all_button = st.selectbox(
            "Home all motors button",
            BUTTON_OPTIONS,
            index=option_index(BUTTON_OPTIONS, controller.home_all_button),
            key="controller_home_all_button",
            format_func=lambda item: item[0],
        )
        if st.button("Apply home-all button", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "home_button", "value": home_all_button[1]}, refresh=True, rerun=True)
    with home_mid:
        st.write(f"Configured home-all button: `{controller.home_all_button}`")
        st.caption("Pressing this button on the controller will send every joint to its configured home angle once per button press.")

    feedback_left, feedback_mid = st.columns(2)
    with feedback_left:
        st.write("Controller feedback")
        led_r = st.slider("LED red", 0, 255, controller.led_r)
        led_g = st.slider("LED green", 0, 255, controller.led_g)
        led_b = st.slider("LED blue", 0, 255, controller.led_b)
        if st.button("Apply LED color", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "led", "r": led_r, "g": led_g, "b": led_b}, refresh=True, rerun=True)
    with feedback_mid:
        rumble_force = st.slider("Rumble force", 0, 255, controller.rumble_force)
        rumble_duration = st.slider("Rumble duration", 0, 255, controller.rumble_duration)
        if st.button("Apply rumble", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "rumble", "force": rumble_force, "duration": rumble_duration}, refresh=True, rerun=True)

    calibration_left, calibration_mid = st.columns(2)
    with calibration_left:
        axis_deadzone = st.slider("Stick deadzone", 0, 200, controller.axis_deadzone)
        if st.button("Apply deadzone", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "deadzone", "value": axis_deadzone}, refresh=True, rerun=True)
    with calibration_mid:
        st.write(
            f"Centers: LX `{controller.axis_center_lx}` | LY `{controller.axis_center_ly}` | "
            f"RX `{controller.axis_center_rx}` | RY `{controller.axis_center_ry}`"
        )
        if st.button("Capture stick centers", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "calibrate_center"}, refresh=True, rerun=True)

    with st.expander("Advanced recovery"):
        recovery_left, recovery_mid, recovery_right = st.columns(3)
        with recovery_left:
            if st.button("Disconnect current controller", use_container_width=True):
                run_device_action(base_url, "/api/ps4", params={"cmd": "disconnect"}, refresh=True, rerun=True)
        with recovery_mid:
            if st.button("Forget remembered controller", use_container_width=True):
                run_device_action(base_url, "/api/ps4", params={"cmd": "forget_target"}, refresh=True, rerun=True)
        with recovery_right:
            if st.button("Forget all controller bond data", use_container_width=True):
                run_device_action(base_url, "/api/ps4", params={"cmd": "forget"}, refresh=True, rerun=True)
        st.caption("Use recovery actions only if normal reconnect fails. Bluepad32 handles reconnection from stored Bluetooth keys when pairing mode is off.")


def joint_controls(base_url: str, joint: JointState) -> None:
    with st.expander(joint.name.replace("_", " ").title(), expanded=True):
        is_continuous = joint.motor_type == "continuous_360"

        def clamp(value: int, low: int, high: int) -> int:
            return max(low, min(high, int(value)))

        command_label = "Current speed command" if is_continuous else "Current logical angle"
        move_label = "Speed command (%)" if is_continuous else "Logical angle target"
        min_label = "Min logical angle"
        max_label = "Max logical angle"
        home_label = "Home logical angle"
        space_label = "speed percent" if is_continuous else "logical servo-angle degrees"
        home_button_label = "Stop" if is_continuous else "Home"

        left, right, actions = st.columns([2, 2, 1])

        with right:
            pin = st.number_input("Pin", min_value=0, max_value=255, value=joint.pin, key=f"{joint.name}_pin")
            motor_type = st.toggle("Continuous 360 motor", value=is_continuous, key=f"{joint.name}_motor_type")

        range_min = -100 if motor_type else 0
        range_max = 100 if motor_type else 180

        safe_min = clamp(joint.min_angle, range_min, range_max)
        safe_max = clamp(joint.max_angle, range_min, range_max)

        if safe_min > safe_max:
            safe_min, safe_max = range_min, range_max

        safe_home = clamp(joint.home_angle, safe_min, safe_max)
        safe_position = clamp(joint.position, safe_min, safe_max)
        safe_step = max(1, int(joint.step))

        with left:
            st.write(
                f"{command_label}: `{joint.position}` | Startup/home target: `{joint.startup_target}` | "
                f"Raw servo output: `{joint.raw_output}` | Attached: `{joint.attached}`"
            )
            st.caption(
                f"Coordinate space: `{space_label}`. Continuous motors use signed speed with neutral stop calibration; positional joints use logical angles."
            )
            move_angle = st.slider(
                f"{joint.name}_move",
                min_value=safe_min,
                max_value=safe_max,
                value=safe_position,
                key=f"{joint.name}_move_slider",
                help=move_label,
                label_visibility="collapsed",
            )
            nudge_amount = st.number_input(
                f"{joint.name}_nudge",
                min_value=-100 if motor_type else -180,
                max_value=100 if motor_type else 180,
                value=safe_step,
                key=f"{joint.name}_nudge_value",
                step=1,
                label_visibility="collapsed",
            )
            if is_continuous:
                test_left, test_mid, test_right = st.columns(3)
                with test_left:
                    if st.button("Reverse", key=f"{joint.name}_reverse_btn", use_container_width=True):
                        run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": -100}, refresh=True, rerun=True)
                with test_mid:
                    if st.button("Stop", key=f"{joint.name}_stop_btn", use_container_width=True):
                        run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": 0}, refresh=True, rerun=True)
                with test_right:
                    if st.button("Forward", key=f"{joint.name}_forward_btn", use_container_width=True):
                        run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": 100}, refresh=True, rerun=True)

        with right:
            if is_continuous:
                st.caption("Continuous motor calibration")
                neutral_output = st.number_input("Neutral output", min_value=0, max_value=180, value=joint.neutral_output, key=f"{joint.name}_neutral_output")
                stop_deadband = st.number_input("Stop deadband", min_value=0, max_value=20, value=joint.stop_deadband, key=f"{joint.name}_stop_deadband")
                max_speed_scale = st.number_input("Max speed scale (%)", min_value=1, max_value=100, value=joint.max_speed_scale, key=f"{joint.name}_max_speed_scale")
                step = st.number_input("Button speed step", min_value=1, max_value=30, value=safe_step, key=f"{joint.name}_step")
                min_angle = -100
                max_angle = 100
                home_angle = 0
            else:
                min_angle = st.number_input(
                    min_label,
                    min_value=range_min,
                    max_value=range_max,
                    value=safe_min,
                    key=f"{joint.name}_min",
                )
                max_angle = st.number_input(
                    max_label,
                    min_value=range_min,
                    max_value=range_max,
                    value=safe_max,
                    key=f"{joint.name}_max",
                )

                if min_angle > max_angle:
                    st.warning("Min cannot be greater than max.")
                    max_angle = min_angle

                home_angle = st.number_input(
                    home_label,
                    min_value=int(min_angle),
                    max_value=int(max_angle),
                    value=clamp(safe_home, int(min_angle), int(max_angle)),
                    key=f"{joint.name}_home",
                )
                step = st.number_input("Step", min_value=1, max_value=30, value=safe_step, key=f"{joint.name}_step")
                neutral_output = joint.neutral_output
                stop_deadband = joint.stop_deadband
                max_speed_scale = joint.max_speed_scale
            pulse_min = st.number_input("Pulse min", min_value=100, max_value=3000, value=joint.pulse_min, key=f"{joint.name}_pulse_min")
            pulse_max = st.number_input("Pulse max", min_value=200, max_value=4000, value=joint.pulse_max, key=f"{joint.name}_pulse_max")
            invert = st.checkbox("Invert", value=joint.invert, key=f"{joint.name}_invert")
            if is_continuous:
                st.caption(
                    f"Stored on device: neutral `{joint.neutral_output}` | deadband `{joint.stop_deadband}` | "
                    f"max scale `{joint.max_speed_scale}%` | last speed `{joint.stored_position}`"
                )
            else:
                st.caption(
                    f"Stored on device: min `{joint.stored_min_angle}` | max `{joint.stored_max_angle}` | "
                    f"home `{joint.stored_home_angle}` | last position `{joint.stored_position}`"
                )
            control_mode = st.selectbox(
                "Controller input mode",
                CONTROL_MODE_OPTIONS,
                index=option_index(CONTROL_MODE_OPTIONS, joint.control_mode),
                key=f"{joint.name}_control_mode",
                format_func=lambda item: item[0],
            )
            axis_source = st.selectbox(
                "Axis source",
                AXIS_SOURCE_OPTIONS,
                index=option_index(AXIS_SOURCE_OPTIONS, joint.axis_source),
                key=f"{joint.name}_axis_source",
                format_func=lambda item: item[0],
            )
            positive_button = st.selectbox(
                "Positive button",
                BUTTON_OPTIONS,
                index=option_index(BUTTON_OPTIONS, joint.positive_button),
                key=f"{joint.name}_positive_button",
                format_func=lambda item: item[0],
            )
            negative_button = st.selectbox(
                "Negative button",
                BUTTON_OPTIONS,
                index=option_index(BUTTON_OPTIONS, joint.negative_button),
                key=f"{joint.name}_negative_button",
                format_func=lambda item: item[0],
            )
            input_invert = st.checkbox("Invert controller input", value=joint.input_invert, key=f"{joint.name}_input_invert")

        with actions:
            if st.button("Move", key=f"{joint.name}_move_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "move", "joint": joint.name, "value": move_angle}, refresh=True, rerun=True)

            if st.button("+", key=f"{joint.name}_plus_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "nudge", "joint": joint.name, "value": int(abs(nudge_amount))}, refresh=True, rerun=True)

            if st.button("-", key=f"{joint.name}_minus_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "nudge", "joint": joint.name, "value": -int(abs(nudge_amount))}, refresh=True, rerun=True)

            if st.button(home_button_label, key=f"{joint.name}_home_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "home", "joint": joint.name}, refresh=True, rerun=True)

            if st.button("Attach", key=f"{joint.name}_attach_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "attach", "joint": joint.name}, refresh=True, rerun=True)

            if st.button("Detach", key=f"{joint.name}_detach_btn", use_container_width=True):
                run_device_action(base_url, "/api/joint", params={"cmd": "detach", "joint": joint.name}, refresh=True, rerun=True)

            if st.button("Apply joint settings", key=f"{joint.name}_apply_settings", use_container_width=True):
                run_device_action(
                    base_url,
                    "/api/joint",
                    params={
                        "cmd": "apply",
                        "joint": joint.name,
                        "pin": int(pin),
                        "motor_type": 1 if motor_type else 0,
                        "min": int(min_angle),
                        "max": int(max_angle),
                        "home": int(home_angle),
                        "step": int(step),
                        "pulse_min": int(pulse_min),
                        "pulse_max": int(pulse_max),
                        "neutral_output": int(neutral_output),
                        "stop_deadband": int(stop_deadband),
                        "max_speed_scale": int(max_speed_scale),
                        "invert": 1 if invert else 0,
                        "control_mode": control_mode[1],
                        "axis_source": axis_source[1],
                        "positive_button": positive_button[1],
                        "negative_button": negative_button[1],
                        "input_invert": 1 if input_invert else 0,
                    },
                    refresh=True,
                    rerun=True,
                )


def main() -> None:
    st.set_page_config(
        page_title="DUM-E Control Console",
        page_icon=str(BRANDING_DIR / "icon-app.png"),
        layout="wide",
    )
    render_brand_header()

    render_sidebar_brand()
    if "robot_state" not in st.session_state:
        auto_discovery_flow()
    device_url = render_connection_sidebar()
    st.session_state.device_url = device_url

    refresh_col, retry_col = st.sidebar.columns(2)
    with refresh_col:
        if st.button("Refresh current device", use_container_width=True):
            connect_device(st.session_state.get("connected_base_url", device_url))
    with retry_col:
        if st.button("Retry discovery", use_container_width=True):
            st.session_state.pop("robot_state", None)
            st.session_state.pop("connected_base_url", None)
            st.session_state.pop("device_error", None)
            st.session_state["discovery_ran"] = False
            st.rerun()

    if st.session_state.get("device_error"):
        st.error(st.session_state["device_error"])

    if "robot_state" not in st.session_state:
        st.info("DUM-E is not connected yet. The dashboard is trying last known address, mDNS, and local subnet discovery before falling back to manual URL entry.")
        return

    state = st.session_state["robot_state"]
    device_url = st.session_state.get("connected_base_url", device_url)
    top_left, top_mid, top_right = st.columns(3)
    with top_left:
        if st.button("Save calibration to flash", use_container_width=True):
            if run_device_action(device_url, "/api/system", params={"cmd": "save"}, refresh=True):
                st.success("Saved to ESP32 flash.")
    with top_mid:
        if st.button("Reload from flash", use_container_width=True):
            run_device_action(device_url, "/api/system", params={"cmd": "load"}, refresh=True, rerun=True)
    with top_right:
        if st.button("Home all joints", use_container_width=True):
            run_device_action(device_url, "/api/system", params={"cmd": "home_all"}, refresh=True, rerun=True)

    wifi_controls(device_url, state["wifi"])
    controller_controls(device_url, state["controller"])
    for joint_name in state["joints"]:
        joint_controls(device_url, state["joints"][joint_name])


if __name__ == "__main__":
    main()
