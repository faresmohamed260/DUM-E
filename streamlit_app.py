from __future__ import annotations

import ipaddress
import json
import socket
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List
from urllib.parse import urlparse

import requests
import streamlit as st
from ik_path import GRIPPER_ACTIONS, IK_JOINTS, CapturedIkPose, IkPathModule, IkPathPlan
from sequence_live import LiveSequenceRecorder
from sequence_module import DumeSequenceModule

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
ROS2_SHARED_PROFILE_PATH = APP_ROOT / "ros2_dashboard" / "shared_profile.json"
DISCOVERY_TIMEOUT = 1.0
DISCOVERY_WORKERS = 32


def create_http_session() -> requests.Session:
    session = requests.Session()
    session.trust_env = False
    session.headers.update(
        {
            "Connection": "close",
            "User-Agent": "DUM-E-Dashboard/1.0",
        }
    )
    return session


HTTP_SESSION = create_http_session()

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
    response = HTTP_SESSION.get(f"{base_url.rstrip('/')}{path}", params=params, timeout=8)
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


def load_sequence_config(base_url: str) -> dict:
    return {
        "bridge": {
            "device_base_url": normalize_base_url(base_url),
            "dry_run": True,
        },
        "controller": {
            "axis_deadzone": 48,
        },
    }


def get_sequence_recorder(base_url: str) -> LiveSequenceRecorder:
    normalized = normalize_base_url(base_url)
    recorder = st.session_state.get("main_sequence_live_recorder")
    recorder_url = st.session_state.get("main_sequence_live_recorder_url")
    if recorder is None or recorder_url != normalized:
        recorder = LiveSequenceRecorder(root=APP_ROOT, config=load_sequence_config(normalized))
        st.session_state.main_sequence_live_recorder = recorder
        st.session_state.main_sequence_live_recorder_url = normalized
    return recorder


def get_sequence_module(base_url: str) -> DumeSequenceModule:
    normalized = normalize_base_url(base_url)
    module = st.session_state.get("main_sequence_module")
    module_url = st.session_state.get("main_sequence_module_url")
    if module is None or module_url != normalized:
        module = DumeSequenceModule(root=APP_ROOT, config=load_sequence_config(normalized))
        st.session_state.main_sequence_module = module
        st.session_state.main_sequence_module_url = normalized
    return module


def get_ik_path_module(base_url: str) -> IkPathModule:
    normalized = normalize_base_url(base_url)
    module = st.session_state.get("main_ik_path_module")
    module_url = st.session_state.get("main_ik_path_module_url")
    if module is None or module_url != normalized:
        module = IkPathModule(root=APP_ROOT, base_url=normalized)
        st.session_state.main_ik_path_module = module
        st.session_state.main_ik_path_module_url = normalized
    return module


def normalize_base_url(value: str) -> str:
    candidate = value.strip()
    if not candidate:
        return ""
    if "://" not in candidate:
        candidate = f"http://{candidate}"
    return candidate.rstrip("/")


def canonical_device_base_url(base_url: str, payload: dict | None = None) -> str:
    normalized = normalize_base_url(base_url)
    if not payload:
        return normalized
    ip_address = str(payload.get("ip_address", "")).strip()
    if ip_address:
        return normalize_base_url(ip_address)
    return normalized


def local_cache() -> dict:
    if DEVICE_CACHE_PATH.exists():
        try:
            return json.loads(DEVICE_CACHE_PATH.read_text(encoding="utf-8"))
        except Exception:
            return {}
    return {}


def save_local_cache(data: dict) -> None:
    DEVICE_CACHE_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")


def export_ros2_shared_profile(*, base_url: str, plan: IkPathPlan, safe_height_mm: float, delay_after_ms: int, steps_per_segment: int) -> None:
    payload = {
        "base_url": normalize_base_url(base_url),
        "safe_height_mm": float(safe_height_mm),
        "waypoint_delay_ms": int(delay_after_ms),
        "gripper_close_angle": int(plan.gripper_close_value or 20),
        "steps_per_segment": int(steps_per_segment),
        "arrival_tolerance_mm": 10,
        "pick_position": [
            round(float(plan.start_pose.cartesian_pose_mm["x"]) / 1000.0, 6),
            round(float(plan.start_pose.cartesian_pose_mm["y"]) / 1000.0, 6),
            round(float(plan.start_pose.cartesian_pose_mm["z"]) / 1000.0, 6),
        ],
        "drop_position": [
            round(float(plan.end_pose.cartesian_pose_mm["x"]) / 1000.0, 6),
            round(float(plan.end_pose.cartesian_pose_mm["y"]) / 1000.0, 6),
            round(float(plan.end_pose.cartesian_pose_mm["z"]) / 1000.0, 6),
        ],
        "source": "streamlit_app.full_ik_path",
        "exported_at": time.time(),
        "ik_plan_name": plan.name,
    }
    ROS2_SHARED_PROFILE_PATH.parent.mkdir(parents=True, exist_ok=True)
    ROS2_SHARED_PROFILE_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def is_captured_ik_pose(value: object) -> bool:
    return hasattr(value, "servo_pose") and hasattr(value, "cartesian_pose_mm")


def is_ik_path_plan(value: object) -> bool:
    return hasattr(value, "name") and hasattr(value, "commands") and hasattr(value, "start_pose") and hasattr(value, "end_pose")


def parse_device_info(base_url: str, payload: dict) -> DeviceInfo:
    canonical_base_url = canonical_device_base_url(base_url, payload)
    return DeviceInfo(
        base_url=canonical_base_url,
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
        response = HTTP_SESSION.get(f"{candidate}{IDENTIFY_PATH}", timeout=timeout)
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


def format_request_exception(exc: Exception) -> str:
    if isinstance(exc, requests.exceptions.ConnectTimeout):
        return "connection timed out"
    if isinstance(exc, requests.exceptions.ReadTimeout):
        return "device responded too slowly"
    if isinstance(exc, requests.exceptions.ConnectionError):
        return "connection failed"
    if isinstance(exc, requests.exceptions.HTTPError):
        response = exc.response
        if response is not None:
            return f"HTTP {response.status_code}"
        return "HTTP error"
    if isinstance(exc, ValueError):
        return "response was not valid JSON"
    return str(exc)


def diagnose_device_connection(base_url: str, original_error: Exception) -> str:
    target = normalize_base_url(base_url)
    details: list[str] = [f"Failed to connect to `{target}`: {format_request_exception(original_error)}."]
    for path, timeout in [(IDENTIFY_PATH, 3), ("/api/state", 5)]:
        try:
            response = HTTP_SESSION.get(f"{target}{path}", timeout=timeout)
            response.raise_for_status()
            payload = response.json()
            details.append(
                f"`{path}` responded with HTTP {response.status_code}, ok={payload.get('ok', True)}, "
                f"device_type={payload.get('device_type', 'n/a')}."
            )
        except Exception as exc:
            details.append(f"`{path}` check failed: {format_request_exception(exc)}.")
    local_ips = local_ipv4_addresses()
    if local_ips:
        details.append(f"Local IPv4 addresses: {', '.join(local_ips)}.")
    details.append("Confirm this PC is on the same SSID/VLAN as the ESP and not on an isolated guest network.")
    return " ".join(details)


def connect_device(base_url: str, device: DeviceInfo | None = None) -> bool:
    target = normalize_base_url(base_url)
    try:
        if device is None:
            device = probe_device(target, timeout=1.0)
        resolved_target = device.base_url if device is not None else target
        sync_state(resolved_target)
        st.session_state.connected_base_url = resolved_target
        st.session_state.device_url = resolved_target
        st.session_state.pop("device_error", None)
        if device is not None:
            cache_successful_device(resolved_target, device)
        else:
            cache_successful_device(resolved_target)
        return True
    except Exception as exc:
        st.session_state.device_error = diagnose_device_connection(target, exc)
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


def render_sequence_recorder_section(base_url: str) -> None:
    recorder = get_sequence_recorder(base_url)
    module = get_sequence_module(base_url)
    snapshot = recorder.snapshot()

    with st.expander("Sequence Recorder", expanded=False):
        st.caption(
            "Record controller-driven moves into a replayable sequence. "
            "This uses live `/api/state` updates and `ps4.inputs` for more reliable 360-joint capture."
        )

        controls_left, controls_mid, controls_right = st.columns(3)
        with controls_left:
            session_name = st.text_input(
                "Session name",
                value=st.session_state.get("main_sequence_session_name", "test_sequence"),
                key="main_sequence_session_name_input",
            )
            st.session_state.main_sequence_session_name = session_name
            overwrite_existing = st.checkbox("Overwrite existing session file", value=False, key="main_sequence_overwrite")
        with controls_mid:
            start_clicked = st.button("Start Recording", use_container_width=True, disabled=snapshot.running, key="main_sequence_start")
        with controls_right:
            stop_clicked = st.button("Stop Recording", use_container_width=True, disabled=not snapshot.running, key="main_sequence_stop")

        if start_clicked:
            try:
                session = recorder.start(session_name, overwrite_existing=overwrite_existing)
                st.success(f"Recording started: {session.session_name}")
                st.rerun()
            except Exception as exc:
                st.error(str(exc))

        if stop_clicked:
            session = recorder.stop()
            if session is not None:
                st.success(f"Recording stopped: {session.session_name}")
                st.rerun()

        status_left, status_mid, status_right = st.columns(3)
        with status_left:
            st.metric("Recorder", "Running" if snapshot.running else "Idle")
            st.write(f"DUM-E: `{snapshot.dume_base_url or normalize_base_url(base_url)}`")
        with status_mid:
            st.metric("Steps Recorded", snapshot.steps_recorded)
            st.write(f"Controller connected: `{snapshot.controller_connected}`")
        with status_right:
            st.write(f"Included 360 joint: `{snapshot.included_continuous_joint or 'none'}`")
            st.write(f"Excluded 360 joints: `{', '.join(snapshot.excluded_continuous_joints) or 'none'}`")

        if snapshot.running:
            st.info("Recording is active. Move DUM-E with the controller now.")
        if snapshot.last_error:
            st.error(f"Recorder issue: {snapshot.last_error}")

        session_to_show = snapshot.session_name or st.session_state.get("main_sequence_session_name")
        if session_to_show:
            session_path = module.session_path(session_to_show)
            if session_path.exists():
                session_payload = json.loads(session_path.read_text(encoding="utf-8"))
                steps = session_payload.get("steps", [])
                if steps:
                    st.dataframe(
                        [
                            {
                                "index": index + 1,
                                "kind": step.get("kind"),
                                "joint": step.get("joint_name"),
                                "target": step.get("target_value"),
                                "speed": step.get("speed_percent"),
                                "direction": step.get("direction"),
                                "duration_ms": step.get("duration_ms"),
                                "delay_after_ms": step.get("delay_after_ms"),
                                "note": step.get("note"),
                            }
                            for index, step in enumerate(steps)
                        ],
                        use_container_width=True,
                        hide_index=True,
                    )
                else:
                    st.caption("No steps recorded yet for this session.")

                action_left, action_mid = st.columns(2)
                with action_left:
                    if st.button("Preview Replay", use_container_width=True, disabled=snapshot.running, key="main_sequence_preview_replay"):
                        st.session_state.main_sequence_replay_result = module.replay(session_to_show, dry_run=True).to_dict()
                    if st.button("Execute Replay", use_container_width=True, disabled=snapshot.running, key="main_sequence_execute_replay"):
                        st.session_state.main_sequence_replay_result = module.replay(session_to_show, dry_run=False).to_dict()
                with action_mid:
                    if st.button("Preview Return Home", use_container_width=True, disabled=snapshot.running, key="main_sequence_preview_return"):
                        st.session_state.main_sequence_return_result = module.return_home(session_to_show, dry_run=True).to_dict()
                    if st.button("Execute Return Home", use_container_width=True, disabled=snapshot.running, key="main_sequence_execute_return"):
                        st.session_state.main_sequence_return_result = module.return_home(session_to_show, dry_run=False).to_dict()

                with st.expander("Session JSON", expanded=False):
                    st.json(session_payload)

        if "main_sequence_replay_result" in st.session_state:
            with st.expander("Replay Result", expanded=False):
                st.json(st.session_state.main_sequence_replay_result)

        if "main_sequence_return_result" in st.session_state:
            with st.expander("Return Home Result", expanded=False):
                st.json(st.session_state.main_sequence_return_result)

    if snapshot.running:
        time.sleep(0.5)
        st.rerun()


def render_ik_path_section(base_url: str) -> None:
    module = get_ik_path_module(base_url)

    with st.expander("Full IK Path", expanded=False):
        st.caption(
            "Capture a manual `start` and `end` gripper position, then generate a Cartesian IK path using only `base`, `shoulder`, and `elbow`. "
            "Optional safe-height waypoints help avoid platform collisions, and path building now rejects waypoints that dip into the platform footprint."
        )

        name_col, steps_col, delay_col, safe_col = st.columns(4)
        with name_col:
            plan_name = st.text_input(
                "IK path name",
                value=st.session_state.get("main_ik_path_name", "ik_path_1"),
                key="main_ik_path_name_input",
            )
            st.session_state.main_ik_path_name = plan_name
        with steps_col:
            interpolation_steps = st.number_input("Steps per segment", min_value=2, max_value=80, value=10, step=1)
        with delay_col:
            delay_after_ms = st.number_input("Delay after each waypoint (ms)", min_value=0, max_value=3000, value=140, step=10)
        with safe_col:
            safe_height_mm = st.number_input("Safe travel height Z (mm)", min_value=0.0, max_value=500.0, value=120.0, step=5.0)

        use_safe_height = st.checkbox("Use safe-height waypoint path", value=True, key="main_ik_use_safe_height")
        grip_left, grip_mid, grip_right = st.columns(3)
        with grip_left:
            gripper_prepare_action = st.selectbox(
                "Gripper before start move",
                options=list(GRIPPER_ACTIONS),
                index=list(GRIPPER_ACTIONS).index("open"),
                key="main_ik_gripper_prepare_action",
            )
        with grip_mid:
            gripper_action_after_start_delay = st.selectbox(
                "Gripper after reaching start",
                options=list(GRIPPER_ACTIONS),
                index=list(GRIPPER_ACTIONS).index("close"),
                key="main_ik_gripper_action_after_start_delay",
            )
        with grip_right:
            gripper_action_at_end = st.selectbox(
                "Gripper after reaching end",
                options=list(GRIPPER_ACTIONS),
                index=list(GRIPPER_ACTIONS).index("open"),
                key="main_ik_gripper_action_end",
            )
        grip_cfg_left, grip_cfg_mid, grip_cfg_right = st.columns(3)
        with grip_cfg_left:
            gripper_start_delay_ms = st.number_input(
                "Start grip delay (ms)",
                min_value=0,
                max_value=5000,
                value=300,
                step=50,
                key="main_ik_gripper_start_delay_ms",
            )
        with grip_cfg_mid:
            gripper_end_delay_ms = st.number_input(
                "End release delay (ms)",
                min_value=0,
                max_value=5000,
                value=0,
                step=50,
                key="main_ik_gripper_end_delay_ms",
            )
        with grip_cfg_right:
            gripper_close_value = st.number_input(
                "Gripper close angle",
                min_value=0,
                max_value=180,
                value=20,
                step=1,
                key="main_ik_gripper_close_value",
            )

        capture_left, capture_mid, capture_right = st.columns(3)
        with capture_left:
            if st.button("Capture Start Pose", use_container_width=True, key="main_ik_capture_start"):
                try:
                    st.session_state.main_ik_start = module.capture_pose()
                    st.success("Captured start pose.")
                except Exception as exc:
                    st.error(str(exc))
        with capture_mid:
            if st.button("Capture End Pose", use_container_width=True, key="main_ik_capture_end"):
                try:
                    st.session_state.main_ik_end = module.capture_pose()
                    st.success("Captured end pose.")
                except Exception as exc:
                    st.error(str(exc))
        with capture_right:
            if st.button("Clear IK Poses", use_container_width=True, key="main_ik_clear"):
                st.session_state.pop("main_ik_start", None)
                st.session_state.pop("main_ik_end", None)
                st.session_state.pop("main_ik_plan", None)
                st.session_state.pop("main_ik_result", None)
                st.rerun()

        start_pose = st.session_state.get("main_ik_start")
        end_pose = st.session_state.get("main_ik_end")

        status_left, status_right = st.columns(2)
        with status_left:
            st.markdown("**Start pose**")
            if is_captured_ik_pose(start_pose):
                st.json({"servo": start_pose.servo_pose, "cartesian_mm": start_pose.cartesian_pose_mm})
            else:
                st.caption("Not captured yet.")
        with status_right:
            st.markdown("**End pose**")
            if is_captured_ik_pose(end_pose):
                st.json({"servo": end_pose.servo_pose, "cartesian_mm": end_pose.cartesian_pose_mm})
            else:
                st.caption("Not captured yet.")

        if st.button("Build IK Path", use_container_width=True, key="main_ik_build"):
            if not is_captured_ik_pose(start_pose) or not is_captured_ik_pose(end_pose):
                st.error("Capture both start and end poses first.")
            else:
                try:
                    plan = module.build_plan(
                        plan_name,
                        start_pose,
                        end_pose,
                        interpolation_steps=int(interpolation_steps),
                        delay_after_ms=int(delay_after_ms),
                        use_safe_height=bool(use_safe_height),
                        safe_height_mm=float(safe_height_mm),
                        gripper_prepare_action=gripper_prepare_action,
                        gripper_action_after_start_delay=gripper_action_after_start_delay,
                        gripper_action_at_end=gripper_action_at_end,
                        gripper_start_delay_ms=int(gripper_start_delay_ms),
                        gripper_end_delay_ms=int(gripper_end_delay_ms),
                        gripper_close_value=int(gripper_close_value),
                    )
                    st.session_state.main_ik_plan = plan
                    export_ros2_shared_profile(
                        base_url=base_url,
                        plan=plan,
                        safe_height_mm=float(safe_height_mm),
                        delay_after_ms=int(delay_after_ms),
                        steps_per_segment=int(interpolation_steps),
                    )
                    st.success(f"Built IK path: {plan.name}")
                except Exception as exc:
                    st.error(str(exc))

        plan = st.session_state.get("main_ik_plan")
        if is_ik_path_plan(plan):
            st.write(f"Path `{plan.name}` with `{len(plan.commands)}` solved IK waypoints across `{', '.join(IK_JOINTS)}`.")
            st.caption(
                f"Gripper: before start `{plan.gripper_prepare_action}`"
                + (f" -> {plan.gripper_open_value}" if plan.gripper_prepare_action == "open" and plan.gripper_open_value is not None else "")
                + (f" -> {plan.gripper_close_value}" if plan.gripper_prepare_action == "close" and plan.gripper_close_value is not None else "")
                + f", after start `{plan.gripper_action_after_start_delay}` after {plan.gripper_start_delay_ms} ms"
                + (f" -> {plan.gripper_open_value}" if plan.gripper_action_after_start_delay == "open" and plan.gripper_open_value is not None else "")
                + (f" -> {plan.gripper_close_value}" if plan.gripper_action_after_start_delay == "close" and plan.gripper_close_value is not None else "")
                + f", end `{plan.gripper_action_at_end}`"
                + (f" -> {plan.gripper_open_value}" if plan.gripper_action_at_end == "open" and plan.gripper_open_value is not None else "")
                + (f" -> {plan.gripper_close_value}" if plan.gripper_action_at_end == "close" and plan.gripper_close_value is not None else "")
                + "."
            )
            st.dataframe(
                [
                    {
                        "index": index + 1,
                        "base": command["base"],
                        "shoulder": command["shoulder"],
                        "elbow": command["elbow"],
                    }
                    for index, command in enumerate(plan.commands)
                ],
                use_container_width=True,
                hide_index=True,
            )

            execute_left, execute_mid = st.columns(2)
            with execute_left:
                if st.button("Preview IK Execution", use_container_width=True, key="main_ik_preview"):
                    st.session_state.main_ik_result = module.execute_plan(plan, dry_run=True)
            with execute_mid:
                if st.button("Execute IK Path", use_container_width=True, key="main_ik_execute"):
                    st.session_state.main_ik_result = module.execute_plan(plan, dry_run=False)

        if "main_ik_result" in st.session_state:
            with st.expander("IK Result", expanded=False):
                st.json(st.session_state.main_ik_result)


def wifi_controls(base_url: str, wifi: WifiState) -> None:
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
    with st.expander(joint.name.replace("_", " ").title(), expanded=False):
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

    render_sequence_recorder_section(device_url)
    render_ik_path_section(device_url)
    with st.expander("Wi-Fi", expanded=False):
        wifi_controls(device_url, state["wifi"])
    with st.expander("Wireless Controller", expanded=False):
        controller_controls(device_url, state["controller"])
    for joint_name in state["joints"]:
        joint_controls(device_url, state["joints"][joint_name])


if __name__ == "__main__":
    main()
