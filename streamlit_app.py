from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import requests
import streamlit as st


DEFAULT_DEVICE_URL = "http://192.168.4.1"
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


@dataclass
class JointState:
    name: str
    pin: int
    motor_type: str
    min_angle: int
    max_angle: int
    home_angle: int
    step: int
    pulse_min: int
    pulse_max: int
    invert: bool
    position: int
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
    connected: bool
    esp32_bt_mac: str
    controller_name: str
    controller_type: str
    controller_bt_addr: str
    led_r: int
    led_g: int
    led_b: int
    rumble_force: int
    rumble_duration: int
    battery: int


@dataclass
class WifiState:
    hostname: str
    ap_ssid: str
    ap_ip: str
    sta_ssid: str
    sta_connected: bool
    sta_ip: str


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


def parse_joint_state(payload: dict) -> JointState:
    return JointState(
        name=payload["name"],
        pin=int(payload["pin"]),
        motor_type=str(payload["motor_type"]),
        min_angle=int(payload["min_angle"]),
        max_angle=int(payload["max_angle"]),
        home_angle=int(payload["home_angle"]),
        step=int(payload["step"]),
        pulse_min=int(payload["pulse_min"]),
        pulse_max=int(payload["pulse_max"]),
        invert=bool(payload["invert"]),
        position=int(payload["position"]),
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
        connected=bool(payload["connected"]),
        esp32_bt_mac=str(payload["esp32_bt_mac"]),
        controller_name=str(payload["controller_name"]),
        controller_type=str(payload["controller_type"]),
        controller_bt_addr=str(payload["controller_bt_addr"]),
        led_r=int(payload["led_r"]),
        led_g=int(payload["led_g"]),
        led_b=int(payload["led_b"]),
        rumble_force=int(payload["rumble_force"]),
        rumble_duration=int(payload["rumble_duration"]),
        battery=int(payload["battery"]),
    )


def parse_wifi_state(payload: dict) -> WifiState:
    return WifiState(
        hostname=str(payload["hostname"]),
        ap_ssid=str(payload["ap_ssid"]),
        ap_ip=str(payload["ap_ip"]),
        sta_ssid=str(payload["sta_ssid"]),
        sta_connected=bool(payload["sta_connected"]),
        sta_ip=str(payload["sta_ip"]),
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


def wifi_controls(base_url: str, wifi: WifiState) -> None:
    st.subheader("Wi-Fi")
    info_left, info_mid, info_right = st.columns(3)
    with info_left:
        st.write(f"AP SSID: `{wifi.ap_ssid}`")
        st.write(f"AP IP: `{wifi.ap_ip}`")
    with info_mid:
        st.write(f"Station SSID: `{wifi.sta_ssid or 'not set'}`")
        st.write(f"Station connected: `{wifi.sta_connected}`")
    with info_right:
        st.write(f"Station IP: `{wifi.sta_ip or 'not connected'}`")
        st.write(f"Hostname: `{wifi.hostname}`")

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
    info_left, info_mid, info_right = st.columns(3)
    with info_left:
        st.write(f"ESP32 Bluetooth MAC: `{controller.esp32_bt_mac}`")
        st.write(f"Pairing mode: `{controller.allow_new_connections}`")
    with info_mid:
        st.write(f"Connected: `{controller.connected}`")
        st.write(f"Battery: `{controller.battery}`")
    with info_right:
        st.write(f"Controller: `{controller.controller_name or 'none'}`")
        st.write(f"Type / BT addr: `{controller.controller_type or 'unknown'}` / `{controller.controller_bt_addr or 'n/a'}`")

    st.caption("To pair a DualShock 4 wirelessly, enable pairing mode below, then hold SHARE + PS until the light bar flashes quickly.")

    top_left, top_mid, top_right = st.columns(3)
    with top_left:
        enabled = st.toggle(
            "Controller input enabled",
            value=controller.enabled,
            help="Disable this while tuning joints from the UI to avoid conflicting commands.",
        )
        if enabled != controller.enabled:
            run_device_action(base_url, "/api/ps4", params={"cmd": "enable", "value": 1 if enabled else 0}, refresh=True, rerun=True)
    with top_mid:
        pair_mode = st.toggle(
            "Allow new wireless pairing",
            value=controller.allow_new_connections,
            help="Turn this on before putting the gamepad in pairing mode.",
        )
        if pair_mode != controller.allow_new_connections:
            run_device_action(base_url, "/api/ps4", params={"cmd": "pair_mode", "value": 1 if pair_mode else 0}, refresh=True, rerun=True)
    with top_right:
        if st.button("Disconnect current controller", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "disconnect"}, refresh=True, rerun=True)

    danger_left, danger_mid = st.columns(2)
    with danger_left:
        if st.button("Forget all paired controllers", use_container_width=True):
            run_device_action(base_url, "/api/ps4", params={"cmd": "forget"}, refresh=True, rerun=True)
            st.warning("The ESP32 forgot its stored Bluetooth keys. Re-enable pairing mode and pair again if needed.")
    with danger_mid:
        st.info("Once paired, you can turn pairing mode off so only remembered controllers reconnect.")

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


def joint_controls(base_url: str, joint: JointState) -> None:
    with st.expander(joint.name.replace("_", " ").title(), expanded=True):
        is_continuous = joint.motor_type == "continuous_360"
        command_label = "Current speed" if is_continuous else "Current position"
        move_label = "Speed command" if is_continuous else "Target position"
        min_label = "Min speed" if is_continuous else "Min angle"
        max_label = "Max speed" if is_continuous else "Max angle"
        home_label = "Stop / home" if is_continuous else "Home angle"
        left, right, actions = st.columns([2, 2, 1])
        with left:
            st.write(f"{command_label}: `{joint.position}` | Attached: `{joint.attached}` | Velocity: `{joint.velocity}`")
            move_angle = st.slider(
                f"{joint.name}_move",
                min_value=joint.min_angle,
                max_value=joint.max_angle,
                value=joint.position,
                key=f"{joint.name}_move_slider",
                help=move_label,
                label_visibility="collapsed",
            )
            nudge_amount = st.number_input(
                f"{joint.name}_nudge",
                min_value=-100 if is_continuous else -180,
                max_value=100 if is_continuous else 180,
                value=joint.step,
                key=f"{joint.name}_nudge_value",
                step=1,
                label_visibility="collapsed",
            )
        with right:
            pin = st.number_input("Pin", min_value=0, max_value=255, value=joint.pin, key=f"{joint.name}_pin")
            motor_type = st.toggle("Continuous 360 motor", value=is_continuous, key=f"{joint.name}_motor_type")
            min_angle = st.number_input(min_label, min_value=-100 if motor_type else 0, max_value=100 if motor_type else 180, value=joint.min_angle, key=f"{joint.name}_min")
            max_angle = st.number_input(max_label, min_value=-100 if motor_type else 0, max_value=100 if motor_type else 180, value=joint.max_angle, key=f"{joint.name}_max")
            home_angle = st.number_input(home_label, min_value=-100 if motor_type else 0, max_value=100 if motor_type else 180, value=joint.home_angle, key=f"{joint.name}_home")
            step = st.number_input("Step", min_value=1, max_value=30, value=joint.step, key=f"{joint.name}_step")
            pulse_min = st.number_input("Pulse min", min_value=100, max_value=3000, value=joint.pulse_min, key=f"{joint.name}_pulse_min")
            pulse_max = st.number_input("Pulse max", min_value=200, max_value=4000, value=joint.pulse_max, key=f"{joint.name}_pulse_max")
            invert = st.checkbox("Invert", value=joint.invert, key=f"{joint.name}_invert")

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
            if st.button("Home", key=f"{joint.name}_home_btn", use_container_width=True):
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
    st.set_page_config(page_title="Robot Arm Calibration Console", layout="wide")
    st.title("Robot Arm Calibration Console")
    st.caption("Wireless ESP32 control over Wi-Fi with native wireless controller pairing through Bluepad32.")

    default_url = st.session_state.get("device_url", DEFAULT_DEVICE_URL)
    device_url = st.sidebar.text_input("ESP32 device URL", value=default_url).strip().rstrip("/")
    st.session_state.device_url = device_url
    st.sidebar.caption("Default AP address is usually `http://192.168.4.1`.")

    connect_col, refresh_col = st.sidebar.columns(2)
    with connect_col:
        if st.button("Connect", use_container_width=True):
            if run_device_action(device_url, "/api/state"):
                sync_state(device_url)
    with refresh_col:
        if st.button("Refresh", use_container_width=True):
            run_device_action(device_url, "/api/state")
            sync_state(device_url)

    if st.session_state.get("device_error"):
        st.error(st.session_state["device_error"])

    if "robot_state" not in st.session_state:
        st.info("Power the ESP32, connect to its Wi-Fi or local network, enter the device URL, then click Connect.")
        return

    state = st.session_state["robot_state"]
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
