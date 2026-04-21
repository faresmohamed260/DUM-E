# Portable ROS2 Compatibility Layer

This package presents DUM-E through an assignment-friendly ROS2-shaped wrapper that runs entirely locally and can be bundled into a portable Windows app.

What is real:

- DUM-E firmware
- DUM-E dashboard calibration
- DUM-E kinematics and gripper execution
- DUM-E HTTP control and state feedback

What is simulated locally:

- ROS2 node registry
- ROS2 action transport
- ROS2 topic echo output

The goal is to satisfy the assignment/demo interface expectations while letting DUM-E continue doing the real robot work underneath.

## Files

- `dume_ros2_mock/action/PickAndPlace.action`
- `pick_and_place_server.py`
- `mock_ros2_cli.py`
- `dume_bridge.py`
- `runtime.py`

## Run the server

```powershell
python -m ros2_mock.pick_and_place_server --safe-height-mm 200 --waypoint-delay-ms 200
```

Optional:

```powershell
python -m ros2_mock.pick_and_place_server --base-url http://192.168.1.132 --gripper-close-angle 25
```

## Show nodes

```powershell
python -m ros2_mock.mock_ros2_cli node list
```

## Send a goal

```powershell
python -m ros2_mock.mock_ros2_cli action send_goal /pick_and_place dume_ros2_mock/action/PickAndPlace "{""pick_pose"": {""position"": {""x"": 0.10, ""y"": 0.00, ""z"": 0.20}, ""orientation"": {""x"": 0.0, ""y"": 0.0, ""z"": 0.0, ""w"": 1.0}}, ""drop_pose"": {""position"": {""x"": 0.20, ""y"": 0.10, ""z"": 0.05}, ""orientation"": {""x"": 0.0, ""y"": 0.0, ""z"": 0.0, ""w"": 1.0}}}" --feedback
```

## Echo topics

```powershell
python -m ros2_mock.mock_ros2_cli topic echo /arm_controller/joint_trajectory --follow
python -m ros2_mock.mock_ros2_cli topic echo /joint_states --follow
```

## Runtime output

All mock ROS2 bus files are written under:

- `output/ros2_mock/actions/`
- `output/ros2_mock/topics/`
- `output/ros2_mock/nodes/`
