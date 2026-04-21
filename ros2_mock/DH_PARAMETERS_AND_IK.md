# DH Parameters and IK Explanation

This portable ROS2 layer presents DUM-E through a ROS2-shaped interface, but the actual motion planning still comes from DUM-E's hand-engineered kinematics.

## Coordinate frame

- Origin: center of the elevated platform top
- `x`, `y`: platform plane
- `z = 0`: platform top
- `z < 0`: below platform top toward the surrounding table

## Kinematic chain used in DUM-E

The active pick-and-place path uses:

1. `base`
2. `shoulder`
3. `elbow`

The gripper is handled as a separate end-action primitive instead of being part of the solved IK chain.

## DH-style interpretation

The current implementation is not a full symbolic DH solver, but it follows the same practical structure:

- one revolute joint for base rotation
- one revolute joint for shoulder elevation
- one revolute joint for elbow elevation
- fixed link lengths and offsets loaded from `config/arm_geometry.json`

This is enough to model the end-effector position used for pick-and-place.

## Forward kinematics

Forward kinematics is implemented in:

- `ik_kinematics.py`

It converts joint values into a Cartesian end-effector position:

- servo pose -> mechanical pose
- mechanical pose -> `(x, y, z)`

That FK path is used for:

- live pose capture
- validating current end-effector position
- reporting motion progress

## Inverse kinematics

Inverse kinematics is implemented in:

- `ik_kinematics.py`
- `ik_path.py`

The solver takes a Cartesian waypoint and computes a valid:

- base angle
- shoulder angle
- elbow angle

Those joint targets are then converted back into DUM-E servo commands.

## Safe travel path

The pick-and-place flow uses waypoint-based IK instead of a single direct move:

1. move above start
2. descend to start
3. grip
4. rise
5. move above end
6. descend to end
7. release

This reduces:

- platform collisions
- side impacts on the target
- unsafe horizontal motion at low height

## Why this matters for the assignment

Even though the ROS2 layer is portable and locally bundled, the actual robot logic still demonstrates:

- FK implementation
- IK implementation
- a working pick-and-place state machine
- feedback publication with status and distance
- successful result return after execution
