# servo_bridge

ROS 2 bridge between the Arduino UNO R4 + DYNAMIXEL Shield firmware
([`arduino/two_servos`](../arduino/two_servos)) and ROS 2 topics.
Up to three servos: arm joint 1 (ID 1), arm joint 2 (ID 2), gripper (ID 3).

| Topic | Type | Direction |
|---|---|---|
| `arm/joint_states` | `sensor_msgs/JointState` | published, ~20 Hz, connected joints only, position [rad] / velocity [rad/s] |
| `arm/setpoint_reached` | `servo_bridge/SetpointReached` | published, ~20 Hz, `joint1` / `joint2` / `gripper` bools (false if not connected) |
| `arm/goal` | `sensor_msgs/JointState` | subscribed, goal position [rad] for **every connected** joint |

### Connected servos

The board pings IDs 1-3 **once at startup** and keeps the ones that answer. It reports them in a
`C <n> <ids...>` line, which the bridge also requests on every (re)connect. Log example:

```
[servo_bridge]: 2 of 3 servos connected: arm_joint1 (ID 1), arm_joint2 (ID 2)
```

To add a servo later, plug it in and **reset the board** (reset button, or replug USB).

### Goals

A goal must contain exactly one position per **connected** joint: either with the names of all
connected joints (any order) or with no names (then in `servo_ids` order, skipping joints that
are not connected). Goals with the wrong number of positions, unknown, unconnected or repeated
names, or NaN/inf values are ignored as a whole and logged as a warning; no servo moves.

0 rad = encoder tick 2048 (`center_tick` parameter). Parameters are in
[`config/servo_bridge.yaml`](config/servo_bridge.yaml).

## Build (inside the container)

```bash
cd $ROS_WS
colcon build --packages-select servo_bridge
source install/setup.bash
```

## Run

```bash
ros2 launch servo_bridge servo_bridge.launch.py port:=/dev/ttyACM0
```

## Try it

```bash
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/setpoint_reached

# With 3 servos connected: all three by name (any order) ...
ros2 topic pub --once /arm/goal sensor_msgs/msg/JointState \
  "{name: [arm_joint1, arm_joint2, gripper], position: [0.5, 0.0, -0.3]}"
# ... or in order (joint1, joint2, gripper), no names
ros2 topic pub --once /arm/goal sensor_msgs/msg/JointState "{position: [0.0, 0.0, 0.0]}"

# With only the two arm servos connected:
ros2 topic pub --once /arm/goal sensor_msgs/msg/JointState "{position: [0.5, -0.5]}"
# ... while this one is ignored (gripper not connected):
ros2 topic pub --once /arm/goal sensor_msgs/msg/JointState "{position: [0.5, -0.5, 0.0]}"
```

## Testing the firmware without ROS

Use the PlatformIO serial monitor on Windows (the stream scrolls; type `G 1 2048` + Enter).
Only one program can hold the port, so stop the bridge first.

`talker` (also built here) is the older request/reply terminal. It prints replies until the board
goes quiet, which never happens with the streaming firmware, so it only suits request/reply sketches.
