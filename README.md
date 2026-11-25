## Simple ROS2+Arduino Dynamixel control for the uni project 

This project has been built for the Arduino Uno R4 WiFi (ESP32), Dynamixel Arduino shield and Dynamixel XL430-W250T servomotors. The ROS2 node is supposed to run on the onboard machine (e.g Nvidia Jetson) and talk to the motors via serial interface. The motors are used for actuating a 2DoF 3D-printed robot arm. Supports ROS2 Humble.

The code does not specify the amount of motors connected, but normally it is 3 (2 joints and 1 gripper). 

## Quick start
### Build and run 
```bash
cd /your_ros2_ws/src
git clone https://github.com/shellyshampaine/easy_dynamixel_control.git
colcon build 
source install/setup.bash
ros2 launch servo_bridge servo_bridge.launch.py port:=/dev/ttyACM0
```

### Sending a position command via ROS2 topic
```bash
ros2 topic pub --once /arm/goal sensor_msgs/msg/JointState \
  "{name: [arm_joint1, arm_joint2], position: [0.5, -0.5]}"
```

or 

```bash
ros2 topic pub --once /arm/goal sensor_msgs/msg/JointState "{position: [0.5, -0.5]}"
```

### Echoing joint states

```bash
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/joint_states --field position # echo only position
```

### Echoing if the joints reached the goals
```bash
ros2 topic echo /arm/setpoint_reached
```