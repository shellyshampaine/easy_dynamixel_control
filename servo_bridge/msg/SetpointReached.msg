# Whether each servo has come to rest after its last goal (arm/setpoint_reached).
# A servo counts as "reached" when it has stopped moving after the goal was sent and
# reports no hardware error. It is false while moving, on a hardware error, or when
# the servo does not answer. A gripper stopped by an object also counts as reached.

std_msgs/Header header

bool joint1     # arm joint 1
bool joint2     # arm joint 2
bool gripper
