# State of the Franka Hand as read by libfranka Gripper::readOnce().
std_msgs/Header header

# Current opening between the fingers. Unit: [m]
float64 width

# Maximum opening, measured by homing. Unit: [m]
float64 max_width

# True while an object is held within the epsilon window of the last grasp.
bool is_grasped

# Gripper temperature. Unit: [°C]
uint16 temperature
