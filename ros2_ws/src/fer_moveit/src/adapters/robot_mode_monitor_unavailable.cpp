#include "fer_moveit_config/adapters/robot_mode_monitor.hpp"

#include <stdexcept>

namespace fer_moveit_config
{

// Built instead of robot_mode_monitor.cpp when franka_msgs is absent (core+sim import).
RobotModeMonitor::RobotModeMonitor(
  const rclcpp::Node::SharedPtr &, const rclcpp::CallbackGroup::SharedPtr &)
{
  throw std::runtime_error(
          "hardware:=real needs franka_msgs; this build has none (core+sim import)");
}

}  // namespace fer_moveit_config
