#include "fer_moveit_config/adapters/robot_mode_monitor.hpp"

#include <optional>
#include <sstream>
#include <string>

#include "franka_msgs/msg/franka_state.hpp"

namespace fer_moveit_config
{

using franka_msgs::msg::FrankaState;

namespace
{

constexpr char ROBOT_STATE_TOPIC[] = "/franka_robot_state_broadcaster/robot_state";

// Names of the flags that are set, e.g. "joint_reflex, cartesian_reflex".
std::string active_errors(const franka_msgs::msg::Errors & errors)
{
  std::istringstream lines(franka_msgs::msg::to_yaml(errors));
  std::string line;
  std::string names;
  while (std::getline(lines, line)) {
    const auto colon = line.find(": true");
    if (colon != std::string::npos) {
      names += (names.empty() ? "" : ", ") + line.substr(0, colon);
    }
  }
  return names.empty() ? "none" : names;
}

}  // namespace

RobotModeMonitor::RobotModeMonitor(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  subscription_ = node->create_subscription<FrankaState>(
    ROBOT_STATE_TOPIC,
    rclcpp::SystemDefaultsQoS(),
    [this](const FrankaState::ConstSharedPtr & state) 
    {
      std::optional<std::string> error;
      if (state->robot_mode == FrankaState::ROBOT_MODE_REFLEX)
      {
        error = "robot in reflex; current errors: " + active_errors(state->current_errors) +
        "; last motion errors: " + active_errors(state->last_motion_errors);
      } 
      else if (state->robot_mode == FrankaState::ROBOT_MODE_USER_STOPPED)
      {
        error = "robot user-stopped";
      }
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = error;
    }, 
    options);
}

}  // namespace fer_moveit_config
