#ifndef FER_MOVEIT_CONFIG__ADAPTERS__ROBOT_MODE_MONITOR_HPP_
#define FER_MOVEIT_CONFIG__ADAPTERS__ROBOT_MODE_MONITOR_HPP_

#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace fer_moveit_config
{

/// \brief Robot mode of the real FER from franka_robot_state_broadcaster (hardware:=real).
///
/// Built from robot_mode_monitor.cpp when franka_msgs is found; otherwise the constructor of
/// robot_mode_monitor_unavailable.cpp throws.
class RobotModeMonitor
{
public:
  /// \throws std::runtime_error in a build without franka_msgs.
  RobotModeMonitor(
    const rclcpp::Node::SharedPtr & node, 
    const rclcpp::CallbackGroup::SharedPtr & group
  );

  /// \return A description when the robot is in reflex or user stop, else nothing.
  std::optional<std::string> error() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::optional<std::string> error_;
};

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__ADAPTERS__ROBOT_MODE_MONITOR_HPP_
