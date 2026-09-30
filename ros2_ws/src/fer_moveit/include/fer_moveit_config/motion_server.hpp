#ifndef FER_MOVEIT_CONFIG__MOTION_SERVER_HPP_
#define FER_MOVEIT_CONFIG__MOTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "fer_interfaces/action/move_to_joints.hpp"
#include "fer_interfaces/action/move_to_pose.hpp"
#include "fer_interfaces/msg/pose_target.hpp"
#include "fer_interfaces/msg/world_object.hpp"
#include "fer_interfaces/srv/check_reachable.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "fer_moveit_config/adapters/move_group_client.hpp"
#include "fer_moveit_config/adapters/robot_mode_monitor.hpp"
#include "fer_moveit_config/adapters/world_model_client.hpp"

namespace fer_moveit_config
{

/// \brief Serves /motion/move_to_pose, /motion/move_to_joints and /motion/check_reachable.
///
/// One motion runs at a time; a newer goal stops the running one, which ends CANCELLED once
/// the arm is at rest.
class MotionServer
{
public:
  /// \param robot_mode null when the hardware reports no robot mode (MuJoCo).
  MotionServer(
    const rclcpp::Node::SharedPtr & node,
    WorldModelClient & world_model,
    MoveGroupClient & move_group,
    const RobotModeMonitor * robot_mode,
    const rclcpp::CallbackGroup::SharedPtr & group
  );

  /// \brief True once move_group is reachable and a joint state has arrived.
  bool ready() const {return ready_;}

private:
  using MoveToPose = fer_interfaces::action::MoveToPose;
  using MoveToJoints = fer_interfaces::action::MoveToJoints;
  using CheckReachable = fer_interfaces::srv::CheckReachable;

  template<typename ActionT>
  typename rclcpp_action::Server<ActionT>::SharedPtr create_action(const std::string & name);

  template<typename ActionT>
  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle
  );

  MotionTarget target_of(const MoveToPose::Goal & goal) const;
  MotionTarget target_of(const MoveToJoints::Goal & goal) const;
  MotionTarget pose_target(const fer_interfaces::msg::PoseTarget & target) const;
  void check_speed(double speed_scaling) const;
  void check_robot_mode() const;
  bool is_active(const rclcpp_action::GoalUUID & goal_id);
  bool wait_for_rest();
  std::optional<sensor_msgs::msg::JointState> current_joints();

  void on_check_reachable(
    const std::shared_ptr<CheckReachable::Request> request,
    std::shared_ptr<CheckReachable::Response> response
  );
  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);
  void on_startup();

  rclcpp::Node::SharedPtr node_;
  WorldModelClient & world_model_;
  MoveGroupClient & move_group_;
  const RobotModeMonitor * robot_mode_;
  rclcpp::CallbackGroup::SharedPtr group_;

  double tf_timeout_;
  double rest_velocity_;
  double rest_timeout_;
  double startup_timeout_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Server<MoveToPose>::SharedPtr pose_server_;
  rclcpp_action::Server<MoveToJoints>::SharedPtr joints_server_;

  rclcpp::Service<CheckReachable>::SharedPtr reachable_service_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::chrono::steady_clock::time_point startup_begin_;

  std::atomic<bool> ready_{false};
  std::mutex goal_mutex_;
  std::optional<rclcpp_action::GoalUUID> active_goal_;
  // Held by the goal that plans and moves; a replaced goal releases it once the arm rests.
  std::mutex motion_mutex_;

  std::mutex joint_mutex_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  rclcpp::Time joint_stamp_;
  uint64_t joint_count_{0};
};

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__MOTION_SERVER_HPP_
