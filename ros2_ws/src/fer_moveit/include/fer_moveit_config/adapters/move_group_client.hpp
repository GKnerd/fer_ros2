#ifndef FER_MOVEIT_CONFIG__ADAPTERS__MOVE_GROUP_CLIENT_HPP_
#define FER_MOVEIT_CONFIG__ADAPTERS__MOVE_GROUP_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fer_interfaces/msg/world_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace fer_moveit_config
{

/// \brief One motion request: a TCP pose in the base frame, or a joint configuration.
struct MotionTarget
{
  enum class Kind {POSE_FREE, POSE_STRAIGHT, JOINTS};

  Kind kind{Kind::JOINTS};
  geometry_msgs::msg::PoseStamped pose;
  std::vector<double> joints;
  std::vector<std::string> may_touch;
};

/// \brief Everything the motion server asks of move_group: scene, plans, execution, stop.
class MoveGroupClient
{
public:
  MoveGroupClient(
    const rclcpp::Node::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group);

  /// \brief True once move_group's actions, services and stop topic are reachable.
  bool ready() const;

  /// \brief Arm joint names, joint 1 to 7.
  const std::vector<std::string> & joint_names() const;

  /// \brief Make move_group's scene match the world-model snapshot (lasting).
  /// \throws MotionError TIMEOUT or NO_PATH.
  void sync_scene(const std::vector<fer_interfaces::msg::WorldObject> & objects);

  /// \brief Plan only; \p start empty means move_group's current state.
  ///
  /// A straight path may keep the contacts a held object has at \p start (lift off a surface).
  /// \throws MotionError UNREACHABLE, NO_PATH or TIMEOUT; Interrupted.
  moveit_msgs::msg::RobotTrajectory plan(
    const MotionTarget & target, double speed_scaling,
    const std::optional<sensor_msgs::msg::JointState> & start,
    const std::function<bool()> & interrupted);

  /// \brief Execute through move_group; publishes "stop" when \p interrupted.
  /// \throws MotionError EXECUTION_FAILED or TIMEOUT; Interrupted once move_group has stopped.
  void execute(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::function<bool()> & interrupted,
    const std::function<void(double)> & on_progress);

private:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;

  using Contact = std::pair<std::string, std::string>;

  /// (held object, world object) pairs in contact at \p start; empty if nothing is held.
  std::vector<Contact> held_contacts(const std::optional<sensor_msgs::msg::JointState> & start);
  moveit_msgs::msg::AllowedCollisionMatrix touch_matrix(
    const std::vector<std::string> & ids, const std::vector<Contact> & contacts);
  void stop();

  double planning_time_;
  int planning_attempts_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  std::chrono::duration<double> timeout_;

  rclcpp_action::Client<MoveGroup>::SharedPtr move_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr execute_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop_publisher_;

  std::mutex scene_mutex_;
  std::set<std::string> world_ids_;
  std::map<std::string, std::string> attached_links_;
  std::mutex plan_mutex_;
};

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__ADAPTERS__MOVE_GROUP_CLIENT_HPP_
