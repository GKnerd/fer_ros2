#include "fer_moveit_config/motion_server.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fer_interfaces/msg/joint_configuration.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "fer_moveit_config/core/checks.hpp"

namespace fer_moveit_config
{

using fer_interfaces::msg::JointConfiguration;
using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::PoseTarget;
using fer_interfaces::msg::WorldObject;

namespace
{

constexpr char BASE_FRAME[] = "base";

Outcome make_outcome(uint8_t code, const std::string & message)
{
  Outcome outcome;
  outcome.code = code;
  outcome.message = message;
  return outcome;
}

void check_ids(const std::vector<WorldObject> & objects, const std::vector<std::string> & ids)
{
  for (const auto & id : ids) {
    const bool known = std::any_of(
      objects.begin(), objects.end(), [&id](const WorldObject & o) {return o.id == id;});
    if (!known) {
      throw MotionError(Outcome::NOT_FOUND, "unknown object '" + id + "' in may_touch");
    }
  }
}

}  // namespace

template<typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr MotionServer::create_action(
  const std::string & name)
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    node_, name,
    [this](const rclcpp_action::GoalUUID & /*goal_id*/, 
      std::shared_ptr<const typename ActionT::Goal> /*goal*/) 
    {
      return ready_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
             rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<GoalHandle> /*goal_handle*/) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this](const std::shared_ptr<GoalHandle> goal_handle) 
    {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        active_goal_ = goal_handle->get_goal_id();
      }
      std::thread([this, goal_handle] {execute<ActionT>(goal_handle);}).detach();
    },
    rcl_action_server_get_default_options(), group_);
}

template<typename ActionT>
void MotionServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle)
{
  std::lock_guard<std::mutex> motion(motion_mutex_);
  const auto goal = goal_handle->get_goal();
  const auto goal_id = goal_handle->get_goal_id();
  auto result = std::make_shared<typename ActionT::Result>();
  auto feedback = std::make_shared<typename ActionT::Feedback>();
  const std::function<bool()> interrupted = [this, goal_handle, goal_id] {
      return goal_handle->is_canceling() || !is_active(goal_id);
    };
  bool moving = false;
  try {
    if (interrupted()) {
      throw Interrupted();
    }
    const MotionTarget target = target_of(*goal);
    check_robot_mode();
    feedback->phase = ActionT::Feedback::PLANNING;
    goal_handle->publish_feedback(feedback);

    const auto objects = world_model_.snapshot();
    check_ids(objects, target.may_touch);
    move_group_.sync_scene(objects);
    const auto trajectory =
      move_group_.plan(target, goal->speed_scaling, std::nullopt, interrupted);

    feedback->phase = ActionT::Feedback::EXECUTING;
    moving = true;
    move_group_.execute(
      trajectory, interrupted, [&feedback, &goal_handle](double progress) {
        feedback->progress = static_cast<float>(progress);
        goal_handle->publish_feedback(feedback);
      });
    result->outcome = make_outcome(Outcome::OK, "");
  } catch (const Interrupted &) {
    const std::string reason =
      goal_handle->is_canceling() ? "cancelled by the client" : "replaced by a newer goal";
    if (moving && !wait_for_rest()) {
      result->outcome = make_outcome(
        Outcome::EXECUTION_FAILED, reason + ", but the arm did not come to rest");
    } else {
      result->outcome = make_outcome(Outcome::CANCELLED, reason);
    }
  } catch (const MotionError & e) {
    result->outcome = make_outcome(e.code(), e.what());
    if (moving && e.code() == Outcome::EXECUTION_FAILED && robot_mode_) {
      if (const auto error = robot_mode_->error()) {
        result->outcome = make_outcome(Outcome::ROBOT_ERROR, *error);
      }
    }
  } catch (const std::exception & e) {
    result->outcome = make_outcome(Outcome::EXECUTION_FAILED, e.what());
  }

  if (result->outcome.code == Outcome::OK) {
    goal_handle->succeed(result);
  } else if (result->outcome.code == Outcome::CANCELLED && goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else {
    goal_handle->abort(result);
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_ == goal_id) {
    active_goal_.reset();
  }
}

MotionServer::MotionServer(
  const rclcpp::Node::SharedPtr & node, 
  WorldModelClient & world_model,
  MoveGroupClient & move_group, 
  const RobotModeMonitor * robot_mode,
  const rclcpp::CallbackGroup::SharedPtr & group)
: node_(node), world_model_(world_model), move_group_(move_group), robot_mode_(robot_mode),
  group_(group),
  tf_timeout_(node->declare_parameter("tf_timeout", 0.2)),
  rest_velocity_(node->declare_parameter("rest_velocity", 0.01)),
  rest_timeout_(node->declare_parameter("rest_timeout", 3.0)),
  startup_timeout_(node->declare_parameter("startup_timeout", 10.0)),
  startup_begin_(std::chrono::steady_clock::now())
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node);

  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr & msg) {on_joint_state(msg);},
    options);

  pose_server_ = create_action<MoveToPose>("/motion/move_to_pose");
  joints_server_ = create_action<MoveToJoints>("/motion/move_to_joints");
  reachable_service_ = node->create_service<CheckReachable>(
    "/motion/check_reachable",
    [this](
      const std::shared_ptr<CheckReachable::Request> request,
      std::shared_ptr<CheckReachable::Response> response) {
      on_check_reachable(request, response);
    },
    rclcpp::ServicesQoS(), group);

  startup_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100), [this] {on_startup();}, group);
}

MotionTarget MotionServer::target_of(const MoveToPose::Goal & goal) const
{
  check_speed(goal.speed_scaling);
  return pose_target(goal.target);
}

MotionTarget MotionServer::target_of(const MoveToJoints::Goal & goal) const
{
  check_speed(goal.speed_scaling);
  MotionTarget target;
  target.kind = MotionTarget::Kind::JOINTS;
  target.joints.assign(goal.joints.positions.begin(), goal.joints.positions.end());
  return target;
}

MotionTarget MotionServer::pose_target(const PoseTarget & target) const
{
  if (target.path != PoseTarget::PATH_FREE && target.path != PoseTarget::PATH_STRAIGHT) {
    throw MotionError(
      Outcome::INVALID_GOAL, "unknown path type " + std::to_string(target.path));
  }
  if (target.pose.header.frame_id.empty()) {
    throw MotionError(Outcome::INVALID_GOAL, "pose has no frame_id");
  }
  auto pose = target.pose;
  pose.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  MotionTarget result;
  try {
    result.pose = tf_buffer_->transform(pose, BASE_FRAME, tf2::durationFromSec(tf_timeout_));
  } catch (const tf2::TransformException & e) {
    throw MotionError(
      Outcome::INVALID_GOAL,
      "cannot transform '" + target.pose.header.frame_id + "' to '" + BASE_FRAME + "': " +
      e.what());
  }
  result.kind = target.path == PoseTarget::PATH_STRAIGHT ?
    MotionTarget::Kind::POSE_STRAIGHT : MotionTarget::Kind::POSE_FREE;
  result.may_touch = target.may_touch;
  return result;
}

void MotionServer::check_speed(double speed_scaling) const
{
  if (!speed_valid(speed_scaling)) {
    throw MotionError(
      Outcome::INVALID_GOAL,
      "speed_scaling must be in (0, 1], got " + std::to_string(speed_scaling));
  }
}

void MotionServer::check_robot_mode() const
{
  if (robot_mode_) {
    if (const auto error = robot_mode_->error()) {
      throw MotionError(Outcome::ROBOT_ERROR, *error);
    }
  }
}

bool MotionServer::is_active(const rclcpp_action::GoalUUID & goal_id)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  return active_goal_ == goal_id;
}

bool MotionServer::wait_for_rest()
{
  uint64_t seen;
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    seen = joint_count_;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(rest_timeout_);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(POLL_PERIOD);
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (joint_count_ > seen && at_rest(velocities_, rest_velocity_)) {
      return true;
    }
  }
  return false;
}

std::optional<sensor_msgs::msg::JointState> MotionServer::current_joints()
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  if (joint_count_ == 0) {
    return std::nullopt;
  }
  sensor_msgs::msg::JointState state;
  state.name = move_group_.joint_names();
  state.position = positions_;
  return state;
}

void MotionServer::on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  const auto & names = move_group_.joint_names();
  std::vector<double> positions;
  std::vector<double> velocities;
  for (const auto & name : names) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) {
      return;
    }
    const auto i = static_cast<size_t>(it - msg->name.begin());
    positions.push_back(msg->position.at(i));
    if (msg->velocity.size() == msg->name.size()) {
      velocities.push_back(msg->velocity[i]);
    }
  }
  const rclcpp::Time stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
  std::lock_guard<std::mutex> lock(joint_mutex_);
  if (velocities.size() != positions.size()) {
    // No velocities published: estimate them from consecutive positions.
    const double dt = joint_count_ > 0 ? (stamp - joint_stamp_).seconds() : 0.0;
    velocities.assign(positions.size(), std::numeric_limits<double>::infinity());
    if (dt > 0.0 && positions_.size() == positions.size()) {
      for (size_t i = 0; i < positions.size(); ++i) {
        velocities[i] = (positions[i] - positions_[i]) / dt;
      }
    }
  }
  positions_ = std::move(positions);
  velocities_ = std::move(velocities);
  joint_stamp_ = stamp;
  ++joint_count_;
}

void MotionServer::on_check_reachable(
  const std::shared_ptr<CheckReachable::Request> request,
  std::shared_ptr<CheckReachable::Response> response)
{
  response->outcome = make_outcome(Outcome::OK, "");
  const auto & names = move_group_.joint_names();
  size_t index = 0;
  try {
    if (!ready_) {
      throw MotionError(Outcome::TIMEOUT, "motion server not ready");
    }
    if (request->targets.empty()) {
      throw MotionError(Outcome::INVALID_GOAL, "no targets");
    }
    std::vector<MotionTarget> targets;
    for (index = 0; index < request->targets.size(); ++index) {
      targets.push_back(pose_target(request->targets[index]));
    }
    index = 0;
    const auto objects = world_model_.snapshot();
    for (index = 0; index < targets.size(); ++index) {
      check_ids(objects, targets[index].may_touch);
    }
    index = 0;
    move_group_.sync_scene(objects);

    auto start = current_joints();
    for (index = 0; index < targets.size(); ++index) {
      const auto trajectory =
        move_group_.plan(targets[index], 1.0, start, [] {return false;});
      const auto & joint_trajectory = trajectory.joint_trajectory;
      sensor_msgs::msg::JointState reached;
      reached.name = joint_trajectory.joint_names;
      reached.position = joint_trajectory.points.back().positions;
      JointConfiguration configuration;
      for (size_t j = 0; j < names.size() && j < configuration.positions.size(); ++j) {
        const auto it = std::find(reached.name.begin(), reached.name.end(), names[j]);
        if (it != reached.name.end()) {
          configuration.positions[j] = reached.position.at(
            static_cast<size_t>(it - reached.name.begin()));
        }
      }
      response->configurations.push_back(configuration);
      start = reached;
    }
  } catch (const MotionError & e) {
    response->outcome = make_outcome(e.code(), e.what());
    response->failed_index = static_cast<uint32_t>(index);
  }
}

void MotionServer::on_startup()
{
  if (ready_) {
    return;
  }
  bool joint_state_seen;
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    joint_state_seen = joint_count_ > 0;
  }
  if (joint_state_seen && move_group_.ready()) {
    ready_ = true;
    startup_timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "Motion server ready");
    return;
  }
  const std::chrono::duration<double> waited = std::chrono::steady_clock::now() - startup_begin_;
  if (waited.count() > startup_timeout_) {
    startup_timer_->cancel();
    RCLCPP_FATAL(
      node_->get_logger(),
      "move_group (/move_action, /execute_trajectory, /apply_planning_scene, "
      "/get_planning_scene, /trajectory_execution_event) or /joint_states not available after "
      "%.1f s", startup_timeout_);
    node_->get_node_base_interface()->get_context()->shutdown("motion server startup failed");
  }
}

}  // namespace fer_moveit_config
