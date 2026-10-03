#include "fer_moveit_config/adapters/move_group_client.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "fer_interfaces/msg/outcome.hpp"
#include "moveit/kinematic_constraints/utils.hpp"
#include "moveit/utils/moveit_error_code.hpp"
#include "moveit_msgs/msg/contact_information.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"

#include "fer_moveit_config/core/checks.hpp"

namespace fer_moveit_config
{

using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::WorldObject;
using moveit_msgs::msg::AllowedCollisionMatrix;
using moveit_msgs::msg::AttachedCollisionObject;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::msg::ContactInformation;
using moveit_msgs::msg::MoveItErrorCodes;

namespace
{

constexpr char PLANNING_GROUP[] = "fer_arm";
constexpr char TCP_LINK[] = "fer_hand_tcp";
constexpr char FREE_PIPELINE[] = "ompl";
constexpr char STRAIGHT_PIPELINE[] = "pilz_industrial_motion_planner";
constexpr char STRAIGHT_PLANNER[] = "LIN";
const std::vector<std::string> HAND_LINKS = {"fer_hand", "fer_leftfinger", "fer_rightfinger"};
const std::vector<std::string> ARM_JOINTS = {
  "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6", "fer_joint7"};
constexpr double JOINT_GOAL_TOLERANCE = 1e-4;
// move_group times the execution itself; this only catches a move_group that stops answering.
constexpr double EXECUTION_WATCHDOG_FACTOR = 2.0;

std::string error_name(const MoveItErrorCodes & code)
{
  return moveit::core::errorCodeToString(moveit::core::MoveItErrorCode(code));
}

uint8_t plan_outcome(const MoveItErrorCodes & code)
{
  switch (code.val) {
    case MoveItErrorCodes::NO_IK_SOLUTION:
    case MoveItErrorCodes::GOAL_IN_COLLISION:
    case MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
    case MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
    case MoveItErrorCodes::GOAL_STATE_INVALID:
      return Outcome::UNREACHABLE;
    default:
      return Outcome::NO_PATH;
  }
}

size_t matrix_entry(AllowedCollisionMatrix & matrix, const std::string & name)
{
  const auto it = std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name);
  if (it != matrix.entry_names.end()) {
    return static_cast<size_t>(it - matrix.entry_names.begin());
  }
  matrix.entry_names.push_back(name);
  for (auto & row : matrix.entry_values) {
    row.enabled.push_back(false);
  }
  matrix.entry_values.emplace_back();
  matrix.entry_values.back().enabled.assign(matrix.entry_names.size(), false);
  return matrix.entry_names.size() - 1;
}

void allow(AllowedCollisionMatrix & matrix, const std::string & a, const std::string & b)
{
  const size_t i = matrix_entry(matrix, a);
  const size_t j = matrix_entry(matrix, b);
  matrix.entry_values[i].enabled[j] = true;
  matrix.entry_values[j].enabled[i] = true;
}

template<typename ActionT>
typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr send_goal(
  const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
  const typename ActionT::Goal & goal, std::chrono::duration<double> timeout,
  const std::string & what)
{
  auto future = client->async_send_goal(goal);
  if (!wait_for(future, timeout)) {
    throw MotionError(Outcome::TIMEOUT, "no answer from move_group (" + what + ")");
  }
  auto handle = future.get();
  if (!handle) {
    throw MotionError(Outcome::TIMEOUT, "move_group rejected the " + what + " goal");
  }
  return handle;
}

}  // namespace

MoveGroupClient::MoveGroupClient(
  const rclcpp::Node::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group)
: planning_time_(node->declare_parameter("planning_time", 5.0)),
  planning_attempts_(static_cast<int>(node->declare_parameter("planning_attempts", 1))),
  goal_position_tolerance_(node->declare_parameter("goal_position_tolerance", 0.001)),
  goal_orientation_tolerance_(node->declare_parameter("goal_orientation_tolerance", 0.01)),
  timeout_(node->declare_parameter("move_group_timeout", 7.0)),
  move_client_(rclcpp_action::create_client<MoveGroup>(node, "/move_action", group)),
  execute_client_(rclcpp_action::create_client<ExecuteTrajectory>(
      node, "/execute_trajectory", group)),
  apply_client_(node->create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene", rclcpp::ServicesQoS(), group)),
  get_client_(node->create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene", rclcpp::ServicesQoS(), group)),
  validity_client_(node->create_client<moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity", rclcpp::ServicesQoS(), group)),
  stop_publisher_(node->create_publisher<std_msgs::msg::String>(
      "/trajectory_execution_event", rclcpp::ServicesQoS()))
{
}

const std::vector<std::string> & MoveGroupClient::joint_names() const
{
  return ARM_JOINTS;
}

bool MoveGroupClient::ready() const
{
  return move_client_->action_server_is_ready() && execute_client_->action_server_is_ready() &&
         apply_client_->service_is_ready() && get_client_->service_is_ready() &&
         validity_client_->service_is_ready() &&
         stop_publisher_->get_subscription_count() > 0;
}

void MoveGroupClient::sync_scene(const std::vector<WorldObject> & objects)
{
  std::lock_guard<std::mutex> lock(scene_mutex_);
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  auto & scene = request->scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;

  std::set<std::string> world_ids;
  std::map<std::string, std::string> attached_links;
  for (const auto & object : objects) {
    CollisionObject box;
    box.id = object.id;
    box.header.frame_id = object.pose.header.frame_id;
    box.pose = object.pose.pose;
    box.primitives = {object.shape};
    box.primitive_poses.resize(1);
    box.primitive_poses[0].orientation.w = 1.0;
    box.operation = CollisionObject::ADD;
    if (object.status == WorldObject::GRASPED) {
      AttachedCollisionObject attached;
      attached.link_name = object.held_by;
      attached.object = box;
      attached.touch_links = HAND_LINKS;
      scene.robot_state.attached_collision_objects.push_back(attached);
      attached_links[object.id] = object.held_by;
    } else {
      scene.world.collision_objects.push_back(box);
      world_ids.insert(object.id);
    }
  }
  // A detached object returns to the world; removed below unless it is FREE again.
  std::set<std::string> leaving;
  for (const auto & [id, link] : attached_links_) {
    if (attached_links.count(id) == 0) {
      AttachedCollisionObject detach;
      detach.link_name = link;
      detach.object.id = id;
      detach.object.operation = CollisionObject::REMOVE;
      scene.robot_state.attached_collision_objects.push_back(detach);
      leaving.insert(id);
    }
  }
  leaving.insert(world_ids_.begin(), world_ids_.end());
  for (const auto & id : leaving) {
    if (world_ids.count(id) == 0 && attached_links.count(id) == 0) {
      CollisionObject remove;
      remove.id = id;
      remove.operation = CollisionObject::REMOVE;
      scene.world.collision_objects.push_back(remove);
    }
  }

  auto pending = apply_client_->async_send_request(request);
  auto future = pending.future.share();
  if (!wait_for(future, timeout_)) {
    apply_client_->remove_pending_request(pending.request_id);
    throw MotionError(Outcome::TIMEOUT, "move_group did not answer the planning scene update");
  }
  if (!future.get()->success) {
    throw MotionError(Outcome::NO_PATH, "move_group rejected the planning scene update");
  }
  world_ids_ = std::move(world_ids);
  attached_links_ = std::move(attached_links);
}

std::vector<MoveGroupClient::Contact> MoveGroupClient::held_contacts(
  const std::optional<sensor_msgs::msg::JointState> & start)
{
  {
    std::lock_guard<std::mutex> lock(scene_mutex_);
    if (attached_links_.empty()) {
      return {};
    }
  }
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  request->robot_state.is_diff = true;
  if (start) {
    request->robot_state.joint_state = *start;
  }
  request->group_name = PLANNING_GROUP;
  auto pending = validity_client_->async_send_request(request);
  auto future = pending.future.share();
  if (!wait_for(future, timeout_)) {
    validity_client_->remove_pending_request(pending.request_id);
    throw MotionError(Outcome::TIMEOUT, "move_group did not check the start state");
  }
  std::vector<Contact> contacts;
  for (const auto & contact : future.get()->contacts) {
    if (contact.body_type_1 == ContactInformation::ROBOT_ATTACHED &&
      contact.body_type_2 == ContactInformation::WORLD_OBJECT)
    {
      contacts.emplace_back(contact.contact_body_1, contact.contact_body_2);
    } else if (contact.body_type_2 == ContactInformation::ROBOT_ATTACHED &&
      contact.body_type_1 == ContactInformation::WORLD_OBJECT)
    {
      contacts.emplace_back(contact.contact_body_2, contact.contact_body_1);
    }
  }
  return contacts;
}

AllowedCollisionMatrix MoveGroupClient::touch_matrix(
  const std::vector<std::string> & ids, const std::vector<Contact> & contacts)
{
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto pending = get_client_->async_send_request(request);
  auto future = pending.future.share();
  if (!wait_for(future, timeout_)) {
    get_client_->remove_pending_request(pending.request_id);
    throw MotionError(Outcome::TIMEOUT, "move_group did not return its allowed collisions");
  }
  // A scene diff replaces the whole matrix, so the pairs go into a complete copy.
  AllowedCollisionMatrix matrix = future.get()->scene.allowed_collision_matrix;
  for (const auto & id : ids) {
    for (const auto & link : HAND_LINKS) {
      allow(matrix, id, link);
    }
  }
  for (const auto & [held, other] : contacts) {
    allow(matrix, held, other);
  }
  return matrix;
}

moveit_msgs::msg::RobotTrajectory MoveGroupClient::plan(
  const MotionTarget & target, double speed_scaling,
  const std::optional<sensor_msgs::msg::JointState> & start,
  const std::function<bool()> & interrupted)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  MoveGroup::Goal goal;
  auto & request = goal.request;
  request.group_name = PLANNING_GROUP;
  if (target.kind == MotionTarget::Kind::JOINTS) {
    request.pipeline_id = FREE_PIPELINE;
    moveit_msgs::msg::Constraints constraints;
    for (size_t i = 0; i < ARM_JOINTS.size() && i < target.joints.size(); ++i) {
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name = ARM_JOINTS[i];
      joint.position = target.joints[i];
      joint.tolerance_above = JOINT_GOAL_TOLERANCE;
      joint.tolerance_below = JOINT_GOAL_TOLERANCE;
      joint.weight = 1.0;
      constraints.joint_constraints.push_back(joint);
    }
    request.goal_constraints = {constraints};
  } else {
    const bool straight = target.kind == MotionTarget::Kind::POSE_STRAIGHT;
    request.pipeline_id = straight ? STRAIGHT_PIPELINE : FREE_PIPELINE;
    request.planner_id = straight ? STRAIGHT_PLANNER : "";
    request.goal_constraints = {kinematic_constraints::constructGoalConstraints(
        TCP_LINK, target.pose, goal_position_tolerance_, goal_orientation_tolerance_)};
  }
  request.start_state.is_diff = true;
  if (start) {
    request.start_state.joint_state = *start;
  }
  request.max_velocity_scaling_factor = speed_scaling;
  request.max_acceleration_scaling_factor = speed_scaling;
  request.allowed_planning_time = planning_time_;
  request.num_planning_attempts = planning_attempts_;
  goal.planning_options.plan_only = true;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  // Only a straight path keeps them: a free path could drag the held object through the surface.
  std::vector<Contact> contacts;
  if (target.kind == MotionTarget::Kind::POSE_STRAIGHT) {
    contacts = held_contacts(start);
  }
  if (!target.may_touch.empty() || !contacts.empty()) {
    goal.planning_options.planning_scene_diff.allowed_collision_matrix =
      touch_matrix(target.may_touch, contacts);
  }

  auto handle = send_goal<MoveGroup>(move_client_, goal, timeout_, "planning");
  auto result_future = move_client_->async_get_result(handle);
  try {
    if (!wait_for(result_future, timeout_, interrupted)) {
      move_client_->async_cancel_goal(handle);
      throw MotionError(Outcome::TIMEOUT, "move_group did not return a plan");
    }
  } catch (const Interrupted &) {
    // move_group finishes a started plan before it sees the cancel.
    move_client_->async_cancel_goal(handle);
    wait_for(result_future, timeout_);
    throw;
  }
  const auto result = result_future.get().result;
  if (!result) {
    throw MotionError(Outcome::NO_PATH, "move_group returned no planning result");
  }
  if (result->error_code.val != MoveItErrorCodes::SUCCESS) {
    throw MotionError(
      plan_outcome(result->error_code), "planning failed: " + error_name(result->error_code));
  }
  if (result->planned_trajectory.joint_trajectory.points.empty()) {
    throw MotionError(Outcome::NO_PATH, "move_group returned an empty trajectory");
  }
  return result->planned_trajectory;
}

void MoveGroupClient::execute(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::function<bool()> & interrupted,
  const std::function<void(double)> & on_progress)
{
  const double duration =
    rclcpp::Duration(trajectory.joint_trajectory.points.back().time_from_start).seconds();
  const std::chrono::duration<double> limit(
    duration * EXECUTION_WATCHDOG_FACTOR + timeout_.count());

  // No controller named: move_group uses its default arm controller (arm_control_type).
  ExecuteTrajectory::Goal goal;
  goal.trajectory = trajectory;
  auto handle = send_goal<ExecuteTrajectory>(execute_client_, goal, timeout_, "execution");
  auto result_future = execute_client_->async_get_result(handle);

  const auto start = std::chrono::steady_clock::now();
  while (result_future.wait_for(POLL_PERIOD) != std::future_status::ready) {
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
    if (interrupted()) {
      stop();
      if (!wait_for(result_future, timeout_)) {
        throw MotionError(Outcome::EXECUTION_FAILED, "move_group did not confirm the stop");
      }
      throw Interrupted();
    }
    if (elapsed > limit) {
      stop();
      wait_for(result_future, timeout_);
      throw MotionError(Outcome::EXECUTION_FAILED, "no execution result from move_group");
    }
    on_progress(progress(elapsed.count(), duration));
  }

  const auto result = result_future.get().result;
  if (!result) {
    throw MotionError(Outcome::EXECUTION_FAILED, "move_group returned no execution result");
  }
  switch (result->error_code.val) {
    case MoveItErrorCodes::SUCCESS:
      on_progress(1.0);
      return;
    case MoveItErrorCodes::PREEMPTED:
      throw MotionError(Outcome::EXECUTION_FAILED, "stopped by another client");
    case MoveItErrorCodes::TIMED_OUT:
      throw MotionError(Outcome::EXECUTION_FAILED, "motion took too long");
    default:
      throw MotionError(
        Outcome::EXECUTION_FAILED, "execution failed (" + error_name(result->error_code) +
        "); is the arm controller active?");
  }
}

void MoveGroupClient::stop()
{
  std_msgs::msg::String event;
  event.data = "stop";
  stop_publisher_->publish(event);
}

}  // namespace fer_moveit_config
