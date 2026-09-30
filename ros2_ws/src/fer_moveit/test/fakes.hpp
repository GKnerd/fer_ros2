#ifndef FER_MOVEIT_CONFIG__TEST__FAKES_HPP_
#define FER_MOVEIT_CONFIG__TEST__FAKES_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fer_interfaces/msg/world_object.hpp"
#include "fer_interfaces/srv/query_objects.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace fer_moveit_config
{

inline const std::vector<std::string> & arm_joints()
{
  static const std::vector<std::string> names{
    "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4", "fer_joint5", "fer_joint6",
    "fer_joint7"};
  return names;
}

/// Arm joints with positions and velocities, published at 100 Hz like joint_state_broadcaster.
class FakeArm
{
public:
  explicit FakeArm(const rclcpp::Node::SharedPtr & node)
  : positions_{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}, velocities_(7, 0.0)
  {
    publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = node->create_wall_timer(
      std::chrono::milliseconds(10), [this, node] {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = node->now();
        msg.name = arm_joints();
        msg.name.push_back("fer_finger_joint1");
        msg.name.push_back("fer_finger_joint2");
        std::lock_guard<std::mutex> lock(mutex_);
        msg.position = positions_;
        msg.position.push_back(finger_);
        msg.position.push_back(finger_);
        msg.velocity = velocities_;
        msg.velocity.push_back(0.0);
        msg.velocity.push_back(0.0);
        publisher_->publish(msg);
      });
  }

  std::vector<double> positions()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return positions_;
  }

  void set(const std::vector<double> & positions, const std::vector<double> & velocities)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    positions_ = positions;
    velocities_ = velocities;
  }

  void set_finger(double position)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    finger_ = position;
  }

private:
  std::mutex mutex_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  double finger_{0.0};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// move_group as the motion server sees it: MoveGroup (plan only), ExecuteTrajectory,
/// the "stop" event, ApplyPlanningScene and GetPlanningScene.
class FakeMoveGroup
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using Codes = moveit_msgs::msg::MoveItErrorCodes;

  FakeMoveGroup(const rclcpp::Node::SharedPtr & node, FakeArm & arm)
  : node_(node), arm_(arm)
  {
    move_server_ = rclcpp_action::create_server<MoveGroup>(
      node, "/move_action",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveGroup::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> handle) {
        start([this, handle] {plan(handle);});
      });
    execute_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
      node, "/execute_trajectory",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTrajectory::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTrajectory>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTrajectory>> handle) {
        start([this, handle] {execute(handle);});
      });
    stop_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/trajectory_execution_event", rclcpp::ServicesQoS(),
      [this](const std_msgs::msg::String::ConstSharedPtr & msg) {
        if (msg->data == "stop") {
          ++stops;
          stop_requested_ = true;
        }
      });
    apply_service_ = node->create_service<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene",
      [this](
        const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> request,
        std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> response) {
        std::lock_guard<std::mutex> lock(mutex);
        applied_scenes.push_back(request->scene);
        response->success = true;
      });
    get_service_ = node->create_service<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene",
      [](
        const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request>,
        std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response) {
        auto & matrix = response->scene.allowed_collision_matrix;
        matrix.entry_names = {"fer_link0", "fer_link1"};
        matrix.entry_values.resize(2);
        matrix.entry_values[0].enabled = {false, true};
        matrix.entry_values[1].enabled = {true, false};
      });
  }

  ~FakeMoveGroup()
  {
    stopping_ = true;
    for (auto & thread : threads_) {
      thread.join();
    }
  }

  // Behaviour, set by the tests.
  std::atomic<int32_t> plan_error{Codes::SUCCESS};
  std::atomic<int> fail_plan_call{-1};
  std::atomic<double> plan_delay{0.0};
  std::atomic<double> trajectory_duration{0.3};
  std::atomic<int32_t> execute_error{Codes::SUCCESS};
  std::atomic<double> rest_delay{0.3};

  // Recorded by the fake.
  std::mutex mutex;
  std::vector<MoveGroup::Goal> plan_goals;
  std::vector<ExecuteTrajectory::Goal> execute_goals;
  std::vector<moveit_msgs::msg::PlanningScene> applied_scenes;
  std::atomic<int> stops{0};
  std::atomic<bool> executing{false};

private:
  template<typename F>
  void start(F && work)
  {
    std::lock_guard<std::mutex> lock(threads_mutex_);
    threads_.emplace_back(std::forward<F>(work));
  }

  void sleep_for(double seconds)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!stopping_ && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroup>> & handle)
  {
    const auto goal = *handle->get_goal();
    int call;
    {
      std::lock_guard<std::mutex> lock(mutex);
      call = static_cast<int>(plan_goals.size());
      plan_goals.push_back(goal);
    }
    sleep_for(plan_delay);
    auto result = std::make_shared<MoveGroup::Result>();
    result->error_code.val = (fail_plan_call < 0 || fail_plan_call == call) ?
      plan_error.load() : Codes::SUCCESS;
    if (result->error_code.val != Codes::SUCCESS) {
      handle->abort(result);
      return;
    }
    std::vector<double> from = arm_.positions();
    if (!goal.request.start_state.joint_state.position.empty()) {
      from = goal.request.start_state.joint_state.position;
    }
    std::vector<double> to = from;
    const auto & joints = goal.request.goal_constraints.at(0).joint_constraints;
    for (size_t i = 0; i < to.size(); ++i) {
      to[i] = joints.size() == to.size() ? joints[i].position : from[i] + 0.1;
    }
    auto & trajectory = result->planned_trajectory.joint_trajectory;
    trajectory.joint_names = arm_joints();
    constexpr int POINTS = 10;
    const double duration = trajectory_duration;
    for (int k = 0; k < POINTS; ++k) {
      const double s = static_cast<double>(k) / (POINTS - 1);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      for (size_t i = 0; i < to.size(); ++i) {
        point.positions.push_back(from[i] + s * (to[i] - from[i]));
        point.velocities.push_back(k == POINTS - 1 ? 0.0 : (to[i] - from[i]) / duration);
      }
      point.time_from_start = rclcpp::Duration::from_seconds(s * duration);
      trajectory.points.push_back(point);
    }
    result->error_code.val = Codes::SUCCESS;
    handle->succeed(result);
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTrajectory>> & handle)
  {
    const auto goal = *handle->get_goal();
    {
      std::lock_guard<std::mutex> lock(mutex);
      execute_goals.push_back(goal);
    }
    stop_requested_ = false;
    executing = true;
    auto result = std::make_shared<ExecuteTrajectory::Result>();
    const auto & points = goal.trajectory.joint_trajectory.points;
    const double duration = rclcpp::Duration(points.back().time_from_start).seconds();
    const auto & first = points.front();
    const auto & last = points.back();
    const auto begin = std::chrono::steady_clock::now();
    while (!stopping_) {
      const double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      const double s = std::min(t / duration, 1.0);
      std::vector<double> positions;
      std::vector<double> velocities;
      for (size_t i = 0; i < first.positions.size(); ++i) {
        positions.push_back(first.positions[i] + s * (last.positions[i] - first.positions[i]));
        velocities.push_back(s < 1.0 ? (last.positions[i] - first.positions[i]) / duration : 0.0);
      }
      if (stop_requested_) {
        // The controller decelerates; move_group reports PREEMPTED right away.
        executing = false;
        result->error_code.val = Codes::PREEMPTED;
        handle->abort(result);
        sleep_for(rest_delay);
        arm_.set(positions, std::vector<double>(positions.size(), 0.0));
        return;
      }
      arm_.set(positions, velocities);
      if (s >= 1.0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executing = false;
    result->error_code.val = execute_error;
    if (execute_error == Codes::SUCCESS) {
      handle->succeed(result);
    } else {
      handle->abort(result);
    }
  }

  rclcpp::Node::SharedPtr node_;
  FakeArm & arm_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> stopping_{false};
  std::mutex threads_mutex_;
  std::vector<std::thread> threads_;
  rclcpp_action::Server<MoveGroup>::SharedPtr move_server_;
  rclcpp_action::Server<ExecuteTrajectory>::SharedPtr execute_server_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop_sub_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_service_;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_service_;
};

/// QueryObjects backed by a list the tests edit.
class FakeWorldModel
{
public:
  explicit FakeWorldModel(const rclcpp::Node::SharedPtr & node)
  {
    service_ = node->create_service<fer_interfaces::srv::QueryObjects>(
      "/world_model/query_objects",
      [this](
        const std::shared_ptr<fer_interfaces::srv::QueryObjects::Request>,
        std::shared_ptr<fer_interfaces::srv::QueryObjects::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->objects = objects_;
      });
  }

  void set(const std::vector<fer_interfaces::msg::WorldObject> & objects)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objects_ = objects;
  }

private:
  std::mutex mutex_;
  std::vector<fer_interfaces::msg::WorldObject> objects_;
  rclcpp::Service<fer_interfaces::srv::QueryObjects>::SharedPtr service_;
};

/// A box in the world model; GRASPED boxes are held by fer_hand_tcp.
inline fer_interfaces::msg::WorldObject box(
  const std::string & id, uint8_t status = fer_interfaces::msg::WorldObject::FREE,
  bool fixed = false)
{
  fer_interfaces::msg::WorldObject object;
  object.id = id;
  object.class_id = id;
  object.status = status;
  object.fixed = fixed;
  object.held_by = status == fer_interfaces::msg::WorldObject::GRASPED ? "fer_hand_tcp" : "";
  object.pose.header.frame_id = object.held_by.empty() ? "base" : object.held_by;
  object.pose.pose.position.x = 0.5;
  object.pose.pose.orientation.w = 1.0;
  object.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  object.shape.dimensions = {0.05, 0.05, 0.1};
  return object;
}

/// Static transform base -> camera, 1 m above the base.
inline std::shared_ptr<tf2_ros::StaticTransformBroadcaster> camera_frame(
  const rclcpp::Node::SharedPtr & node)
{
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node->now();
  transform.header.frame_id = "base";
  transform.child_frame_id = "camera";
  transform.transform.translation.z = 1.0;
  transform.transform.rotation.w = 1.0;
  broadcaster->sendTransform(transform);
  return broadcaster;
}

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__TEST__FAKES_HPP_
