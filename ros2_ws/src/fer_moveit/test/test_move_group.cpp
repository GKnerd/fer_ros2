// Runs against the real move_group started by move_group_test.launch.py.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "fer_interfaces/action/move_to_joints.hpp"
#include "fer_interfaces/action/move_to_pose.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fer_moveit_config/adapters/move_group_client.hpp"
#include "fer_moveit_config/adapters/world_model_client.hpp"
#include "fer_moveit_config/motion_server.hpp"
#include "fakes.hpp"

namespace fer_moveit_config
{

using control_msgs::action::FollowJointTrajectory;
using control_msgs::action::GripperCommand;
using fer_interfaces::action::MoveToJoints;
using fer_interfaces::action::MoveToPose;
using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::PoseTarget;
using fer_interfaces::msg::WorldObject;
using namespace std::chrono_literals;

/// A trajectory controller that plays trajectories into FakeArm and decelerates on cancel.
class FakeTrajectoryController
{
public:
  FakeTrajectoryController(
    const rclcpp::Node::SharedPtr & node, FakeArm & arm, const std::string & name)
  : arm_(arm)
  {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node, "/" + name + "/follow_joint_trajectory",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>) {
        ++cancels;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> h) {
        std::lock_guard<std::mutex> lock(threads_mutex_);
        threads_.emplace_back([this, h] {play(h);});
      });
  }

  ~FakeTrajectoryController()
  {
    stopping_ = true;
    for (auto & thread : threads_) {
      thread.join();
    }
  }

  trajectory_msgs::msg::JointTrajectory last()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return trajectories_.empty() ? trajectory_msgs::msg::JointTrajectory() : trajectories_.back();
  }

  std::atomic<int> cancels{0};
  std::atomic<bool> playing{false};

private:
  void play(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> & h)
  {
    const auto trajectory = h->get_goal()->trajectory;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      trajectories_.push_back(trajectory);
    }
    playing = true;
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    const auto & points = trajectory.points;
    const auto begin = std::chrono::steady_clock::now();
    const double duration = rclcpp::Duration(points.back().time_from_start).seconds();
    std::vector<double> positions = points.front().positions;
    while (!stopping_) {
      const double t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      size_t k = 1;
      while (k < points.size() - 1 &&
        rclcpp::Duration(points[k].time_from_start).seconds() < t)
      {
        ++k;
      }
      const double t0 = rclcpp::Duration(points[k - 1].time_from_start).seconds();
      const double t1 = rclcpp::Duration(points[k].time_from_start).seconds();
      const double s = t1 > t0 ? std::clamp((t - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
      std::vector<double> velocities;
      for (size_t i = 0; i < positions.size(); ++i) {
        positions[i] = points[k - 1].positions[i] +
          s * (points[k].positions[i] - points[k - 1].positions[i]);
        velocities.push_back(t < duration ? points[k].velocities.at(i) : 0.0);
      }
      if (h->is_canceling()) {
        std::this_thread::sleep_for(300ms);
        arm_.set(positions, std::vector<double>(positions.size(), 0.0));
        playing = false;
        h->canceled(result);
        return;
      }
      arm_.set(positions, velocities);
      if (t >= duration) {
        break;
      }
      std::this_thread::sleep_for(10ms);
    }
    playing = false;
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    h->succeed(result);
  }

  FakeArm & arm_;
  std::atomic<bool> stopping_{false};
  std::mutex mutex_;
  std::vector<trajectory_msgs::msg::JointTrajectory> trajectories_;
  std::mutex threads_mutex_;
  std::vector<std::thread> threads_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
};

class MoveGroupIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"startup_timeout", 120.0}, {"move_group_timeout", 10.0}});
    node_ = std::make_shared<rclcpp::Node>("fer_moveit_motion_server", options);
    fakes_ = std::make_shared<rclcpp::Node>("move_group_test_fakes");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    arm_ = std::make_unique<FakeArm>(fakes_);
    controller_ =
      std::make_unique<FakeTrajectoryController>(fakes_, *arm_, "effort_trajectory_controller");
    position_controller_ = std::make_unique<FakeTrajectoryController>(
      fakes_, *arm_, "position_trajectory_controller");
    for (const auto & name : {"gripper_effort_controller", "gripper_position_controller"}) {
      grippers_.push_back(
        rclcpp_action::create_server<GripperCommand>(
          fakes_, std::string("/") + name + "/gripper_cmd",
          [](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperCommand::Goal>) {
            return rclcpp_action::GoalResponse::REJECT;
          },
          [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>) {
            return rclcpp_action::CancelResponse::ACCEPT;
          },
          [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>) {}));
    }
    world_ = std::make_unique<FakeWorldModel>(fakes_);
    world_->set({table()});
    tf_ = camera_frame(fakes_);

    world_model_ = std::make_unique<WorldModelClient>(node_, group_);
    move_group_ = std::make_unique<MoveGroupClient>(node_, group_);
    server_ = std::make_unique<MotionServer>(
      node_, *world_model_, *move_group_, nullptr, group_);

    pose_client_ = rclcpp_action::create_client<MoveToPose>(fakes_, "/motion/move_to_pose");
    joints_client_ =
      rclcpp_action::create_client<MoveToJoints>(fakes_, "/motion/move_to_joints");
    fk_client_ = fakes_->create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 8);
    executor_->add_node(node_);
    executor_->add_node(fakes_);
    spinner_ = std::thread([this] {executor_->spin();});

    ASSERT_TRUE(wait_until([this] {return server_->ready();}, 120s));
    ASSERT_TRUE(fk_client_->wait_for_service(10s));
    // Let move_group see the fake arm's state.
    std::this_thread::sleep_for(500ms);
  }

  void TearDown() override
  {
    move_group_->sync_scene({});
    executor_->cancel();
    spinner_.join();
    server_.reset();
    controller_.reset();
    position_controller_.reset();
    rclcpp::shutdown();
  }

  template<typename Predicate>
  static bool wait_until(Predicate predicate, std::chrono::duration<double> timeout = 10s)
  {
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (!predicate()) {
      if (std::chrono::steady_clock::now() > end) {
        return false;
      }
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  static WorldObject table()
  {
    auto object = box("table", WorldObject::FREE, true);
    object.pose.pose.position.x = 0.3;
    object.pose.pose.position.z = -0.01;
    object.shape.dimensions = {1.2, 1.2, 0.02};
    return object;
  }

  geometry_msgs::msg::Pose tcp_pose(
    const std::vector<std::string> & names, const std::vector<double> & positions)
  {
    auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
    request->header.frame_id = "base";
    request->fk_link_names = {"fer_hand_tcp"};
    request->robot_state.joint_state.name = names;
    request->robot_state.joint_state.position = positions;
    auto future = fk_client_->async_send_request(request).future.share();
    EXPECT_EQ(future.wait_for(10s), std::future_status::ready);
    const auto response = future.get();
    EXPECT_EQ(response->pose_stamped.size(), 1u);
    return response->pose_stamped.at(0).pose;
  }

  MoveToPose::Goal down_from_here(double z, const std::vector<std::string> & may_touch = {})
  {
    MoveToPose::Goal goal;
    goal.target.pose.header.frame_id = "base";
    goal.target.pose.pose = tcp_pose(arm_joints(), arm_->positions());
    goal.target.pose.pose.position.z = z;
    goal.target.path = PoseTarget::PATH_STRAIGHT;
    goal.target.may_touch = may_touch;
    goal.speed_scaling = 0.5;
    return goal;
  }

  template<typename ActionT>
  Outcome run(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal)
  {
    auto sent = client->async_send_goal(goal);
    EXPECT_EQ(sent.wait_for(20s), std::future_status::ready);
    auto result = client->async_get_result(sent.get());
    EXPECT_EQ(result.wait_for(60s), std::future_status::ready);
    return result.get().result->outcome;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr fakes_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<FakeArm> arm_;
  std::unique_ptr<FakeTrajectoryController> controller_;
  std::unique_ptr<FakeTrajectoryController> position_controller_;
  std::vector<rclcpp_action::Server<GripperCommand>::SharedPtr> grippers_;
  std::unique_ptr<FakeWorldModel> world_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  std::unique_ptr<WorldModelClient> world_model_;
  std::unique_ptr<MoveGroupClient> move_group_;
  std::unique_ptr<MotionServer> server_;
  rclcpp_action::Client<MoveToPose>::SharedPtr pose_client_;
  rclcpp_action::Client<MoveToJoints>::SharedPtr joints_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
};

TEST_F(MoveGroupIntegration, StraightPathKeepsTheTcpOnTheLine)
{
  const auto start = tcp_pose(arm_joints(), arm_->positions());
  const auto goal = down_from_here(start.position.z - 0.15);
  const auto outcome = run<MoveToPose>(pose_client_, goal);
  ASSERT_EQ(outcome.code, Outcome::OK) << outcome.message;

  const auto trajectory = controller_->last();
  ASSERT_GE(trajectory.points.size(), 2u);
  const auto & a = start.position;
  const auto & b = goal.target.pose.pose.position;
  const double length = std::hypot(b.x - a.x, b.y - a.y, b.z - a.z);
  for (const auto & point : trajectory.points) {
    const auto p = tcp_pose(trajectory.joint_names, point.positions).position;
    const double s = std::clamp(
      ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y) + (p.z - a.z) * (b.z - a.z)) /
      (length * length), 0.0, 1.0);
    const double distance = std::hypot(
      p.x - (a.x + s * (b.x - a.x)), p.y - (a.y + s * (b.y - a.y)),
      p.z - (a.z + s * (b.z - a.z)));
    EXPECT_LT(distance, 1e-3);
  }
  const auto end = tcp_pose(trajectory.joint_names, trajectory.points.back().positions).position;
  EXPECT_NEAR(end.z, b.z, 1e-3);
}

TEST_F(MoveGroupIntegration, HandMayTouchOnlyListedObjects)
{
  const auto goal = down_from_here(tcp_pose(arm_joints(), arm_->positions()).position.z - 0.1);
  auto cube = box("cup_1");
  cube.pose.pose.position = goal.target.pose.pose.position;
  cube.shape.dimensions = {0.03, 0.03, 0.03};
  world_->set({table(), cube});

  // move_group's plan-only path reports every planning failure as FAILURE, hence NO_PATH.
  const auto blocked = run<MoveToPose>(pose_client_, goal);
  EXPECT_EQ(blocked.code, Outcome::NO_PATH) << blocked.message;
  const auto touching = down_from_here(goal.target.pose.pose.position.z, {"cup_1"});
  const auto allowed = run<MoveToPose>(pose_client_, touching);
  EXPECT_EQ(allowed.code, Outcome::OK) << allowed.message;
}

TEST_F(MoveGroupIntegration, HeldObjectCollidesWithTheTable)
{
  // A 30 cm bar held in the hand, reaching 25 cm below the TCP.
  auto bar = box("bar", WorldObject::GRASPED);
  bar.pose.pose.position.x = 0.0;
  bar.pose.pose.position.z = 0.1;
  bar.shape.dimensions = {0.04, 0.04, 0.3};
  world_->set({table(), bar});
  const auto held = run<MoveToPose>(pose_client_, down_from_here(0.2));
  EXPECT_NE(held.code, Outcome::OK) << held.message;

  world_->set({table()});
  const auto empty = run<MoveToPose>(pose_client_, down_from_here(0.2));
  EXPECT_EQ(empty.code, Outcome::OK) << empty.message;
}

TEST_F(MoveGroupIntegration, HeldObjectLiftsStraightOffTheSurfaceItRestsOn)
{
  // A held bar reaching 1 mm into the table, as a box picked from it.
  const double height = tcp_pose(arm_joints(), arm_->positions()).position.z;
  auto bar = box("bar", WorldObject::GRASPED);
  bar.pose.pose.position.x = 0.0;
  bar.shape.dimensions = {0.04, 0.04, height + 0.001};
  bar.pose.pose.position.z = bar.shape.dimensions[2] / 2.0;
  world_->set({table(), bar});

  MoveToJoints::Goal free;
  free.joints.positions = {0.3, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  free.speed_scaling = 1.0;
  const auto dragged = run<MoveToJoints>(joints_client_, free);
  EXPECT_EQ(dragged.code, Outcome::NO_PATH) << dragged.message;

  const auto lifted = run<MoveToPose>(pose_client_, down_from_here(height + 0.05));
  EXPECT_EQ(lifted.code, Outcome::OK) << lifted.message;
}

TEST_F(MoveGroupIntegration, ClosedFingerBelowItsLimitDoesNotBlockPlanning)
{
  arm_->set_finger(-0.00085);
  std::this_thread::sleep_for(500ms);
  MoveToJoints::Goal goal;
  goal.joints.positions = {0.3, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  goal.speed_scaling = 1.0;
  const auto joints = run<MoveToJoints>(joints_client_, goal);
  EXPECT_EQ(joints.code, Outcome::OK) << joints.message;
  const auto straight = run<MoveToPose>(
    pose_client_, down_from_here(tcp_pose(arm_joints(), arm_->positions()).position.z - 0.05));
  EXPECT_EQ(straight.code, Outcome::OK) << straight.message;
}

TEST_F(MoveGroupIntegration, SpeedScalingBoundsTheJointVelocities)
{
  // Velocity limits of fer_joint1..7 in config/joint_limits.yaml.
  const std::vector<double> limits{2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
  const auto start = arm_->positions();
  MoveToJoints::Goal away;
  away.joints.positions = {0.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  away.speed_scaling = 0.5;
  MoveToJoints::Goal back;
  std::copy(start.begin(), start.end(), back.joints.positions.begin());
  back.speed_scaling = 0.2;

  for (const auto & goal : {away, back}) {
    ASSERT_EQ(run<MoveToJoints>(joints_client_, goal).code, Outcome::OK);
    for (const auto & point : controller_->last().points) {
      for (size_t i = 0; i < limits.size(); ++i) {
        EXPECT_LE(std::abs(point.velocities.at(i)), goal.speed_scaling * limits[i] + 1e-3) <<
          "joint " << i + 1 << " at speed_scaling " << goal.speed_scaling;
      }
    }
  }
}

TEST_F(MoveGroupIntegration, StopCancelsTheControllerGoal)
{
  MoveToJoints::Goal goal;
  goal.joints.positions = {1.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
  goal.speed_scaling = 0.1;
  auto sent = joints_client_->async_send_goal(goal);
  ASSERT_EQ(sent.wait_for(20s), std::future_status::ready);
  auto handle = sent.get();
  auto result = joints_client_->async_get_result(handle);
  ASSERT_TRUE(wait_until([this] {return controller_->playing.load();}, 30s));
  joints_client_->async_cancel_goal(handle);
  ASSERT_EQ(result.wait_for(30s), std::future_status::ready);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
  EXPECT_EQ(controller_->cancels.load(), 1);
}

}  // namespace fer_moveit_config
