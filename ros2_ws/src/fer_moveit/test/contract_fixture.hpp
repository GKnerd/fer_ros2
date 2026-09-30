#ifndef FER_MOVEIT_CONFIG__TEST__CONTRACT_FIXTURE_HPP_
#define FER_MOVEIT_CONFIG__TEST__CONTRACT_FIXTURE_HPP_

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fer_interfaces/action/move_to_joints.hpp"
#include "fer_interfaces/action/move_to_pose.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "fer_interfaces/srv/check_reachable.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fer_moveit_config/adapters/move_group_client.hpp"
#include "fer_moveit_config/adapters/robot_mode_monitor.hpp"
#include "fer_moveit_config/adapters/world_model_client.hpp"
#include "fer_moveit_config/motion_server.hpp"
#include "fakes.hpp"

namespace fer_moveit_config
{

using fer_interfaces::action::MoveToJoints;
using fer_interfaces::action::MoveToPose;
using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::PoseTarget;
using fer_interfaces::msg::WorldObject;
using fer_interfaces::srv::CheckReachable;
using Codes = moveit_msgs::msg::MoveItErrorCodes;
using std::chrono_literals::operator""s;
using std::chrono_literals::operator""ms;

template<typename FutureT>
inline bool wait(const FutureT & future, std::chrono::duration<double> timeout = 10s)
{
  return future.wait_for(timeout) == std::future_status::ready;
}

template<typename Predicate>
inline bool wait_until(Predicate predicate, std::chrono::duration<double> timeout = 10s)
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

/// The real MotionServer with fake move_group, world model and arm in one process.
class Contract : public ::testing::Test
{
protected:
  void start(bool with_world_model = true, bool with_robot_mode = false)
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"world_model_timeout", 0.5},
        {"move_group_timeout", 1.5},
        {"rest_timeout", 2.0},
        {"startup_timeout", 10.0},
      });
    node_ = std::make_shared<rclcpp::Node>("fer_moveit_motion_server", options);
    fakes_ = std::make_shared<rclcpp::Node>("contract_fakes");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    arm_ = std::make_unique<FakeArm>(fakes_);
    move_group_fake_ = std::make_unique<FakeMoveGroup>(fakes_, *arm_);
    if (with_world_model) {
      world_ = std::make_unique<FakeWorldModel>(fakes_);
      world_->set({box("table", WorldObject::FREE, true), box("cup_1")});
    }
    tf_ = camera_frame(fakes_);

    world_model_ = std::make_unique<WorldModelClient>(node_, group_);
    move_group_ = std::make_unique<MoveGroupClient>(node_, group_);
    if (with_robot_mode) {
      robot_mode_ = std::make_unique<RobotModeMonitor>(node_, group_);
    }
    server_ = std::make_unique<MotionServer>(
      node_, *world_model_, *move_group_, robot_mode_.get(), group_);

    pose_client_ = rclcpp_action::create_client<MoveToPose>(fakes_, "/motion/move_to_pose");
    joints_client_ =
      rclcpp_action::create_client<MoveToJoints>(fakes_, "/motion/move_to_joints");
    reachable_client_ = fakes_->create_client<CheckReachable>("/motion/check_reachable");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 8);
    executor_->add_node(node_);
    executor_->add_node(fakes_);
    spinner_ = std::thread([this] {executor_->spin();});

    ASSERT_TRUE(wait_until([this] {return server_->ready();}));
    ASSERT_TRUE(pose_client_->wait_for_action_server(5s));
    ASSERT_TRUE(joints_client_->wait_for_action_server(5s));
    ASSERT_TRUE(reachable_client_->wait_for_service(5s));
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
      spinner_.join();
    }
    server_.reset();
    move_group_fake_.reset();
    rclcpp::shutdown();
  }

  MoveToJoints::Goal joints_goal(double speed = 0.3)
  {
    MoveToJoints::Goal goal;
    goal.joints.positions = {0.1, -0.7, 0.0, -2.3, 0.0, 1.6, 0.8};
    goal.speed_scaling = speed;
    return goal;
  }

  MoveToPose::Goal pose_goal(
    uint8_t path = PoseTarget::PATH_FREE, const std::string & frame = "base",
    const std::vector<std::string> & may_touch = {})
  {
    MoveToPose::Goal goal;
    goal.target.pose.header.frame_id = frame;
    goal.target.pose.pose.position.x = 0.4;
    goal.target.pose.pose.orientation.x = 1.0;
    goal.target.path = path;
    goal.target.may_touch = may_touch;
    goal.speed_scaling = 0.3;
    return goal;
  }

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr send(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal)
  {
    auto future = client->async_send_goal(goal);
    EXPECT_TRUE(wait(future));
    auto handle = future.get();
    EXPECT_TRUE(handle);
    return handle;
  }

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult run(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal)
  {
    auto handle = send<ActionT>(client, goal);
    auto result = client->async_get_result(handle);
    EXPECT_TRUE(wait(result));
    return result.get();
  }

  CheckReachable::Response::SharedPtr check(const std::vector<PoseTarget> & targets)
  {
    auto request = std::make_shared<CheckReachable::Request>();
    request->targets = targets;
    auto future = reachable_client_->async_send_request(request).future.share();
    EXPECT_TRUE(wait(future));
    return future.get();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr fakes_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<FakeArm> arm_;
  std::unique_ptr<FakeMoveGroup> move_group_fake_;
  std::unique_ptr<FakeWorldModel> world_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  std::unique_ptr<WorldModelClient> world_model_;
  std::unique_ptr<MoveGroupClient> move_group_;
  std::unique_ptr<RobotModeMonitor> robot_mode_;
  std::unique_ptr<MotionServer> server_;
  rclcpp_action::Client<MoveToPose>::SharedPtr pose_client_;
  rclcpp_action::Client<MoveToJoints>::SharedPtr joints_client_;
  rclcpp::Client<CheckReachable>::SharedPtr reachable_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
};

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__TEST__CONTRACT_FIXTURE_HPP_
