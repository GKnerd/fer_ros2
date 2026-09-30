#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "contract_fixture.hpp"

namespace fer_moveit_config
{

TEST_F(Contract, SpeedScalingOutsideZeroToOneIsInvalid)
{
  start();
  for (const double speed : {0.0, 1.5}) {
    const auto result = run<MoveToJoints>(joints_client_, joints_goal(speed));
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(result.result->outcome.code, Outcome::INVALID_GOAL);
  }
  EXPECT_TRUE(move_group_fake_->plan_goals.empty());
}

TEST_F(Contract, UnknownFrameIsInvalid)
{
  start();
  const auto result = run<MoveToPose>(pose_client_, pose_goal(PoseTarget::PATH_FREE, "nowhere"));
  EXPECT_EQ(result.result->outcome.code, Outcome::INVALID_GOAL);
}

TEST_F(Contract, UnknownMayTouchIdIsNotFound)
{
  start();
  const auto result =
    run<MoveToPose>(pose_client_, pose_goal(PoseTarget::PATH_STRAIGHT, "base", {"cup_7"}));
  EXPECT_EQ(result.result->outcome.code, Outcome::NOT_FOUND);
}

TEST_F(Contract, NoWorldModelIsTimeout)
{
  start(false);
  const auto result = run<MoveToJoints>(joints_client_, joints_goal());
  EXPECT_EQ(result.result->outcome.code, Outcome::TIMEOUT);
}

TEST_F(Contract, SlowMoveGroupIsTimeout)
{
  start();
  move_group_fake_->plan_delay = 3.0;
  const auto result = run<MoveToJoints>(joints_client_, joints_goal());
  EXPECT_EQ(result.result->outcome.code, Outcome::TIMEOUT);
}

TEST_F(Contract, JointTargetIsPlannedWithOmplAndExecutedOnTheController)
{
  start();
  const auto result = run<MoveToJoints>(joints_client_, joints_goal(0.3));
  ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(result.result->outcome.code, Outcome::OK);

  std::lock_guard<std::mutex> lock(move_group_fake_->mutex);
  ASSERT_EQ(move_group_fake_->plan_goals.size(), 1u);
  const auto & goal = move_group_fake_->plan_goals[0];
  EXPECT_TRUE(goal.planning_options.plan_only);
  EXPECT_EQ(goal.request.group_name, "fer_arm");
  EXPECT_EQ(goal.request.pipeline_id, "ompl");
  EXPECT_DOUBLE_EQ(goal.request.max_velocity_scaling_factor, 0.3);
  EXPECT_DOUBLE_EQ(goal.request.max_acceleration_scaling_factor, 0.3);
  ASSERT_EQ(goal.request.goal_constraints.size(), 1u);
  ASSERT_EQ(goal.request.goal_constraints[0].joint_constraints.size(), 7u);
  EXPECT_DOUBLE_EQ(goal.request.goal_constraints[0].joint_constraints[0].position, 0.1);
  ASSERT_EQ(move_group_fake_->execute_goals.size(), 1u);
  // No controller named: move_group uses its default arm controller.
  EXPECT_TRUE(move_group_fake_->execute_goals[0].controller_names.empty());
}

TEST_F(Contract, FreeAndStraightPosesUseTheirPipelines)
{
  start();
  EXPECT_EQ(
    run<MoveToPose>(pose_client_, pose_goal(PoseTarget::PATH_FREE)).result->outcome.code,
    Outcome::OK);
  EXPECT_EQ(
    run<MoveToPose>(
      pose_client_, pose_goal(PoseTarget::PATH_STRAIGHT, "camera")).result->outcome.code,
    Outcome::OK);

  std::lock_guard<std::mutex> lock(move_group_fake_->mutex);
  ASSERT_EQ(move_group_fake_->plan_goals.size(), 2u);
  const auto & free = move_group_fake_->plan_goals[0].request;
  EXPECT_EQ(free.pipeline_id, "ompl");
  EXPECT_EQ(free.planner_id, "");
  const auto & straight = move_group_fake_->plan_goals[1].request;
  EXPECT_EQ(straight.pipeline_id, "pilz_industrial_motion_planner");
  EXPECT_EQ(straight.planner_id, "LIN");
  const auto & position = straight.goal_constraints.at(0).position_constraints.at(0);
  EXPECT_EQ(position.link_name, "fer_hand_tcp");
  EXPECT_EQ(position.header.frame_id, "base");
  // camera is 1 m above base.
  EXPECT_NEAR(position.constraint_region.primitive_poses.at(0).position.z, 1.0, 1e-9);
}

TEST_F(Contract, SceneFollowsTheWorldModel)
{
  start();
  ASSERT_EQ(run<MoveToJoints>(joints_client_, joints_goal()).result->outcome.code, Outcome::OK);
  world_->set({box("table", WorldObject::FREE, true), box("cup_1", WorldObject::GRASPED)});
  ASSERT_EQ(run<MoveToJoints>(joints_client_, joints_goal()).result->outcome.code, Outcome::OK);
  world_->set({box("table", WorldObject::FREE, true)});
  ASSERT_EQ(run<MoveToJoints>(joints_client_, joints_goal()).result->outcome.code, Outcome::OK);

  std::lock_guard<std::mutex> lock(move_group_fake_->mutex);
  const auto & scenes = move_group_fake_->applied_scenes;
  ASSERT_EQ(scenes.size(), 3u);
  EXPECT_TRUE(scenes[0].is_diff);
  EXPECT_EQ(scenes[0].world.collision_objects.size(), 2u);
  EXPECT_TRUE(scenes[0].robot_state.attached_collision_objects.empty());

  const auto & held = scenes[1].robot_state.attached_collision_objects;
  ASSERT_EQ(held.size(), 1u);
  EXPECT_EQ(held[0].object.id, "cup_1");
  EXPECT_EQ(held[0].link_name, "fer_hand_tcp");
  EXPECT_EQ(held[0].object.operation, moveit_msgs::msg::CollisionObject::ADD);
  EXPECT_EQ(
    held[0].touch_links,
    (std::vector<std::string>{"fer_hand", "fer_leftfinger", "fer_rightfinger"}));

  const auto & detached = scenes[2].robot_state.attached_collision_objects;
  ASSERT_EQ(detached.size(), 1u);
  EXPECT_EQ(detached[0].object.operation, moveit_msgs::msg::CollisionObject::REMOVE);
  const auto & world = scenes[2].world.collision_objects;
  EXPECT_TRUE(
    std::any_of(
      world.begin(), world.end(), [](const auto & o) {
        return o.id == "cup_1" && o.operation == moveit_msgs::msg::CollisionObject::REMOVE;
      }));
}

TEST_F(Contract, MayTouchAllowsTheHandForThisRequestOnly)
{
  start();
  const auto result =
    run<MoveToPose>(pose_client_, pose_goal(PoseTarget::PATH_STRAIGHT, "base", {"cup_1"}));
  ASSERT_EQ(result.result->outcome.code, Outcome::OK);

  std::lock_guard<std::mutex> lock(move_group_fake_->mutex);
  const auto & options = move_group_fake_->plan_goals.at(0).planning_options;
  const auto & matrix = options.planning_scene_diff.allowed_collision_matrix;
  auto index = [&matrix](const std::string & name) {
      return static_cast<size_t>(
        std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name) -
        matrix.entry_names.begin());
    };
  ASSERT_LT(index("cup_1"), matrix.entry_names.size());
  for (const auto & link : {"fer_hand", "fer_leftfinger", "fer_rightfinger"}) {
    ASSERT_LT(index(link), matrix.entry_names.size());
    EXPECT_TRUE(matrix.entry_values[index("cup_1")].enabled[index(link)]);
    EXPECT_TRUE(matrix.entry_values[index(link)].enabled[index("cup_1")]);
  }
  // The allowances already in move_group's matrix are kept.
  EXPECT_TRUE(matrix.entry_values[index("fer_link0")].enabled[index("fer_link1")]);
  for (const auto & scene : move_group_fake_->applied_scenes) {
    EXPECT_TRUE(scene.allowed_collision_matrix.entry_names.empty());
  }
}

TEST_F(Contract, PlanningErrorsMapToUnreachableAndNoPath)
{
  start();
  move_group_fake_->plan_error = Codes::NO_IK_SOLUTION;
  EXPECT_EQ(
    run<MoveToPose>(pose_client_, pose_goal()).result->outcome.code, Outcome::UNREACHABLE);
  move_group_fake_->plan_error = Codes::GOAL_IN_COLLISION;
  EXPECT_EQ(
    run<MoveToPose>(pose_client_, pose_goal()).result->outcome.code, Outcome::UNREACHABLE);
  move_group_fake_->plan_error = Codes::PLANNING_FAILED;
  EXPECT_EQ(run<MoveToPose>(pose_client_, pose_goal()).result->outcome.code, Outcome::NO_PATH);
  EXPECT_TRUE(move_group_fake_->execute_goals.empty());
}

TEST_F(Contract, ControllerFailureIsExecutionFailed)
{
  start();
  move_group_fake_->execute_error = Codes::CONTROL_FAILED;
  const auto result = run<MoveToJoints>(joints_client_, joints_goal());
  EXPECT_EQ(result.result->outcome.code, Outcome::EXECUTION_FAILED);
  EXPECT_NE(result.result->outcome.message.find("arm controller"), std::string::npos);
}

TEST_F(Contract, CheckReachableChainsTargetsWithoutMoving)
{
  start();
  const auto before = arm_->positions();
  PoseTarget first = pose_goal().target;
  PoseTarget second = pose_goal(PoseTarget::PATH_STRAIGHT, "base", {"cup_1"}).target;
  const auto response = check({first, second});
  ASSERT_EQ(response->outcome.code, Outcome::OK);
  ASSERT_EQ(response->configurations.size(), 2u);
  EXPECT_NEAR(response->configurations[0].positions[0], before[0] + 0.1, 1e-9);
  EXPECT_NEAR(response->configurations[1].positions[0], before[0] + 0.2, 1e-9);

  std::lock_guard<std::mutex> lock(move_group_fake_->mutex);
  ASSERT_EQ(move_group_fake_->plan_goals.size(), 2u);
  EXPECT_NEAR(
    move_group_fake_->plan_goals[0].request.start_state.joint_state.position.at(0), before[0],
    1e-6);
  EXPECT_NEAR(
    move_group_fake_->plan_goals[1].request.start_state.joint_state.position.at(0),
    before[0] + 0.1, 1e-9);
  EXPECT_TRUE(move_group_fake_->execute_goals.empty());
}

TEST_F(Contract, CheckReachableReportsTheFailedTarget)
{
  start();
  move_group_fake_->plan_error = Codes::NO_IK_SOLUTION;
  move_group_fake_->fail_plan_call = 1;
  const auto response = check({pose_goal().target, pose_goal().target, pose_goal().target});
  EXPECT_EQ(response->outcome.code, Outcome::UNREACHABLE);
  EXPECT_EQ(response->failed_index, 1u);
  EXPECT_EQ(response->configurations.size(), 1u);
}

TEST_F(Contract, NewGoalReplacesTheRunningOneAfterRest)
{
  start();
  move_group_fake_->trajectory_duration = 3.0;
  auto first = send<MoveToJoints>(joints_client_, joints_goal());
  auto first_result = joints_client_->async_get_result(first);
  ASSERT_TRUE(wait_until([this] {return move_group_fake_->executing.load();}));

  move_group_fake_->trajectory_duration = 0.3;
  const auto replaced_at = std::chrono::steady_clock::now();
  auto second = send<MoveToJoints>(joints_client_, joints_goal());
  auto second_result = joints_client_->async_get_result(second);

  ASSERT_TRUE(wait(first_result));
  const auto first_done = std::chrono::steady_clock::now();
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(first_result.get().result->outcome.code, Outcome::CANCELLED);
  // CANCELLED only once the arm is at rest (the fake decelerates for rest_delay).
  EXPECT_GE(first_done - replaced_at, 250ms);
  ASSERT_TRUE(wait(second_result));
  EXPECT_EQ(second_result.get().result->outcome.code, Outcome::OK);
  EXPECT_EQ(move_group_fake_->stops.load(), 1);
}

TEST_F(Contract, ClientCancelStopsTheArm)
{
  start();
  move_group_fake_->trajectory_duration = 3.0;
  auto handle = send<MoveToJoints>(joints_client_, joints_goal());
  auto result = joints_client_->async_get_result(handle);
  ASSERT_TRUE(wait_until([this] {return move_group_fake_->executing.load();}));
  const auto cancelled_at = std::chrono::steady_clock::now();
  joints_client_->async_cancel_goal(handle);

  ASSERT_TRUE(wait(result));
  EXPECT_GE(std::chrono::steady_clock::now() - cancelled_at, 250ms);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
  EXPECT_EQ(move_group_fake_->stops.load(), 1);
}

}  // namespace fer_moveit_config
