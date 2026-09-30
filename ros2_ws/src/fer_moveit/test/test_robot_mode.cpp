// Built only when franka_msgs is found (hardware:=real).
#include <gtest/gtest.h>

#include <string>

#include "franka_msgs/msg/franka_state.hpp"

#include "contract_fixture.hpp"

namespace fer_moveit_config
{

TEST_F(Contract, ReflexIsRobotError)
{
  start(true, true);
  auto publisher = fakes_->create_publisher<franka_msgs::msg::FrankaState>(
    "/franka_robot_state_broadcaster/robot_state", 10);
  franka_msgs::msg::FrankaState state;
  state.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_REFLEX;
  state.current_errors.joint_reflex = true;
  ASSERT_TRUE(
    wait_until(
      [&] {
        publisher->publish(state);
        return robot_mode_->error().has_value();
      }));
  const auto result = run<MoveToJoints>(joints_client_, joints_goal());
  EXPECT_EQ(result.result->outcome.code, Outcome::ROBOT_ERROR);
  EXPECT_NE(result.result->outcome.message.find("joint_reflex"), std::string::npos);
  EXPECT_TRUE(move_group_fake_->plan_goals.empty());
}

}  // namespace fer_moveit_config
