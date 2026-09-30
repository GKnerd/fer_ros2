#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <limits>
#include <vector>

#include "fer_moveit_config/core/checks.hpp"

namespace fer_moveit_config
{

TEST(Checks, SpeedScalingMustLieInZeroToOne)
{
  EXPECT_FALSE(speed_valid(0.0));
  EXPECT_FALSE(speed_valid(-0.1));
  EXPECT_TRUE(speed_valid(1e-6));
  EXPECT_TRUE(speed_valid(1.0));
  EXPECT_FALSE(speed_valid(1.0001));
}

TEST(Checks, AtRestWhenEveryJointIsSlow)
{
  EXPECT_TRUE(at_rest({0.0, 0.005, -0.009}, 0.01));
  EXPECT_FALSE(at_rest({0.0, -0.02, 0.0}, 0.01));
  EXPECT_FALSE(at_rest({std::numeric_limits<double>::infinity()}, 0.01));
}

TEST(Checks, ProgressIsClamped)
{
  EXPECT_DOUBLE_EQ(progress(0.5, 2.0), 0.25);
  EXPECT_DOUBLE_EQ(progress(3.0, 2.0), 1.0);
  EXPECT_DOUBLE_EQ(progress(-1.0, 2.0), 0.0);
  EXPECT_DOUBLE_EQ(progress(1.0, 0.0), 1.0);
}

TEST(Checks, WaitForReturnsFalseOnTimeout)
{
  std::promise<int> promise;
  auto future = promise.get_future().share();
  EXPECT_FALSE(wait_for(future, std::chrono::milliseconds(120)));
  promise.set_value(1);
  EXPECT_TRUE(wait_for(future, std::chrono::milliseconds(120)));
}

TEST(Checks, WaitForThrowsWhenInterrupted)
{
  std::promise<int> promise;
  auto future = promise.get_future().share();
  EXPECT_THROW(
    wait_for(future, std::chrono::seconds(5), [] {return true;}), Interrupted);
}

}  // namespace fer_moveit_config
