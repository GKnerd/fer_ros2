#include <memory>
#include <stdexcept>
#include <string>

#include "fer_moveit/adapters/move_group_client.hpp"
#include "fer_moveit/adapters/robot_mode_monitor.hpp"
#include "fer_moveit/adapters/world_model_client.hpp"
#include "fer_moveit/motion_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fer_moveit_motion_server");
  const auto hardware = node->declare_parameter("hardware", std::string("mujoco"));
  if (hardware != "real" && hardware != "mujoco" && hardware != "mock")
  {
    RCLCPP_FATAL(
      node->get_logger(), "hardware must be 'real', 'mujoco' or 'mock', got '%s'",
      hardware.c_str());
    rclcpp::shutdown();
    return 1;
  }
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  fer_moveit::WorldModelClient world_model(node, group);
  fer_moveit::MoveGroupClient move_group(node, group);
  std::unique_ptr<fer_moveit::RobotModeMonitor> robot_mode;
  if (hardware == "real")
  {
    try
    {
      robot_mode = std::make_unique<fer_moveit::RobotModeMonitor>(node, group);
    }
    catch (const std::runtime_error & e)
    {
      RCLCPP_FATAL(node->get_logger(), "%s", e.what());
      rclcpp::shutdown();
      return 1;
    }
  }
  fer_moveit::MotionServer server(node, world_model, move_group, robot_mode.get(), group);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
