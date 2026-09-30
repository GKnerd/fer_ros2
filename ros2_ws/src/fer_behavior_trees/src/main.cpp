#include <chrono>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "fer_behavior_trees/server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fer_bt_server");
  std::unique_ptr<fer_behavior_trees::Server> server;
  try {
    server = std::make_unique<fer_behavior_trees::Server>(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Timeout: upstream's workaround for a deadlock when entities are added while spinning.
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
