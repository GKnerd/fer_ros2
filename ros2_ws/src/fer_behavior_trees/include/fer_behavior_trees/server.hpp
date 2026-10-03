#ifndef FER_BEHAVIOR_TREES__SERVER_HPP_
#define FER_BEHAVIOR_TREES__SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fer_behavior_trees
{
/// \brief Named arm configurations, parameters `poses.<name>`, on the global blackboard.
const std::vector<std::string> POSE_NAMES{"home", "view"};

/// \brief Behavior tree server of the manipulation trees on fer_interfaces.
///
/// The goal payload (a JSON object) goes to the global blackboard; the named poses are
/// loaded there at startup. Every node status change is printed to stdout.
class Server : public BT::TreeExecutionServer
{
public:
  /// \throws std::runtime_error if a named pose is missing or has not 7 values.
  explicit Server(const rclcpp::Node::SharedPtr & node);

protected:
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override;
  void onTreeCreated(BT::Tree & tree) override;
  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override;

private:
  std::chrono::milliseconds action_timeout_;
  std::chrono::milliseconds service_timeout_;
  std::chrono::milliseconds check_reachable_timeout_;
  std::vector<std::string> payload_keys_;
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
};

/// \brief Registers the fer_interfaces nodes on \p node into \p factory.
void register_nodes(
  BT::BehaviorTreeFactory & factory, const rclcpp::Node::SharedPtr & node,
  std::chrono::milliseconds action_timeout, std::chrono::milliseconds service_timeout,
  std::chrono::milliseconds check_reachable_timeout);
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__SERVER_HPP_
