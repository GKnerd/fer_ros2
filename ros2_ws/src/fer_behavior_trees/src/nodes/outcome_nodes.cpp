#include "fer_behavior_trees/nodes/outcome_nodes.hpp"

#include <cstdint>

namespace fer_behavior_trees
{
uint8_t outcome_code(BT::ActionNodeErrorCode error)
{
  switch (error) {
    case BT::INVALID_GOAL:
    case BT::GOAL_REJECTED_BY_SERVER:
      return Outcome::INVALID_GOAL;
    case BT::ACTION_CANCELLED:
      return Outcome::CANCELLED;
    default:
      return Outcome::TIMEOUT;
  }
}

uint8_t outcome_code(BT::ServiceNodeErrorCode error)
{
  return error == BT::INVALID_REQUEST ? Outcome::INVALID_GOAL : Outcome::TIMEOUT;
}

BT::NodeStatus finish(
  BT::TreeNode & node, const Outcome & outcome, const rclcpp::Logger & logger)
{
  node.setOutput("outcome", static_cast<int>(outcome.code));
  if (outcome.code == Outcome::OK) {
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_WARN(
    logger, "%s: outcome %d: %s", node.name().c_str(), outcome.code, outcome.message.c_str());
  return BT::NodeStatus::FAILURE;
}
}  // namespace fer_behavior_trees
