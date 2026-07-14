#include "fer_behavior_trees/pick_object_client_node.hpp"


PickObjectSkill::PickObjectSkill(const std::string& name,
                                 const BT::NodeConfig& conf,
                                 const BT::RosNodeParams& params)
  : BT::RosActionNode<PickObject>(name, conf, params)
{
}


BT::PortsList PickObjectSkill::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("object_id", "planning-scene id of the object to pick"),
  });
}

bool PickObjectSkill::setGoal(Goal& goal)
{
  auto object_id = getInput<std::string>("object_id");
  if(!object_id)
  {
    RCLCPP_ERROR(logger(), "%s: missing required input [object_id]: %s",
                 name().c_str(), object_id.error().c_str());
    return false;
  }

  goal.object_id = object_id.value();

  RCLCPP_INFO(logger(), "%s: sending PickObject goal (object_id=%s)",
              name().c_str(), goal.object_id.c_str());
  return true;
}

BT::NodeStatus PickObjectSkill::onResultReceived(const WrappedResult& result)
{
  RCLCPP_INFO(logger(), "%s: result success=%s message='%s'",
              name().c_str(),
              result.result->success ? "true" : "false",
              result.result->message.c_str());

  return result.result->success ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
}

BT::NodeStatus PickObjectSkill::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: phase=%s (%.0f%%)", name().c_str(),
              feedback->current_phase.c_str(), feedback->progress * 100.0f);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PickObjectSkill::onFailure(BT::ActionNodeErrorCode error,
                                          const std::optional<WrappedResult>& result)
{
  RCLCPP_ERROR(logger(), "%s: failure (%s)", name().c_str(), BT::toStr(error));
  if(result && result->result)
  {
    RCLCPP_ERROR(logger(), "%s: server message='%s'",
                 name().c_str(), result->result->message.c_str());
  }
  return BT::NodeStatus::FAILURE;
}

void PickObjectSkill::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted — goal cancelled", name().c_str());
}
