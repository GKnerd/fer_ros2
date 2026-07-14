#include "fer_behavior_trees/control_gripper_client_node.hpp"


ControlGripperSkill::ControlGripperSkill(const std::string& name,
                                         const BT::NodeConfig& conf,
                                         const BT::RosNodeParams& params)
  : BT::RosActionNode<ControlGripper>(name, conf, params)
{
}

// ControlGripper.action goal: `control_msgs/GripperCommand gripper_command`,
// i.e. a target finger `position` (m) and a `max_effort` (N). Both are exposed
// as ports so an open/close command can be written straight into the XML.
BT::PortsList ControlGripperSkill::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("position", "target finger gap (m)"),
    BT::InputPort<double>("max_effort", 20.0, "max grasp effort (N)"),
  });
}

bool ControlGripperSkill::setGoal(Goal& goal)
{
  auto position = getInput<double>("position");
  if(!position)
  {
    RCLCPP_ERROR(logger(), "%s: missing required input [position]: %s",
                 name().c_str(), position.error().c_str());
    return false;
  }

  goal.gripper_command.position = position.value();
  goal.gripper_command.max_effort = getInput<double>("max_effort").value_or(20.0);

  RCLCPP_INFO(logger(), "%s: sending ControlGripper goal (position=%.3f, max_effort=%.1f)",
              name().c_str(), goal.gripper_command.position,
              goal.gripper_command.max_effort);
  return true;
}

BT::NodeStatus ControlGripperSkill::onResultReceived(const WrappedResult& result)
{
  RCLCPP_INFO(logger(), "%s: result success=%s message='%s'",
              name().c_str(),
              result.result->success ? "true" : "false",
              result.result->message.c_str());

  return result.result->success ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ControlGripperSkill::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: %s (%.0f%%)", name().c_str(),
              feedback->status.c_str(), feedback->progress * 100.0f);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ControlGripperSkill::onFailure(BT::ActionNodeErrorCode error,
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

void ControlGripperSkill::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted — goal cancelled", name().c_str());
}
