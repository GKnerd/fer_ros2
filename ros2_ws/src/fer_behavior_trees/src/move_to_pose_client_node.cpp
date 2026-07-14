#include "fer_behavior_trees/move_to_pose_client_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>


MoveToPoseSkill::MoveToPoseSkill(const std::string& name,
                                 const BT::NodeConfig& conf,
                                 const BT::RosNodeParams& params)
  : BT::RosActionNode<MoveToPose>(name, conf, params)
{
}

BT::PortsList MoveToPoseSkill::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "TCP goal pose"),
    BT::InputPort<float>("velocity_scaling", 0.5f, "0..1 velocity scaling"),
    BT::InputPort<float>("acceleration_scaling", 0.5f, "0..1 acceleration scaling"),
  });
}

bool MoveToPoseSkill::setGoal(Goal& goal)
{
  auto pose = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  if(!pose)
  {
    RCLCPP_ERROR(logger(), "%s: missing required input [target_pose]: %s",
                 name().c_str(), pose.error().c_str());
    return false;  // -> onFailure(INVALID_GOAL)
  }

  goal.target_pose = pose.value();
  goal.velocity_scaling = getInput<float>("velocity_scaling").value_or(0.5f);
  goal.acceleration_scaling = getInput<float>("acceleration_scaling").value_or(0.5f);

  RCLCPP_INFO(logger(), "%s: sending MoveToPose goal (frame=%s)",
              name().c_str(), goal.target_pose.header.frame_id.c_str());
  return true;
}

BT::NodeStatus MoveToPoseSkill::onResultReceived(const WrappedResult& result)
{
  RCLCPP_INFO(logger(), "%s: result success=%s message='%s'",
              name().c_str(),
              result.result->success ? "true" : "false",
              result.result->message.c_str());

  return result.result->success ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToPoseSkill::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: %s (%.0f%%)", name().c_str(),
              feedback->status.c_str(), feedback->progress * 100.0f);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPoseSkill::onFailure(BT::ActionNodeErrorCode error,
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

void MoveToPoseSkill::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted — goal cancelled", name().c_str());
}
