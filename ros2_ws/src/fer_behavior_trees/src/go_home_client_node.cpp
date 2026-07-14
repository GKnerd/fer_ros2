#include "fer_behavior_trees/go_home_client_node.hpp"



GoHomeSkill::GoHomeSkill(const std::string& name, 
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : 
    BT::RosActionNode<GoHome>(name, conf, params)
    {
        // Constructor Init
    };

// Called once, at the first tick, to fill the goal before it is sent.
// GoHome.action has NO goal fields (the skill reads the "ready" pose from the
// SRDF), so there is nothing to set. Returning true == "go ahead and send it";
// returning false would short-circuit to onFailure(INVALID_GOAL).
bool GoHomeSkill::setGoal(Goal& goal)
{
  (void)goal;  // empty goal — silence -Wunused-parameter
  RCLCPP_INFO(logger(), "%s: sending GoHome goal", name().c_str());
  return true;
};

// Called when the tree halts this node while it is RUNNING (e.g. a reactive
// branch took over).
void GoHomeSkill::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted — goal cancelled", name().c_str());
};

BT::NodeStatus GoHomeSkill::onResultReceived(const WrappedResult& result)
{
  RCLCPP_INFO(logger(), "%s: result success=%s message='%s'",
              name().c_str(),
              result.result->success ? "true" : "false",
              result.result->message.c_str());

  return result.result->success ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
};

BT::NodeStatus GoHomeSkill::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: feedback status='%s'",
              name().c_str(), feedback->status.c_str());
  return BT::NodeStatus::RUNNING;
};

BT::NodeStatus GoHomeSkill::onFailure(
            BT::ActionNodeErrorCode error,
            const std::optional<WrappedResult>& result
        )
{
  RCLCPP_ERROR(logger(), "%s: failure (%s)", name().c_str(), BT::toStr(error));
  if(result && result->result)
  {
    RCLCPP_ERROR(logger(), "%s: server message='%s'",
                 name().c_str(), result->result->message.c_str());
  }
  return BT::NodeStatus::FAILURE;
};
