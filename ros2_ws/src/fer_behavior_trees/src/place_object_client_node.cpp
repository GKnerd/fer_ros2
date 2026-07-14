#include "fer_behavior_trees/place_object_client_node.hpp"


PlaceObjectSkill::PlaceObjectSkill(const std::string& name,
                                   const BT::NodeConfig& conf,
                                   const BT::RosNodeParams& params)
  : BT::RosActionNode<PlaceObject>(name, conf, params)
{
}

// PlaceObject.action goal: `geometry_msgs/PoseStamped place_pose` + `string
// object_id`. There is no string parser for a whole PoseStamped, so the pose is
// exposed as scalar ports (frame_id + position + quaternion). This lets a static
// XML write the target directly, while an upstream node can still override any
// component through the blackboard.
BT::PortsList PlaceObjectSkill::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>("object_id", "planning-scene id of the object being placed"),
    BT::InputPort<std::string>("frame_id", "base", "reference frame of place_pose"),
    BT::InputPort<double>("x", "place position x (m)"),
    BT::InputPort<double>("y", "place position y (m)"),
    BT::InputPort<double>("z", "place position z (m)"),
    BT::InputPort<double>("qx", 0.0, "place orientation x"),
    BT::InputPort<double>("qy", 0.0, "place orientation y"),
    BT::InputPort<double>("qz", 0.0, "place orientation z"),
    BT::InputPort<double>("qw", 1.0, "place orientation w"),
  });
}

bool PlaceObjectSkill::setGoal(Goal& goal)
{
  auto object_id = getInput<std::string>("object_id");
  if(!object_id)
  {
    RCLCPP_ERROR(logger(), "%s: missing required input [object_id]: %s",
                 name().c_str(), object_id.error().c_str());
    return false;
  }

  auto x = getInput<double>("x");
  auto y = getInput<double>("y");
  auto z = getInput<double>("z");
  if(!x || !y || !z)
  {
    RCLCPP_ERROR(logger(), "%s: missing required place position [x/y/z]",
                 name().c_str());
    return false;
  }

  goal.object_id = object_id.value();
  goal.place_pose.header.frame_id = getInput<std::string>("frame_id").value_or("base");
  goal.place_pose.pose.position.x = x.value();
  goal.place_pose.pose.position.y = y.value();
  goal.place_pose.pose.position.z = z.value();
  goal.place_pose.pose.orientation.x = getInput<double>("qx").value_or(0.0);
  goal.place_pose.pose.orientation.y = getInput<double>("qy").value_or(0.0);
  goal.place_pose.pose.orientation.z = getInput<double>("qz").value_or(0.0);
  goal.place_pose.pose.orientation.w = getInput<double>("qw").value_or(1.0);

  RCLCPP_INFO(logger(), "%s: sending PlaceObject goal (object_id=%s, frame=%s, "
              "xyz=[%.3f, %.3f, %.3f])",
              name().c_str(), goal.object_id.c_str(),
              goal.place_pose.header.frame_id.c_str(),
              goal.place_pose.pose.position.x,
              goal.place_pose.pose.position.y,
              goal.place_pose.pose.position.z);
  return true;
}

BT::NodeStatus PlaceObjectSkill::onResultReceived(const WrappedResult& result)
{
  RCLCPP_INFO(logger(), "%s: result success=%s message='%s'",
              name().c_str(),
              result.result->success ? "true" : "false",
              result.result->message.c_str());

  return result.result->success ? BT::NodeStatus::SUCCESS
                                : BT::NodeStatus::FAILURE;
}

BT::NodeStatus PlaceObjectSkill::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: phase=%s (%.0f%%)", name().c_str(),
              feedback->current_phase.c_str(), feedback->progress * 100.0f);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlaceObjectSkill::onFailure(BT::ActionNodeErrorCode error,
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

void PlaceObjectSkill::onHalt()
{
  RCLCPP_INFO(logger(), "%s: halted — goal cancelled", name().c_str());
}
