#include "fer_behavior_trees/nodes/motion_nodes.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "fer_interfaces/msg/pose_target.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace fer_behavior_trees
{
using fer_interfaces::msg::PoseTarget;
using geometry_msgs::msg::PoseStamped;

BT::PortsList MoveToPoseNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<PoseStamped>("pose", "Pose of fer_hand_tcp"),
      BT::InputPort<std::string>("path", "free", "'free' or 'straight'"),
      BT::InputPort<std::string>("may_touch", "", "Object id the hand may touch; empty: none"),
      BT::InputPort<double>("speed_scaling", 0.3, "(0, 1]"),
  });
}

bool MoveToPoseNode::setGoal(Goal & goal)
{
  const auto pose = required_input<PoseStamped>(*this, "pose", logger());
  const auto path = required_input<std::string>(*this, "path", logger());
  if (!pose || !path) {
    return false;
  }
  if (*path == "free") {
    goal.target.path = PoseTarget::PATH_FREE;
  } else if (*path == "straight") {
    goal.target.path = PoseTarget::PATH_STRAIGHT;
  } else {
    RCLCPP_ERROR(logger(), "%s: path '%s' is not 'free' or 'straight'", name().c_str(),
      path->c_str());
    return false;
  }
  goal.target.pose = *pose;
  const auto may_touch = getInput<std::string>("may_touch").value_or("");
  if (!may_touch.empty()) {
    goal.target.may_touch.push_back(may_touch);
  }
  goal.speed_scaling = getInput<double>("speed_scaling").value_or(0.3);
  return true;
}

BT::PortsList MoveToJointsNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<std::vector<double>>("joints", "Joint 1 to 7, rad"),
      BT::InputPort<double>("speed_scaling", 0.3, "(0, 1]"),
  });
}

bool MoveToJointsNode::setGoal(Goal & goal)
{
  const auto joints = required_input<std::vector<double>>(*this, "joints", logger());
  if (!joints) {
    return false;
  }
  if (joints->size() != goal.joints.positions.size()) {
    RCLCPP_ERROR(logger(), "%s: %zu joint values, need %zu", name().c_str(), joints->size(),
      goal.joints.positions.size());
    return false;
  }
  std::copy(joints->begin(), joints->end(), goal.joints.positions.begin());
  goal.speed_scaling = getInput<double>("speed_scaling").value_or(0.3);
  return true;
}
}  // namespace fer_behavior_trees
