#include "fer_behavior_trees/nodes/gripper_nodes.hpp"

#include <string>

namespace fer_behavior_trees
{
BT::PortsList MoveGripperNode::providedPorts()
{
  return outcome_ports({BT::InputPort<double>("width", "Full opening, m")});
}

bool MoveGripperNode::setGoal(Goal & goal)
{
  const auto width = required_input<double>(*this, "width", logger());
  if (!width) {
    return false;
  }
  goal.width = *width;
  return true;
}

BT::PortsList GraspNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<std::string>("object_id", "FREE object to grasp"),
      BT::InputPort<double>("width", "Expected width of the object between the fingers, m"),
      BT::InputPort<double>("force", "N"),
      BT::InputPort<double>("tolerance", 0.005, "Allowed width difference, m"),
  });
}

bool GraspNode::setGoal(Goal & goal)
{
  const auto object_id = required_input<std::string>(*this, "object_id", logger());
  const auto width = required_input<double>(*this, "width", logger());
  const auto force = required_input<double>(*this, "force", logger());
  if (!object_id || !width || !force) {
    return false;
  }
  goal.object_id = *object_id;
  goal.width = *width;
  goal.force = *force;
  goal.tolerance = getInput<double>("tolerance").value_or(0.005);
  return true;
}

BT::PortsList ReleaseNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<std::string>("object_id", "GRASPED object to release"),
      BT::InputPort<double>("open_width", 0.08, "m"),
  });
}

bool ReleaseNode::setGoal(Goal & goal)
{
  const auto object_id = required_input<std::string>(*this, "object_id", logger());
  if (!object_id) {
    return false;
  }
  goal.object_id = *object_id;
  goal.open_width = getInput<double>("open_width").value_or(0.08);
  return true;
}
}  // namespace fer_behavior_trees
