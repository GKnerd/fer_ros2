#include "fer_behavior_trees/nodes/candidate_nodes.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "fer_interfaces/msg/grasp_candidate.hpp"
#include "fer_interfaces/msg/place_candidate.hpp"

namespace fer_behavior_trees
{
using fer_interfaces::msg::GraspCandidate;
using fer_interfaces::msg::PlaceCandidate;

BT::PortsList GetGraspCandidatesNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<std::string>("object_id", "FREE object to grasp"),
      BT::OutputPort<std::vector<GraspCandidate>>("candidates", "Best first"),
  });
}

bool GetGraspCandidatesNode::setRequest(Request::SharedPtr & request)
{
  const auto object_id = required_input<std::string>(*this, "object_id", logger());
  if (!object_id) {
    return false;
  }
  request->object_id = *object_id;
  return true;
}

void GetGraspCandidatesNode::write_outputs(const Response & response)
{
  setOutput("candidates", response.candidates);
}

BT::PortsList GetPlaceCandidatesNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<std::string>("object_id", "GRASPED object to put down"),
      BT::InputPort<double>("x", "Bottom center of the object at the place, m"),
      BT::InputPort<double>("y", "Bottom center of the object at the place, m"),
      BT::InputPort<double>("z", 0.0, "Height of the surface, m"),
      BT::InputPort<double>("yaw", 0.0, "Object orientation about the vertical, rad"),
      BT::InputPort<std::string>("frame", "base", "Frame of x, y, z and yaw"),
      BT::OutputPort<std::vector<PlaceCandidate>>("candidates", "Best first"),
  });
}

bool GetPlaceCandidatesNode::setRequest(Request::SharedPtr & request)
{
  const auto object_id = required_input<std::string>(*this, "object_id", logger());
  const auto x = required_input<double>(*this, "x", logger());
  const auto y = required_input<double>(*this, "y", logger());
  if (!object_id || !x || !y) {
    return false;
  }
  const double yaw = getInput<double>("yaw").value_or(0.0);
  request->object_id = *object_id;
  request->target.header.frame_id = getInput<std::string>("frame").value_or("base");
  request->target.pose.position.x = *x;
  request->target.pose.position.y = *y;
  request->target.pose.position.z = getInput<double>("z").value_or(0.0);
  request->target.pose.orientation.z = std::sin(yaw / 2.0);
  request->target.pose.orientation.w = std::cos(yaw / 2.0);
  return true;
}

void GetPlaceCandidatesNode::write_outputs(const Response & response)
{
  setOutput("candidates", response.candidates);
}
}  // namespace fer_behavior_trees
