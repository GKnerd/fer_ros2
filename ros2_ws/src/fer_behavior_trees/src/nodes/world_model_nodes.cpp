#include "fer_behavior_trees/nodes/world_model_nodes.hpp"

#include <string>
#include <vector>

#include "fer_interfaces/msg/world_object.hpp"

namespace fer_behavior_trees
{
using fer_interfaces::msg::WorldObject;

BT::PortsList DetectObjectsNode::providedPorts()
{
  return outcome_ports({BT::InputPort<std::string>("class_id", "", "Empty: all classes")});
}

bool DetectObjectsNode::setGoal(Goal & goal)
{
  const auto class_id = getInput<std::string>("class_id").value_or("");
  if (!class_id.empty()) {
    goal.class_ids.push_back(class_id);
  }
  return true;
}

BT::PortsList RefineObjectNode::providedPorts()
{
  return outcome_ports({BT::InputPort<std::string>("object_id", "Object to re-detect")});
}

bool RefineObjectNode::setGoal(Goal & goal)
{
  const auto object_id = required_input<std::string>(*this, "object_id", logger());
  if (!object_id) {
    return false;
  }
  goal.object_id = *object_id;
  return true;
}

BT::PortsList QueryObjectsNode::providedPorts()
{
  return outcome_ports(
    {
      BT::InputPort<std::string>("class_id", "", "Empty: all classes"),
      BT::OutputPort<std::vector<std::string>>("ids", "Ids of the matching objects"),
  });
}

bool QueryObjectsNode::setRequest(Request::SharedPtr & request)
{
  const auto class_id = getInput<std::string>("class_id").value_or("");
  if (!class_id.empty()) {
    request->class_ids.push_back(class_id);
  }
  request->statuses.push_back(WorldObject::FREE);
  request->include_fixed = false;
  return true;
}

void QueryObjectsNode::write_outputs(const Response & response)
{
  std::vector<std::string> ids;
  for (const auto & object : response.objects) {
    ids.push_back(object.id);
  }
  setOutput("ids", ids);
}
}  // namespace fer_behavior_trees
