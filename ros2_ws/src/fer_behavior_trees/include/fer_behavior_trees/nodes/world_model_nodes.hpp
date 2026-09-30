#ifndef FER_BEHAVIOR_TREES__NODES__WORLD_MODEL_NODES_HPP_
#define FER_BEHAVIOR_TREES__NODES__WORLD_MODEL_NODES_HPP_

#include "fer_behavior_trees/nodes/outcome_nodes.hpp"
#include "fer_interfaces/action/detect_objects.hpp"
#include "fer_interfaces/action/refine_object.hpp"
#include "fer_interfaces/srv/query_objects.hpp"

namespace fer_behavior_trees
{
/// \brief `/world_model/detect_objects`: one detection of `class_id` (empty: all classes).
class DetectObjectsNode : public OutcomeActionNode<fer_interfaces::action::DetectObjects>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};

/// \brief `/world_model/refine_object`: re-detects one object.
class RefineObjectNode : public OutcomeActionNode<fer_interfaces::action::RefineObject>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};

/// \brief `/world_model/query_objects`: ids of the FREE, non-fixed objects of `class_id`.
class QueryObjectsNode : public OutcomeServiceNode<fer_interfaces::srv::QueryObjects>
{
public:
  using OutcomeServiceNode::OutcomeServiceNode;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr & request) override;

protected:
  void write_outputs(const Response & response) override;
};
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__NODES__WORLD_MODEL_NODES_HPP_
