#ifndef FER_BEHAVIOR_TREES__NODES__GRIPPER_NODES_HPP_
#define FER_BEHAVIOR_TREES__NODES__GRIPPER_NODES_HPP_

#include "fer_behavior_trees/nodes/outcome_nodes.hpp"
#include "fer_interfaces/action/grasp.hpp"
#include "fer_interfaces/action/move_gripper.hpp"
#include "fer_interfaces/action/release.hpp"

namespace fer_behavior_trees
{
/// \brief `/gripper/move`: opens or closes the hand to `width`.
class MoveGripperNode : public OutcomeActionNode<fer_interfaces::action::MoveGripper>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};

/// \brief `/gripper/grasp`: grasps a FREE object.
class GraspNode : public OutcomeActionNode<fer_interfaces::action::Grasp>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};

/// \brief `/gripper/release`: releases a GRASPED object.
class ReleaseNode : public OutcomeActionNode<fer_interfaces::action::Release>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__NODES__GRIPPER_NODES_HPP_
