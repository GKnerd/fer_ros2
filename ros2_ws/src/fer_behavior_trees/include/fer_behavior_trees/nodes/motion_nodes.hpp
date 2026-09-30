#ifndef FER_BEHAVIOR_TREES__NODES__MOTION_NODES_HPP_
#define FER_BEHAVIOR_TREES__NODES__MOTION_NODES_HPP_

#include <string>

#include "fer_behavior_trees/nodes/outcome_nodes.hpp"
#include "fer_interfaces/action/move_to_joints.hpp"
#include "fer_interfaces/action/move_to_pose.hpp"

namespace fer_behavior_trees
{
/// \brief `/motion/move_to_pose`: moves `fer_hand_tcp` to `pose`, freely or in a straight line.
class MoveToPoseNode : public OutcomeActionNode<fer_interfaces::action::MoveToPose>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};

/// \brief `/motion/move_to_joints`: moves the arm to a 7-joint configuration.
class MoveToJointsNode : public OutcomeActionNode<fer_interfaces::action::MoveToJoints>
{
public:
  using OutcomeActionNode::OutcomeActionNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal & goal) override;
};
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__NODES__MOTION_NODES_HPP_
