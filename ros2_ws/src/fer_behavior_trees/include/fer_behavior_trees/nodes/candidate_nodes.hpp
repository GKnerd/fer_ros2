#ifndef FER_BEHAVIOR_TREES__NODES__CANDIDATE_NODES_HPP_
#define FER_BEHAVIOR_TREES__NODES__CANDIDATE_NODES_HPP_

#include "fer_behavior_trees/nodes/outcome_nodes.hpp"
#include "fer_interfaces/srv/get_grasp_candidates.hpp"
#include "fer_interfaces/srv/get_place_candidates.hpp"

namespace fer_behavior_trees
{
/// \brief `/grasp/candidates`: grasp candidates for a FREE object, best first.
class GetGraspCandidatesNode
  : public OutcomeServiceNode<fer_interfaces::srv::GetGraspCandidates>
{
public:
  using OutcomeServiceNode::OutcomeServiceNode;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr & request) override;

protected:
  void write_outputs(const Response & response) override;
};

/// \brief `/place/candidates`: hand poses that put a GRASPED object down at (x, y, z), turned
/// by `yaw` about the vertical, in `frame`.
class GetPlaceCandidatesNode
  : public OutcomeServiceNode<fer_interfaces::srv::GetPlaceCandidates>
{
public:
  using OutcomeServiceNode::OutcomeServiceNode;
  static BT::PortsList providedPorts();
  bool setRequest(Request::SharedPtr & request) override;

protected:
  void write_outputs(const Response & response) override;
};
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__NODES__CANDIDATE_NODES_HPP_
