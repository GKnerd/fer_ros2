#include "fer_behavior_trees/server.hpp"

#include <chrono>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "fer_behavior_trees/nodes/candidate_nodes.hpp"
#include "fer_behavior_trees/nodes/find_reachable.hpp"
#include "fer_behavior_trees/nodes/gripper_nodes.hpp"
#include "fer_behavior_trees/nodes/motion_nodes.hpp"
#include "fer_behavior_trees/nodes/world_model_nodes.hpp"
#include "fer_behavior_trees/payload.hpp"

namespace fer_behavior_trees
{
Server::Server(const rclcpp::Node::SharedPtr & node)
: BT::TreeExecutionServer(node),
  action_timeout_(node->declare_parameter<int64_t>("action_timeout", 2000)),
  service_timeout_(node->declare_parameter<int64_t>("service_timeout", 5000)),
  check_reachable_timeout_(node->declare_parameter<int64_t>("check_reachable_timeout", 60000))
{
  for (const auto & name : POSE_NAMES) {
    const auto joints = node->declare_parameter<std::vector<double>>(
      "poses." + name, std::vector<double>{});
    if (joints.size() != 7) {
      throw std::runtime_error(
              "parameter 'poses." + name + "' needs 7 joint values, got " +
              std::to_string(joints.size()));
    }
    globalBlackboard()->set(name, joints);
  }
}

bool Server::onGoalReceived(const std::string & /*tree_name*/, const std::string & payload)
{
  const std::set<std::string> reserved(POSE_NAMES.begin(), POSE_NAMES.end());
  try {
    payload_keys_ = apply_payload(*globalBlackboard(), payload, payload_keys_, reserved);
  } catch (const PayloadError & e) {
    RCLCPP_ERROR(node()->get_logger(), "goal rejected: %s", e.what());
    return false;
  }
  return true;
}

void Server::registerNodesIntoFactory(BT::BehaviorTreeFactory & factory)
{
  register_nodes(
    factory, node(), action_timeout_, service_timeout_, check_reachable_timeout_);
}

void register_nodes(
  BT::BehaviorTreeFactory & factory, const rclcpp::Node::SharedPtr & node,
  std::chrono::milliseconds action_timeout, std::chrono::milliseconds service_timeout,
  std::chrono::milliseconds check_reachable_timeout)
{
  BT::RosNodeParams params(node);
  params.wait_for_server_timeout = std::chrono::milliseconds(1000);

  params.server_timeout = action_timeout;
  params.default_port_value = "/motion/move_to_pose";
  factory.registerNodeType<MoveToPoseNode>("MoveToPose", params);
  params.default_port_value = "/motion/move_to_joints";
  factory.registerNodeType<MoveToJointsNode>("MoveToJoints", params);
  params.default_port_value = "/gripper/move";
  factory.registerNodeType<MoveGripperNode>("MoveGripper", params);
  params.default_port_value = "/gripper/grasp";
  factory.registerNodeType<GraspNode>("Grasp", params);
  params.default_port_value = "/gripper/release";
  factory.registerNodeType<ReleaseNode>("Release", params);
  params.default_port_value = "/world_model/detect_objects";
  factory.registerNodeType<DetectObjectsNode>("DetectObjects", params);
  params.default_port_value = "/world_model/refine_object";
  factory.registerNodeType<RefineObjectNode>("RefineObject", params);

  params.server_timeout = service_timeout;
  params.default_port_value = "/world_model/query_objects";
  factory.registerNodeType<QueryObjectsNode>("QueryObjects", params);
  params.default_port_value = "/grasp/candidates";
  factory.registerNodeType<GetGraspCandidatesNode>("GetGraspCandidates", params);
  params.default_port_value = "/place/candidates";
  factory.registerNodeType<GetPlaceCandidatesNode>("GetPlaceCandidates", params);

  params.server_timeout = check_reachable_timeout;
  params.default_port_value = "/motion/check_reachable";
  factory.registerNodeType<FindReachableGrasp>("FindReachableGrasp", params);
  factory.registerNodeType<FindReachablePlace>("FindReachablePlace", params);
}
}  // namespace fer_behavior_trees
