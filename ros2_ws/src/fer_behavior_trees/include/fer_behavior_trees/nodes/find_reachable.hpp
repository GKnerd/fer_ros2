#ifndef FER_BEHAVIOR_TREES__NODES__FIND_REACHABLE_HPP_
#define FER_BEHAVIOR_TREES__NODES__FIND_REACHABLE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "fer_interfaces/msg/grasp_candidate.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "fer_interfaces/msg/place_candidate.hpp"
#include "fer_interfaces/srv/check_reachable.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fer_behavior_trees
{
/// \brief Tries the candidates in order with `/motion/check_reachable` and outputs the first
/// one the arm can execute: approach free, then target and retreat straight, the hand
/// allowed to touch `object_id`.
template<class CandidateT>
class FindReachable : public BT::StatefulActionNode
{
public:

  using CheckReachable = fer_interfaces::srv::CheckReachable;
  using Targets = std::array<geometry_msgs::msg::PoseStamped, 3>;

  FindReachable(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params
  );

  /// \brief Ports of the node: \p addition, `object_id`, `candidates` and `outcome`.
  static BT::PortsList base_ports(BT::PortsList addition);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  /// \brief Approach, target and retreat pose of \p candidate.
  virtual Targets targets(const CandidateT & candidate) const = 0;

  /// \brief Writes the outputs for the reachable \p candidate.
  /// \param approach_joints Arm configuration at the approach pose.
  virtual void write_outputs(
    const CandidateT & candidate, const std::vector<double> & approach_joints) = 0;

private:
  void send();
  BT::NodeStatus fail(uint8_t code, const std::string & message);

  rclcpp::Logger logger_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<CheckReachable>::SharedPtr client_;
  std::chrono::milliseconds timeout_;
  std::vector<CandidateT> candidates_;
  std::string object_id_;
  double speed_scaling_{0.3}; 
  size_t index_{0};
  rclcpp::Client<CheckReachable>::SharedFuture future_;
  int64_t request_id_{0};
  bool pending_{false};
  std::chrono::steady_clock::time_point sent_;
};

/// \brief FindReachable over grasp candidates: pre-grasp, grasp, lift.
class FindReachableGrasp : public FindReachable<fer_interfaces::msg::GraspCandidate>
{
public:
  using FindReachable::FindReachable;
  static BT::PortsList providedPorts();

protected:
  Targets targets(const fer_interfaces::msg::GraspCandidate & candidate) const override;
  void write_outputs(
    const fer_interfaces::msg::GraspCandidate & candidate,
    const std::vector<double> & approach_joints) override;
};

/// \brief FindReachable over place candidates: pre-place, place, retreat.
class FindReachablePlace : public FindReachable<fer_interfaces::msg::PlaceCandidate>
{
public:
  using FindReachable::FindReachable;
  static BT::PortsList providedPorts();

protected:
  Targets targets(const fer_interfaces::msg::PlaceCandidate & candidate) const override;
  void write_outputs(
    const fer_interfaces::msg::PlaceCandidate & candidate,
    const std::vector<double> & approach_joints) override;
};
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__NODES__FIND_REACHABLE_HPP_
