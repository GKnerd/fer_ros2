#include "fer_behavior_trees/nodes/find_reachable.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "fer_interfaces/msg/pose_target.hpp"

namespace fer_behavior_trees
{
using fer_interfaces::msg::GraspCandidate;
using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::PlaceCandidate;
using fer_interfaces::msg::PoseTarget;
using geometry_msgs::msg::PoseStamped;

template<class CandidateT>
FindReachable<CandidateT>::FindReachable(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params
)
: BT::StatefulActionNode(name, config),
  logger_(rclcpp::get_logger("fer_bt_server")),
  timeout_(params.server_timeout)
{
  auto node = params.nh.lock();
  if (!node) 
  {
    throw BT::RuntimeError("FindReachable: the ROS node went out of scope");
  }

  logger_ = node->get_logger();
  group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node->get_node_base_interface());
  client_ = node->create_client<CheckReachable>(params.default_port_value, rclcpp::ServicesQoS(), group_);

  if (!client_->wait_for_service(params.wait_for_server_timeout)) 
  {
    RCLCPP_ERROR(logger_, "%s: '%s' is not available", name.c_str(),
      params.default_port_value.c_str());
  }
}

template<class CandidateT>
BT::PortsList FindReachable<CandidateT>::base_ports(BT::PortsList addition)
{
  addition.insert(BT::InputPort<std::string>("object_id", "Object the hand may touch"));
  addition.insert(BT::InputPort<double>("speed_scaling",0.3, "Speed Scaling of the planned trajectory (0, 1]"));
  addition.insert(BT::InputPort<std::vector<CandidateT>>("candidates", "Best first"));
  addition.insert(BT::OutputPort<int>("outcome", "fer_interfaces Outcome code"));
  return addition;
}

template<class CandidateT>
BT::NodeStatus FindReachable<CandidateT>::onStart()
{
  const auto candidates = getInput<std::vector<CandidateT>>("candidates");
  const auto object_id = getInput<std::string>("object_id");
  const auto speed_scaling = getInput<double>("speed_scaling").value_or(0.3);
  if (!candidates || !object_id)
  {
    return fail(Outcome::INVALID_GOAL, !candidates ? candidates.error() : object_id.error());
  }
  if (candidates->empty())
  {
    return fail(Outcome::NO_PATH, "no candidates");
  }
  if (!client_->service_is_ready())
  {
    return fail(Outcome::TIMEOUT, "CheckReachable is not available");
  }
  candidates_ = *candidates;
  object_id_ = *object_id;
  speed_scaling_ = speed_scaling;
  index_ = 0;
  send();
  return BT::NodeStatus::RUNNING;
}

template<class CandidateT>
BT::NodeStatus FindReachable<CandidateT>::onRunning()
{
  executor_.spin_some();
  if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    if (std::chrono::steady_clock::now() - sent_ > timeout_)
    {
      client_->remove_pending_request(request_id_);
      pending_ = false;
      return fail(Outcome::TIMEOUT, "CheckReachable did not answer");
    }
    return BT::NodeStatus::RUNNING;
  }
  const auto response = future_.get();
  pending_ = false;
  if (response->outcome.code == Outcome::OK && response->configurations.size() == 3)
  {
    const auto & positions = response->configurations.front().positions;
    write_outputs(candidates_[index_], std::vector<double>(positions.begin(), positions.end()));
    setOutput("outcome", static_cast<int>(Outcome::OK));
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(logger_, "%s: candidate %zu: outcome %d: %s", name().c_str(), index_,
    response->outcome.code, response->outcome.message.c_str());
  if (++index_ < candidates_.size())
  {
    send();
    return BT::NodeStatus::RUNNING;
  }
  return fail(response->outcome.code, "no candidate is reachable");
}

template<class CandidateT>
void FindReachable<CandidateT>::onHalted()
{
  if (pending_)
  {
    client_->remove_pending_request(request_id_);
    pending_ = false;
  }
}

template<class CandidateT>
void FindReachable<CandidateT>::send()
{
  auto request = std::make_shared<CheckReachable::Request>();
  const auto poses = targets(candidates_[index_]);
  for (size_t i = 0; i < poses.size(); ++i)
  {
    PoseTarget target;
    target.pose = poses[i];
    target.path = i == 0 ? PoseTarget::PATH_FREE : PoseTarget::PATH_STRAIGHT;
    if (i > 0)
    {
      target.may_touch.push_back(object_id_);
    }
    request->targets.push_back(target);
  }
  request->speed_scaling = speed_scaling_;
  auto sent = client_->async_send_request(request);
  request_id_ = sent.request_id;
  future_ = sent.future.share();
  pending_ = true;
  sent_ = std::chrono::steady_clock::now();
}

template<class CandidateT>
BT::NodeStatus FindReachable<CandidateT>::fail(uint8_t code, const std::string & message)
{
  setOutput("outcome", static_cast<int>(code));
  RCLCPP_WARN(logger_, "%s: outcome %d: %s", name().c_str(), code, message.c_str());
  return BT::NodeStatus::FAILURE;
}

template class FindReachable<GraspCandidate>;
template class FindReachable<PlaceCandidate>;

BT::PortsList FindReachableGrasp::providedPorts()
{
  return base_ports(
    {
      BT::OutputPort<std::vector<double>>("pregrasp_joints", "Arm configuration at pre-grasp"),
      BT::OutputPort<PoseStamped>("pregrasp_pose", ""),
      BT::OutputPort<PoseStamped>("grasp_pose", ""),
      BT::OutputPort<PoseStamped>("lift_pose", ""),
      BT::OutputPort<double>("width", "Into Grasp.width"),
      BT::OutputPort<double>("force", "Into Grasp.force"),
  });
}

FindReachableGrasp::Targets FindReachableGrasp::targets(const GraspCandidate & candidate) const
{
  return {candidate.pregrasp_pose, candidate.grasp_pose, candidate.lift_pose};
}

void FindReachableGrasp::write_outputs(
  const GraspCandidate & candidate, const std::vector<double> & approach_joints)
{
  setOutput("pregrasp_joints", approach_joints);
  setOutput("pregrasp_pose", candidate.pregrasp_pose);
  setOutput("grasp_pose", candidate.grasp_pose);
  setOutput("lift_pose", candidate.lift_pose);
  setOutput("width", candidate.width);
  setOutput("force", candidate.force);
}

BT::PortsList FindReachablePlace::providedPorts()
{
  return base_ports(
    {
      BT::OutputPort<std::vector<double>>("preplace_joints", "Arm configuration at pre-place"),
      BT::OutputPort<PoseStamped>("place_pose", ""),
      BT::OutputPort<PoseStamped>("retreat_pose", ""),
  });
}

FindReachablePlace::Targets FindReachablePlace::targets(const PlaceCandidate & candidate) const
{
  return {candidate.preplace_pose, candidate.place_pose, candidate.retreat_pose};
}

void FindReachablePlace::write_outputs(
  const PlaceCandidate & candidate, const std::vector<double> & approach_joints)
{
  setOutput("preplace_joints", approach_joints);
  setOutput("place_pose", candidate.place_pose);
  setOutput("retreat_pose", candidate.retreat_pose);
}
}  // namespace fer_behavior_trees
