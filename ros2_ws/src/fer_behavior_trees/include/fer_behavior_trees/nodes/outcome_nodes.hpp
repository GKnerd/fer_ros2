#ifndef FER_BEHAVIOR_TREES__NODES__OUTCOME_NODES_HPP_
#define FER_BEHAVIOR_TREES__NODES__OUTCOME_NODES_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fer_behavior_trees
{
using fer_interfaces::msg::Outcome;

/// \brief Outcome code for an action call that ended without a result.
uint8_t outcome_code(BT::ActionNodeErrorCode error);

/// \brief Outcome code for a service call that ended without a response.
uint8_t outcome_code(BT::ServiceNodeErrorCode error);

/// \brief Value of input \p port of \p node, or nullopt with an error logged.
template<typename T>
std::optional<T> required_input(const BT::TreeNode & node,
  const std::string & port,
  const rclcpp::Logger & logger)
{
  auto value = node.getInput<T>(port);
  if (!value) 
  {
    RCLCPP_ERROR(logger, "%s: %s", node.name().c_str(), value.error().c_str());
    return std::nullopt;
  }
  return value.value();
}

/// \brief Writes \p outcome to the `outcome` port; OK is SUCCESS, anything else FAILURE.
BT::NodeStatus finish(BT::TreeNode & node,
  const Outcome & outcome,
  const rclcpp::Logger & logger
);

/// \brief Client of a fer_interfaces action whose result starts with `outcome`.
template<class ActionT>
class OutcomeActionNode : public BT::RosActionNode<ActionT>
{
public:
  using Base = BT::RosActionNode<ActionT>;
  using typename Base::WrappedResult;
  using Base::onFailure;

  OutcomeActionNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params)
  : Base(name, config, params) {}

  /// \brief Ports of the node: \p addition, `outcome` and the action name.
  static BT::PortsList outcome_ports(BT::PortsList addition)
  {
    addition.insert(BT::OutputPort<int>("outcome", "fer_interfaces Outcome code"));
    return Base::providedBasicPorts(addition);
  }

  /// \brief Cancels the goal once; Tree::haltTree halts every node a second time.
  void halt() override
  {
    if (this->status() == BT::NodeStatus::RUNNING) {
      Base::halt();
      this->resetStatus();
    }
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    return finish(*this, result.result->outcome, this->logger());
  }

  BT::NodeStatus onFailure(
    BT::ActionNodeErrorCode error, const std::optional<WrappedResult> & result) override
  {
    if (result && result->result) {
      return finish(*this, result->result->outcome, this->logger());
    }
    Outcome outcome;
    outcome.code = outcome_code(error);
    outcome.message = BT::toStr(error);
    return finish(*this, outcome, this->logger());
  }
};

/// \brief Client of a fer_interfaces service whose response starts with `outcome`.
template<class ServiceT>
class OutcomeServiceNode : public BT::RosServiceNode<ServiceT>
{
public:
  using Base = BT::RosServiceNode<ServiceT>;
  using Response = typename ServiceT::Response;

  OutcomeServiceNode(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
  : Base(name, config, params) {}

  /// \brief Ports of the node: \p addition, `outcome` and the service name.
  static BT::PortsList outcome_ports(BT::PortsList addition)
  {
    addition.insert(BT::OutputPort<int>("outcome", "fer_interfaces Outcome code"));
    return Base::providedBasicPorts(addition);
  }

  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr & response) override
  {
    if (response->outcome.code == Outcome::OK) {
      write_outputs(*response);
    }
    return finish(*this, response->outcome, this->logger());
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    Outcome outcome;
    outcome.code = outcome_code(error);
    outcome.message = BT::toStr(error);
    return finish(*this, outcome, this->logger());
  }

protected:
  /// \brief Writes the node's outputs from an OK \p response.
  virtual void write_outputs(const Response & /*response*/) {}
};
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__NODES__OUTCOME_NODES_HPP_
