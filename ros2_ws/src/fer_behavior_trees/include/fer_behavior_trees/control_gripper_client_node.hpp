#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_ros2/bt_action_node.hpp>

#include "fer_skills/action/control_gripper.hpp"

using ControlGripper = fer_skills::action::ControlGripper;


// ControlGripper's goal is a control_msgs/GripperCommand, exposed as position +
// max_effort input ports. See providedPorts() in the .cpp.
class ControlGripperSkill: public BT::RosActionNode<ControlGripper>
{
    public:

        ControlGripperSkill(const std::string& name,
                            const BT::NodeConfig& conf,
                            const BT::RosNodeParams& params);

        // Declares the extra ports (on top of the base "action_name" port).
        static BT::PortsList providedPorts();

        // virtual methods that are being overriden
        bool setGoal(Goal& goal) override;
        void onHalt() override;
        BT::NodeStatus onResultReceived(const WrappedResult& result) override;
        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
        BT::NodeStatus onFailure(
            BT::ActionNodeErrorCode error,
            const std::optional<WrappedResult>&
        ) override;

    private:
};
