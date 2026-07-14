#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_ros2/bt_action_node.hpp>

#include "fer_skills/action/go_home.hpp"

using GoHome = fer_skills::action::GoHome;




class GoHomeSkill: public BT::RosActionNode<GoHome>
{
    public:

        GoHomeSkill(const std::string& name, 
                    const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params);
        
        // virtual methods that are being overriden
        bool setGoal(Goal& goal) override;
        void onHalt() override;
        BT::NodeStatus onResultReceived(const WrappedResult& result) override;
        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/) override;
        BT::NodeStatus onFailure(
            BT::ActionNodeErrorCode error, 
            const std::optional<WrappedResult>&
        ) override;

    private:
};

