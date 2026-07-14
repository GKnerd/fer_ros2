#pragma once

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/bt_factory.h>
#include <optional>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <behaviortree_ros2/tree_execution_server.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>


class BTActionServer : public BT::TreeExecutionServer
{
    public:
        BTActionServer(const rclcpp::NodeOptions& options);

        void onTreeCreated(BT::Tree& tree) override;
        void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override;
        std::optional<std::string> onTreeExecutionCompleted(
            BT::NodeStatus status,
            bool was_cancelled
        ) override;
        std::optional<std::string> onLoopFeedback() override;
    
    private:

        std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};
