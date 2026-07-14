#include "fer_behavior_trees/bt_server_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <memory>
#include <rclcpp/node_options.hpp>

#include "fer_behavior_trees/go_home_client_node.hpp"
#include "fer_behavior_trees/pick_object_client_node.hpp"
#include "fer_behavior_trees/place_object_client_node.hpp"
#include "fer_behavior_trees/move_to_pose_client_node.hpp"
#include "fer_behavior_trees/control_gripper_client_node.hpp"

BTActionServer::BTActionServer(const rclcpp::NodeOptions& options)
: 
BT::TreeExecutionServer(options)
{
    // Add topics the BT subscribes to, to receive its data from here.
    // ...
}


void BTActionServer::onTreeCreated(BT::Tree& tree)
{
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
}

void BTActionServer::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
{
  BT::RosNodeParams params;
  params.nh = node();
  params.default_port_value = "go_home";
  factory.registerNodeType<GoHomeSkill>("GoHome", params);

  params.default_port_value = "pick_object";
  factory.registerNodeType<PickObjectSkill>("PickObject", params);

  params.default_port_value = "place_object";
  factory.registerNodeType<PlaceObjectSkill>("PlaceObject", params);

  params.default_port_value = "move_to_pose";
  factory.registerNodeType<MoveToPoseSkill>("MoveToPose", params);

  params.default_port_value = "control_gripper";
  factory.registerNodeType<ControlGripperSkill>("ControlGripper", params);

}

std::optional<std::string> BTActionServer::onTreeExecutionCompleted(
    BT::NodeStatus /*status*/, bool /*was_cancelled*/)
{
    logger_cout_.reset();
    return std::nullopt;
}

std::optional<std::string> BTActionServer::onLoopFeedback()
{
    return std::nullopt;   // no per-tick feedback for now
}



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto bt_action_server = std::make_shared<BTActionServer>(options);

  // MultiThreadedExecutor with a timeout — upstream's workaround for a deadlock
  // when pubs/subs are added/removed while spinning.
  rclcpp::executors::MultiThreadedExecutor exec(
      rclcpp::ExecutorOptions(), 
      0, 
      false, 
      std::chrono::milliseconds(250)
    );
  exec.add_node(bt_action_server->node());
  exec.spin();
  exec.remove_node(bt_action_server->node());

  rclcpp::shutdown();
}
