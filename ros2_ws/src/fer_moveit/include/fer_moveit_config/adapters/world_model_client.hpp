#ifndef FER_MOVEIT_CONFIG__ADAPTERS__WORLD_MODEL_CLIENT_HPP_
#define FER_MOVEIT_CONFIG__ADAPTERS__WORLD_MODEL_CLIENT_HPP_

#include <chrono>
#include <vector>

#include "fer_interfaces/msg/world_object.hpp"
#include "fer_interfaces/srv/query_objects.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fer_moveit_config
{

/// \brief Reads the objects the collision scene is built from.
class WorldModelClient
{
public:
  WorldModelClient(
    const rclcpp::Node::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group);

  /// \brief FREE and GRASPED objects, fixed ones included.
  /// \throws MotionError TIMEOUT if the world model does not answer.
  std::vector<fer_interfaces::msg::WorldObject> snapshot();

private:
  rclcpp::Client<fer_interfaces::srv::QueryObjects>::SharedPtr client_;
  std::chrono::duration<double> timeout_;
};

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__ADAPTERS__WORLD_MODEL_CLIENT_HPP_
