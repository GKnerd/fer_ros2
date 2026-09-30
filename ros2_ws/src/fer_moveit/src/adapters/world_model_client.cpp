#include "fer_moveit_config/adapters/world_model_client.hpp"

#include <memory>
#include <string>
#include <vector>

#include "fer_interfaces/msg/outcome.hpp"
#include "fer_moveit_config/core/checks.hpp"

namespace fer_moveit_config
{

using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::WorldObject;
using fer_interfaces::srv::QueryObjects;

WorldModelClient::WorldModelClient(
  const rclcpp::Node::SharedPtr & node, const rclcpp::CallbackGroup::SharedPtr & group)
: client_(node->create_client<QueryObjects>(
      "/world_model/query_objects", rclcpp::ServicesQoS(), group)),
  timeout_(node->declare_parameter("world_model_timeout", 2.0))
{
}

std::vector<WorldObject> WorldModelClient::snapshot()
{
  if (!client_->service_is_ready()) {
    throw MotionError(
      Outcome::TIMEOUT, "world model service '" + std::string(client_->get_service_name()) +
      "' not available");
  }
  auto request = std::make_shared<QueryObjects::Request>();
  request->statuses = {WorldObject::FREE, WorldObject::GRASPED};
  request->include_fixed = true;
  auto pending = client_->async_send_request(request);
  auto future = pending.future.share();
  if (!wait_for(future, timeout_)) {
    client_->remove_pending_request(pending.request_id);
    throw MotionError(Outcome::TIMEOUT, "no answer from the world model");
  }
  return future.get()->objects;
}

}  // namespace fer_moveit_config
