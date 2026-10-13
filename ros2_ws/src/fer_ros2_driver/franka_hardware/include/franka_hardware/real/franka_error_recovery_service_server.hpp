// Copyright (c) 2023 Seongjin Bien
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "franka/exception.h"
#include "franka_hardware/real/robot.hpp"
#include "franka_msgs/srv/error_recovery.hpp"

namespace franka_hardware
{
class FrankaErrorRecoveryServiceServer : public rclcpp::Node
{
public:
  FrankaErrorRecoveryServiceServer(
    const rclcpp::NodeOptions & options, std::shared_ptr<Robot> robot, std::string prefix = "");

private:
  void triggerAutomaticRecovery(
    const franka_msgs::srv::ErrorRecovery::Request::SharedPtr & request,
    const franka_msgs::srv::ErrorRecovery::Response::SharedPtr & response);

  std::shared_ptr<Robot> robot_;
  rclcpp::Service<franka_msgs::srv::ErrorRecovery>::SharedPtr error_recovery_service_;
};

}  // namespace franka_hardware
