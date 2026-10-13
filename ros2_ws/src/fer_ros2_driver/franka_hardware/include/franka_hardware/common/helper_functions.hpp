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

#include <algorithm>
#include <iterator>
#include <ostream>
#include <string>
#include <type_traits>
#include <vector>
// #include <mujoco/mujoco.h>

// A dump of functions that are used by both hardware and multi_hardware interfaces.
namespace franka_hardware
{

bool startsWith(const std::string & mainStr, const std::string & toMatch);

bool all_of_element_has_string(std::vector<std::string> vec, std::string content);

/**
   * Checks the command mode type of the provided interface.
   * Returns 0 if size is 0,
   * 1 if it is joint,
   * 2 if it is cartesian,
   * -1 otherwise.
  */
int check_command_mode_type(std::vector<std::string> interfaces);
// Function for extracting ns in joint names
std::string get_ns(std::string const & s);

// Function for extracting joint number
int get_joint_no(std::string const & s);

}  // namespace franka_hardware
