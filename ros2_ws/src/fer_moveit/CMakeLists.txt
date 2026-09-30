cmake_minimum_required(VERSION 3.10)
project(fer_moveit_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(franka_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fer_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# Real profile only; without it the server runs with hardware:=mujoco only.
find_package(franka_msgs QUIET)

set(dependencies
  rclcpp
  rclcpp_action
  fer_interfaces
  geometry_msgs
  moveit_core
  moveit_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(motion_server STATIC
  src/core/checks.cpp
  src/adapters/world_model_client.cpp
  src/adapters/move_group_client.cpp
  src/motion_server.cpp
)
set_target_properties(motion_server PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(motion_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(motion_server ${dependencies})
if(franka_msgs_FOUND)
  target_sources(motion_server PRIVATE src/adapters/robot_mode_monitor.cpp)
  ament_target_dependencies(motion_server franka_msgs)
else()
  target_sources(motion_server PRIVATE src/adapters/robot_mode_monitor_unavailable.cpp)
endif()

add_executable(fer_moveit_motion_server src/main.cpp)
target_link_libraries(fer_moveit_motion_server motion_server)

install(TARGETS fer_moveit_motion_server DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Adapted upstream launch file, kept in its original style.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE launch/fer_moveit_launch.py)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(control_msgs REQUIRED)

  ament_add_ros_isolated_gtest(test_checks test/test_checks.cpp)
  target_link_libraries(test_checks motion_server)

  ament_add_ros_isolated_gtest(test_contract test/test_contract.cpp TIMEOUT 300)
  target_link_libraries(test_contract motion_server)

  if(franka_msgs_FOUND)
    ament_add_ros_isolated_gtest(test_robot_mode test/test_robot_mode.cpp TIMEOUT 120)
    target_link_libraries(test_robot_mode motion_server)
  endif()

  ament_add_gtest_executable(test_move_group test/test_move_group.cpp)
  target_link_libraries(test_move_group motion_server)
  ament_target_dependencies(test_move_group control_msgs)
  add_launch_test(test/move_group_test.launch.py
    TIMEOUT 300
    ARGS "test_binary:=$<TARGET_FILE:test_move_group>")
endif()

ament_package()
