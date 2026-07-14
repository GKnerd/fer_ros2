cmake_minimum_required(VERSION 3.10)
project(fer_behavior_trees)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(fer_skills REQUIRED)

set(BT_SERVER_DEPS
  rclcpp
  rclcpp_action
  behaviortree_ros2
  geometry_msgs
  control_msgs
  fer_skills
)

add_executable(bt_server_node
  src/bt_server_node.cpp
  src/go_home_client_node.cpp
  src/pick_object_client_node.cpp
  src/place_object_client_node.cpp
  src/move_to_pose_client_node.cpp
  src/control_gripper_client_node.cpp
)


target_include_directories(bt_server_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_server_node ${BT_SERVER_DEPS})

install(TARGETS bt_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
launch 
config
rviz
trees
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
