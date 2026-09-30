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
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(fer_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(fer_skills REQUIRED)

# Old server on fer_skills; removed in 8.7.
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

# New server on fer_interfaces.
set(dependencies
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  fer_interfaces
  geometry_msgs
)

add_library(fer_bt STATIC
  src/core/payload.cpp
  src/nodes/outcome_nodes.cpp
  src/nodes/motion_nodes.cpp
  src/nodes/gripper_nodes.cpp
  src/nodes/world_model_nodes.cpp
  src/nodes/candidate_nodes.cpp
  src/nodes/find_reachable.cpp
  src/server.cpp
)
set_target_properties(fer_bt PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(fer_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(fer_bt ${dependencies})

add_executable(fer_bt_server src/main.cpp)
target_link_libraries(fer_bt_server fer_bt)

install(TARGETS bt_server_node fer_bt_server
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
  # Old server, removed in 8.7.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    include/fer_behavior_trees/bt_server_node.hpp
    include/fer_behavior_trees/control_gripper_client_node.hpp
    include/fer_behavior_trees/go_home_client_node.hpp
    include/fer_behavior_trees/move_to_pose_client_node.hpp
    include/fer_behavior_trees/pick_object_client_node.hpp
    include/fer_behavior_trees/place_object_client_node.hpp
    src/bt_server_node.cpp
    src/control_gripper_client_node.cpp
    src/go_home_client_node.cpp
    src/move_to_pose_client_node.cpp
    src/pick_object_client_node.cpp
    src/place_object_client_node.cpp
    launch/bt_server.launch.py
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_gtest(test_payload test/test_payload.cpp)
  target_link_libraries(test_payload fer_bt)
  ament_add_gtest(test_trees test/test_trees.cpp)
  target_link_libraries(test_trees fer_bt)
  target_compile_definitions(test_trees PRIVATE
    TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees/manipulation")
  ament_add_ros_isolated_gtest(test_nodes test/test_nodes.cpp TIMEOUT 120)
  target_link_libraries(test_nodes fer_bt)
  ament_target_dependencies(test_nodes rclcpp_action)
endif()

ament_package()
