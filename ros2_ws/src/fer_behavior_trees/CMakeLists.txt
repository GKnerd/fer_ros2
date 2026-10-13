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

install(TARGETS fer_bt_server
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
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format")
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
