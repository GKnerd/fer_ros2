cmake_minimum_required(VERSION 3.8)
project(fer_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspCandidate.msg"
  "msg/JointConfiguration.msg"
  "msg/Outcome.msg"
  "msg/PoseTarget.msg"
  "msg/WorldObject.msg"
  "msg/WorldObjectArray.msg"
  "srv/CheckReachable.srv"
  "srv/GetGraspCandidates.srv"
  "srv/QueryObjects.srv"
  "srv/SetObjectStatus.srv"
  "action/DetectObjects.action"
  "action/Grasp.action"
  "action/MoveGripper.action"
  "action/MoveToJoints.action"
  "action/MoveToPose.action"
  "action/RefineObject.action"
  "action/Release.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs shape_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
