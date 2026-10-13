# Notices

FER ROS 2 platform. Copyright 2026 Georgios Katranis.
Licensed under the Apache License 2.0 (`LICENSE`), except where a package states otherwise.

## Derived work in this repository

- `ros2_ws/src/fer_ros2_driver`: derived from multipanda_ros2 and franka_ros2
  (Franka Emika GmbH), Apache-2.0. See `ros2_ws/src/fer_ros2_driver/NOTICE`.
- `ros2_ws/src/fer_moveit`: configuration derived from panda_moveit_config, BSD-3-Clause;
  one launch file from moveit_resources, Apache-2.0. See `ros2_ws/src/fer_moveit/NOTICE.md`.
- `docker/`: derived from internal tooling of Proximity Robotics & Automation GmbH
  (Copyright 2025), Apache-2.0. Each file carries its modification notice.

## External components

Not part of this repository; `docker/Dockerfile` downloads them when the image is built.

| Component | License | Source |
|---|---|---|
| ROS 2 Jazzy (`ros:jazzy-ros-base`) | Apache-2.0 | https://github.com/ros2 |
| libfranka 0.9.2 | Apache-2.0 | https://github.com/frankarobotics/libfranka |
| franka_description | Apache-2.0 | https://github.com/frankarobotics/franka_description |
| BehaviorTree.ROS2 | Apache-2.0 (package.xml files: MIT) | https://github.com/BehaviorTree/BehaviorTree.ROS2 |
| MuJoCo | Apache-2.0 | https://github.com/google-deepmind/mujoco |
| mujoco_vendor | Apache-2.0 | https://github.com/pal-robotics/mujoco_vendor |
| mujoco_ros2_control | Apache-2.0 | https://github.com/ros-controls/mujoco_ros2_control |
