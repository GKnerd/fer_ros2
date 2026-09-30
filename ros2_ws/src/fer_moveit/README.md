# fer_moveit_config

MoveIt 2 configuration and launch files for the **Franka Emika Robot (FER)**,
targeting **ROS 2 Jazzy**.

This package is a downstream rework of the long-running
[`panda_moveit_config`](https://github.com/moveit/panda_moveit_config) package
(originally under the `ros-planning` GitHub organization). It has been adapted
to use the current `franka_description` URDF/SRDF, modern MoveIt 2 planning
pipelines (OMPL and Pilz with the new request/response adapters), and a launch entry
point with explicit arguments for controller selection, fake hardware, and
gripper loading.

It also holds the MoveIt motion server of the FER platform,
`fer_moveit_motion_server`, which serves the motion part of `fer_interfaces` through
`move_group` (see [Motion server](#motion-server)).

## Status

- Tested against **ROS 2 Jazzy** on Ubuntu 24.04.
- Tested with the **FER** (Franka Emika Robot / Panda).
  Other Franka models (e.g. FR3) have not been tested.

## Repository layout

```
fer_moveit_config/
├── config/                  # MoveIt + controller YAMLs (kinematics, OMPL, joint/cartesian limits, …)
│                            # fer_moveit_motion_server.yaml: motion server parameters
├── launch/
│   ├── fer_moveit_launch.py                # move_group + optional RViz, with declared launch args
│   └── fer_moveit_motion_server.launch.py  # motion server
├── include/fer_moveit_config/  # motion server: core/ (no ROS), adapters/, motion_server.hpp
├── src/                        # motion server sources and main.cpp
├── test/                       # gtests; move_group_test.launch.py runs against a real move_group
├── rviz/
│   └── moveit_conf.rviz     # RViz config tailored for this package
├── CMakeLists.txt
├── package.xml
├── LICENSE                  # BSD 3-Clause
├── NOTICE.md                # Upstream attribution + third-party file notice
└── README.md
```

## Dependencies

Declared in `package.xml`:

- `franka_description` (URDF/SRDF for FER)
- `xacro`
- `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners_ompl`,
  `pilz_industrial_motion_planner`, `moveit_simple_controller_manager`
- motion server: `rclcpp`, `rclcpp_action`, `fer_interfaces`, `moveit_msgs`,
  `moveit_core`, `tf2_ros`, `tf2_geometry_msgs`; `franka_msgs` for `hardware:=real`
  only (built without it on a core+sim import)
- `moveit_task_constructor_capabilities` (loads
  `move_group/ExecuteTaskSolutionCapability` for MTC users)

Install everything declared in the manifest with `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

From your colcon workspace root:

```bash
colcon build --symlink-install --packages-up-to fer_moveit_config
source install/setup.bash
```

## Launch

The launch file accepts the following arguments (defaults in parentheses):

| Argument               | Default        | Description |
| ---------------------- | -------------- | ----------- |
| `hardware`             | `mujoco`       | `real` or `mujoco` — selects `config/moveit_controllers_<hardware>.yaml` |
| `use_sim_time`         | `true`         | Use the simulated `/clock` |
| `use_rviz`             | `true`         | Start RViz with `rviz/moveit_conf.rviz` |
| `log_level`            | `warn`         | `debug` / `info` / `warn` / `error` / `fatal` |
| `db`                   | `False`        | Reserved for future warehouse DB use |
| `robot_ip`             | `""`           | Hostname/IP of the real robot |
| `namespace`            | `""`           | ROS namespace |
| `load_gripper`         | `true`         | Load the Franka Hand |
| `ee_id`                | `franka_hand`  | End-effector id: `none`, `franka_hand`, `cobot_pump` |
| `use_fake_hardware`    | `false`        | Use `ros2_control` fake hardware |
| `fake_sensor_commands` | `false`        | Fake sensor commands (only with `use_fake_hardware:=true`) |
| `arm_control_type`     | `effort`       | `effort`, `velocity` (real only) or `position` — selects `<type>_trajectory_controller` |
| `hand_control_type`    | `position`     | `effort` or `position` — selects `gripper_<type>_controller`; ignored for `real` |

`config/moveit_controllers_<hardware>.yaml` lists the controllers `move_group`
may route to on that hardware. The `arm_control_type` / `hand_control_type`
args pick the default ones: the `default` flag is set at launch time, so you do
not need to edit the YAML to switch controllers. On the real robot the gripper
is the `franka_gripper` node (`/fer_gripper/gripper_action`).

The controllers must be active in `ros2_control` before `move_group` can
execute; the bringup starts them inactive.

### Planning pipelines

| Pipeline | Planner | Used for |
|---|---|---|
| `ompl` (default) | OMPL, `RRTConnect` for `fer_arm` | collision-free paths; timed by `AddTimeOptimalParameterization` |
| `pilz_industrial_motion_planner` | Pilz `LIN` (also `PTP`, `CIRC`) | straight TCP lines; Pilz times them itself from `joint_limits.yaml` and `cartesian_limits.yaml` |

Both pipelines use the request adapters `ResolveConstraintFrames`,
`ValidateWorkspaceBounds`, `CheckStartStateBounds` and `CheckStartStateCollision`, and
check the result with `ValidateSolution`. Joint and Cartesian limits are loaded under
`robot_description_planning`. Trajectory execution is time-monitored: a motion may take
its planned duration × 1.1 + 0.5 s.

## Motion server

`fer_moveit_motion_server` serves the motion part of `fer_interfaces`. It is a client
of `move_group`: MoveIt plans and executes, the server adds validation, the world model,
`may_touch`, goal replacement and outcome codes. It never switches controllers.

| Served | Type |
|---|---|
| `/motion/move_to_pose` | action `fer_interfaces/MoveToPose` |
| `/motion/move_to_joints` | action `fer_interfaces/MoveToJoints` |
| `/motion/check_reachable` | service `fer_interfaces/CheckReachable` |

| Used | Purpose |
|---|---|
| `/world_model/query_objects` | FREE and GRASPED objects (fixed ones included) per request |
| `/apply_planning_scene`, `/get_planning_scene` | scene sync; allowed collisions for `may_touch` |
| `/move_action` | plan only |
| `/execute_trajectory` | execution on `move_group`'s default arm controller (`arm_control_type`) |
| `/trajectory_execution_event` | `"stop"` on cancel or replacement |
| `/joint_states`, TF | wait for rest; goal pose → `base` |
| `/franka_robot_state_broadcaster/robot_state` | robot mode (`hardware:=real`) |

From goal to motion:

1. **Validate:** `speed_scaling` in (0, 1], pose frame known to TF (converted to `base`
   once, with the latest transform) → else `INVALID_GOAL`. Real: robot in reflex or user
   stop → `ROBOT_ERROR`.
2. **Scene:** `QueryObjects`, then one `ApplyPlanningScene`: FREE and fixed objects as
   boxes, GRASPED objects attached to `held_by` with the hand links as touch links,
   objects gone from the world model removed. Unknown `may_touch` id → `NOT_FOUND`; no
   answer → `TIMEOUT`.
3. **`may_touch`:** the current allowed-collision matrix plus (object, hand link) pairs,
   sent with the plan request only. A diff replaces the whole matrix, hence the full copy.
4. **Plan:** plan-only `MoveGroup` on `fer_arm` with velocity and acceleration scaling =
   `speed_scaling`: `PATH_FREE` and joint targets with `ompl`, `PATH_STRAIGHT` with Pilz
   `LIN`. A planning failure → `NO_PATH`, the reason in `message`: `move_group`'s plan-only
   path reports every planner error as `FAILURE`, so `UNREACHABLE` does not occur.
5. **Execute:** `ExecuteTrajectory` with no controller named, so `move_group` uses the arm
   controller selected by `arm_control_type`. It checks the start state, sends the
   trajectory to the controller and watches the time.
   Failure → `EXECUTION_FAILED` (`ROBOT_ERROR` if the robot is in reflex or user stop).
6. **Cancel or replacement:** the server publishes `"stop"`, the controller decelerates
   (`decelerate_on_cancel`), and the goal ends `CANCELLED` once every joint is below
   `rest_velocity`. The next goal plans from there. A cancel during planning takes effect
   when `move_group` has finished planning.

`CheckReachable` plans each target from the previous target's end configuration and
never executes.

### Launch

```bash
ros2 launch fer_moveit_config fer_moveit_motion_server.launch.py hardware:=mujoco
```

| Argument | Default | Description |
|---|---|---|
| `hardware` | `mujoco` | `real` or `mujoco`; `use_sim_time` follows it |
| `log_level` | `info` | node log level |
| `params_file` | `''` | `''` selects `config/fer_moveit_motion_server.yaml` |

`move_group` must run (`fer_moveit_launch.py`) and its arm controller must be active.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `planning_time`, `planning_attempts` | 5.0 s, 1 | per plan |
| `goal_position_tolerance`, `goal_orientation_tolerance` | 0.001 m, 0.01 rad | pose targets |
| `rest_velocity`, `rest_timeout` | 0.01 rad/s, 3.0 s | at rest after a stop |
| `tf_timeout`, `world_model_timeout`, `move_group_timeout`, `startup_timeout` | 0.2, 2.0, 7.0, 10.0 s | |

Fixed in the code, since they change only with the robot or the planner choice: group
`fer_arm`, TCP `fer_hand_tcp`, hand links, arm joints, frame `base`, pipelines `ompl` and
Pilz `LIN`, topic `/franka_robot_state_broadcaster/robot_state` (remap it in the launch
file if needed).

### Tests

```bash
colcon test --packages-select fer_moveit_config
colcon test-result --verbose
```

`test_checks` covers the core; `test_contract` runs the server against fake
`move_group`, world model and arm; `move_group_test.launch.py` starts the real
`move_group` with this configuration and checks straight paths, `may_touch`, held objects,
speed scaling, the closed-finger start state and the stop. Every test runs in its own DDS
domain on localhost.

### Known limits

- A cancel during planning takes effect once `move_group` has finished planning.
- `"stop"` stops every execution in `move_group`, not only this server's.
- Objects this server wrote stay in `move_group`'s scene; after a server restart they are
  not removed until an object with the same id is written again.
- Poses are converted with the latest TF (arm at rest).
- Planning failures are always `NO_PATH`, never `UNREACHABLE` (see step 4).
- Straight lines depend on the pose: Pilz checks them against `joint_limits.yaml`, and
  `cartesian_limits.yaml` is sized for poses near `ready`. Close to a singularity even a
  slow straight line can end `NO_PATH`.

Example:

```bash
ros2 launch fer_moveit_config fer_moveit_launch.py \
    hardware:=mujoco \
    use_sim_time:=true \
    arm_control_type:=effort \
    hand_control_type:=position
```

## License

This package is released under the **BSD 3-Clause License** — see
[`LICENSE`](LICENSE). The motion server (`include/`, `src/`, `test/`,
`launch/fer_moveit_motion_server.launch.py`) is released under the **Apache License,
Version 2.0**.

`launch/fer_moveit_launch.py` is adapted from a Franka Robotics example
distributed under the **Apache License, Version 2.0**, and retains its
original header. See [`NOTICE.md`](NOTICE.md) for upstream attribution and
third-party file notices.

## Contributing

Issues and pull requests are welcome at
<https://github.com/GKnerd/fer_moveit_config>.
