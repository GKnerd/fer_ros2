# fer_moveit_config

MoveIt 2 configuration and launch files for the **Franka Emika Robot (FER)**,
targeting **ROS 2 Jazzy**.

This package is a downstream rework of the long-running
[`panda_moveit_config`](https://github.com/moveit/panda_moveit_config) package
(originally under the `ros-planning` GitHub organization). It has been adapted
to use the current `franka_description` URDF/SRDF, modern MoveIt 2 planning
pipelines (OMPL with the new request/response adapters), and a launch entry
point with explicit arguments for controller selection, fake hardware, and
gripper loading.

## Status

- Tested against **ROS 2 Jazzy** on Ubuntu 24.04.
- Tested with the **FER** (Franka Emika Robot / Panda).
  Other Franka models (e.g. FR3) have not been tested.

## Repository layout

```
fer_moveit_config/
├── config/                  # MoveIt + controller YAMLs (kinematics, OMPL, joint/cartesian limits, …)
├── launch/
│   └── fer_moveit_launch.py # move_group + optional RViz, with declared launch args
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
  `moveit_simple_controller_manager`
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
| `arm_control_type`     | `effort`       | `effort` or `position` — selects default arm controller |
| `hand_control_type`    | `position`     | `effort` or `position` — selects default gripper controller |

The `arm_control_type` / `hand_control_type` args are the single source of
truth for which controller `move_group` routes trajectories to: the `default`
flag in `config/moveit_controllers.yaml` is mutated at launch time based on
these args, so you do not need to edit the YAML to switch controllers.

Example:

```bash
ros2 launch fer_moveit_config fer_moveit_launch.py \
    use_sim_time:=true \
    arm_control_type:=effort \
    hand_control_type:=position
```

## License

This package is released under the **BSD 3-Clause License** — see
[`LICENSE`](LICENSE).

`launch/fer_moveit_launch.py` is adapted from a Franka Robotics example
distributed under the **Apache License, Version 2.0**, and retains its
original header. See [`NOTICE.md`](NOTICE.md) for upstream attribution and
third-party file notices.

## Contributing

Issues and pull requests are welcome at
<https://github.com/GKnerd/fer_moveit_config>.
