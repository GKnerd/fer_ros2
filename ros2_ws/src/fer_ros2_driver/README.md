# fer_ros2

ROS 2 **Jazzy** driver stack for the real Franka Emika Robot
("FER" / "Panda"). Firmware version 4.2.x, `libfranka` version 0.9.2. 

## Provenance

This repository started as unmodified import of the packages below from
[`tenfoldpaper/multipanda_ros2`][multipanda], branch `humble`, commit
[`ad741e3`][pinned-commit], with every subsequent change committed on top.
That keeps the diff against upstream inspectable and lets upstream fixes
be cherry-picked deliberately.

Packages carried over from multipanda_ros2:
`franka_bringup`, `franka_example_controllers`, `franka_gripper`,
`franka_hardware`, `franka_msgs`, `franka_multi_mode_controller`,
`franka_robot_state_broadcaster`, `franka_semantic_components`.

multipanda_ros2 is itself built on top of [mcbed's Humble port][mcbed] of
`franka_ros2`, which was originally authored by **Franka Emika GmbH**. See
[`NOTICE`](./NOTICE) for the full attribution chain and license notices.

## Why a Jazzy port

multipanda_ros2 targets ROS 2 Humble and ros2_control 3.x. This port:

- Updates to ROS 2 Jazzy / ros2_control 4.x APIs.
- Strips simulation: MuJoCo, `franka_hardware/src/sim`, sim launch/config,
  and the `mujoco_ros2_control` CMake dependency.
- Drops multi-arm bringup surface (single FER only).
- Re-applies known Jazzy compatibility fixes (realtime publisher
  `trylock()` handling in `franka_robot_state_broadcaster`, `--param-file`
  on controller spawners) that are either absent or solved differently
  upstream.

MoveIt configuration is maintained separately in [`fer_moveit_config`][fer-moveit],
not `franka_moveit_config` from upstream.

## License

Apache License 2.0 — see [`LICENSE`](./LICENSE) and [`NOTICE`](./NOTICE).

[multipanda]: https://github.com/tenfoldpaper/multipanda_ros2/tree/humble
[pinned-commit]: https://github.com/tenfoldpaper/multipanda_ros2/tree/ad741e3ffcbe44cf47f7138ac8659802a3c9e43b
[mcbed]: https://github.com/mcbed/franka_ros2/tree/humble
[fer-moveit]: https://github.com/GKnerd/fer_moveit_config
