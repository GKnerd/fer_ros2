# Franka Emika Robot (FER / Panda) — ROS 2 Jazzy platform

ROS 2 Jazzy stack for the FER (libfranka 0.9.2): control, planning, behavior trees and
perception. The same packages run on the real robot and in MuJoCo; the hardware is a launch
argument. A description of the platform, what is tested and what is open is in
`docs/Platform.md`.

Supported hosts: x86_64 workstations and the Jetson Orin AGX (aarch64, real robot only).

## Packages

| Folder in `ros2_ws/src` | Role |
|---|---|
| `fer_interfaces` | actions, services and messages between the components |
| `fer_world_model` | world model server, mock perception |
| `fer_gripper_server` | gripper server, real and MuJoCo adapters |
| `fer_grasp_planner` | grasp and place candidates |
| `fer_moveit` | MoveIt configuration and the motion server |
| `fer_behavior_trees` | BT server and trees |
| `fer_ros2_bringup` | description overlays, controllers, launch files, MuJoCo scenes |
| `fer_ros2_driver` | real-robot driver: `franka_hardware`, `franka_gripper`, `franka_msgs`, `franka_semantic_components`, `franka_robot_state_broadcaster` |

## Build and run

```bash
git clone git@github.com:GKnerd/fer_ros2.git && cd fer_ros2
./docker/build_image.sh
./docker/run_container.sh
```

One Dockerfile (`docker/Dockerfile`) serves both architectures. The image tag is
`fer_ros2/ros:jazzy_<arch>` (`x86_64` or `aarch64`).

- libfranka 0.9.2 and the external ROS packages (`franka_description`, `BehaviorTree.ROS2`,
  and for MuJoCo `mujoco_vendor`, `mujoco_ros2_control`) are cloned at pinned versions into
  the image (`~/deps`, `~/deps_ws`). The pins are the `ARG` lines in `docker/Dockerfile`.
- MuJoCo is built on x86_64 and skipped on aarch64. Override with
  `WITH_MUJOCO=1 ./docker/build_image.sh` or `WITH_MUJOCO=0 ./docker/build_image.sh`.
- The image builds `ros2_ws`; `build_image.sh` copies `build/`, `install/` and `log/` back to
  the host `ros2_ws/`.
- `run_container.sh` mounts `ros2_ws`, `env` and `data` and sets real-time limits (`rtprio`,
  `memlock`) on every host; they only take effect on the real robot. `CYCLONEDDS_URI` points
  at `env/cyclone_dds.xml`.

Rebuild in the container:

```bash
colcon build --symlink-install --merge-install --cmake-args -DFranka_DIR:PATH=/home/fer_ros2/deps/libfranka/build
```

## Run

```bash
ros2 launch fer_ros2_bringup fer_manipulation_bt.launch.py hardware:=mujoco
ros2 launch fer_ros2_bringup fer_manipulation_bt.launch.py hardware:=real robot_ip:=<FCI address>
```

## Jetson Orin AGX

Tune the interrupt coalescing of the NIC that carries FCI traffic before a control session:

```bash
sudo ./config/nic_orin_config.sh -i <iface>
```

The settings are volatile and are lost on reboot or driver reload.

## Branches and releases

- `main`: releases only, each tagged `jazzy-<version>`. A plain clone gets the latest release.
- `jazzy_devel`: development. Changes come in through pull requests, see `CONTRIBUTING.md`.
- `jazzy-0.1.0` is the last state with separate repositories per package (`.repos` profiles).

## Layout

```
docker/     Dockerfile, build_image.sh, run_container.sh
ros2_ws/    src/ tracked; build/, install/, log/ ignored
env/        cyclone_dds.xml
config/     nic_orin_config.sh
docs/       platform description, status, DDS profiles, legacy documents
.github/    CI workflow
data/       ignored; local data
```

## License

Apache-2.0, see `LICENSE`. Third-party notices: `NOTICE.md`.
