# fer_ros2

ROS 2 Jazzy platform for the Franka Emika Robot (FER / Panda, libfranka 0.9.2): control,
planning, behavior trees, perception. Real robot on the Jetson Orin, MuJoCo simulation on
x86_64.

## Start of every session

- Read `docs/STATUS.md`: current step, open items, recent decisions.
- The plan is `docs/Platform.md`. Copies outside `docs/` are stale.

## Layout

- One repository. Every own ROS package is a folder in `ros2_ws/src`; the driver packages
  are in `ros2_ws/src/fer_ros2_driver`. Image in `docker/`, DDS config in `env/`, docs in
  `docs/`.
- Externals are not in the repository. `docker/Dockerfile` clones them at pinned commits
  (`ARG` lines): libfranka into `~/deps`; `franka_description`, `BehaviorTree.ROS2` and,
  with `WITH_MUJOCO=1`, `mujoco_vendor` and `mujoco_ros2_control` into the underlay
  `~/deps_ws`. A pin changes only there.
- One `.clang-format` in `ros2_ws/src`; the package CMake files point at it by path.
- Branches: `main` holds releases tagged `jazzy-<version>` and is moved only by the user.
  `jazzy_devel` takes all work through pull requests; the CI check `description-invariant`
  must pass.

## Build and test

- Image: `./docker/build_image.sh`; container: `./docker/run_container.sh`. Image
  `fer_ros2/ros:jazzy_<arch>`. MuJoCo is built on x86_64 and skipped on aarch64;
  `WITH_MUJOCO=0|1 ./docker/build_image.sh` overrides.
- The image build copies `ros2_ws/build`, `install` and `log` back to the host, and with
  them the `src` snapshot from the start of the build: files changed during a build are
  overwritten.
- The user runs `colcon build` / `colcon test` in the dev container. Do not run them unasked.
- When asked to test: use a throwaway container (`--network none`, Fast DDS, own
  `ROS_DOMAIN_ID`, `--merge-install` overlay, `local_setup.bash`). Never touch the host
  `ros2_ws/build`, `install` or `log`.
- A sub-phase is done when `colcon test` passes for its packages.

## Fixed decisions — do not reopen

- One repository `fer_ros2` with all own packages, driver included; externals pinned in the
  Dockerfile, no vcs, no submodules. One Dockerfile for both architectures; one bringup
  (`fer_ros2_bringup`, `hardware:=real|mujoco|mock`).
- Robot description is upstream `franka_description`, never an own package.
- Servers never switch controllers. The user activates them (launch or
  `ros2 control switch_controllers`); a server waits for its controller and fails with a hint.
- Motion backend is a client of `move_group` (`fer_moveit_motion_server` in
  `fer_moveit`), not MoveItCpp; straight paths through Pilz LIN.
- Servers depend only on `fer_interfaces`. World-model clients are written per package.
- Objects are boxes only.
- Controller names: `<interface>_<type>_controller`.
- C++ is formatted with clang-format 18 using `ros2_ws/src/.clang-format`, with { on its own
  line everywhere.
- A top-level launch file declares and forwards every argument of what it includes.

## Gotchas

- MuJoCo: freejoints must be named (`MujocoSystemInterface` refuses unnamed joints).
- `rosdep` in the image skips the keys of the externals; a new external needs its key added
  to `--skip-keys`, or rosdep installs its apt release next to the pinned one.
- CI builds only `franka_description` and `fer_ros2_bringup`, limited with `--base-paths`;
  `--packages-select` alone fails because the other packages are in the workspace.

## End of a session

When asked, update `docs/STATUS.md`: current step, what changed, decisions with their reason,
next step.
