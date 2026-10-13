# fer_ros2_bringup

Bringup for the Franka Emika Robot (FER / Panda) on ROS 2 Jazzy, for the real
robot and for MuJoCo simulation. The hardware is selected with one argument,
`hardware:=real|mujoco`.

## Launch files

Each level includes the one below it and adds one part of the stack.

| Launch file | Starts |
|---|---|
| `fer_real_ros2_control.launch.py` / `fer_mujoco_ros2_control.launch.py` | description, `robot_state_publisher`, ros2_control, controllers, gripper |
| `fer_moveit_and_control.launch.py` | + `move_group` and RViz (`fer_moveit`); no controller activated |
| `fer_manipulation.launch.py` | + world model (`fer_world_model`, mock perception with `perception:=mock`), grasp planner (`fer_grasp_planner`), motion server (`fer_moveit`), gripper server (`fer_gripper_server`); activates the controllers |
| `fer_manipulation_bt.launch.py` | + behavior tree server (`fer_bt_server` in `fer_behavior_trees`, `/execute_tree`) |

```bash
# simulation
ros2 launch fer_ros2_bringup fer_manipulation_bt.launch.py hardware:=mujoco

# real robot
ros2 launch fer_ros2_bringup fer_manipulation_bt.launch.py hardware:=real robot_ip:=<FCI address>
```

To change a tree without restarting the simulation, run `fer_manipulation.launch.py`
and start the BT server on its own, restarting only that:

```bash
ros2 launch fer_behavior_trees fer_bt_server.launch.py use_sim_time:=true
```

- `hardware` defaults to `mujoco`. `robot_ip` is required for `real`.
- `use_sim_time` follows the hardware: `true` for `mujoco`, `false` for `real`.
- `arm_control_type` (`effort`, `position`; `velocity` on `real`) and
  `hand_control_type` (`effort`, `position`; `mujoco` only) select the
  controllers MoveIt routes to. In simulation they also select the MuJoCo
  actuators.
- On the real robot the whole bringup shuts down if the gripper node exits.

## Running a tree

Send the goal once the gripper server has logged that it is ready; it starts after
the controller switch.

```bash
ros2 action send_goal /execute_tree btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: PickPlace, payload: '{\"target_class\": \"box\", \"pick_attempts\": 2, \"speed_scaling\": 0.1}'}"
```

`speed_scaling` is optional; without it the trees move at the BT server's
`speed_scaling` parameter (0.3).

## Controllers

Only the broadcasters start active. Motion and gripper controllers are loaded
inactive. `fer_manipulation.launch.py` activates
`<arm_control_type>_trajectory_controller` and, on `mujoco`,
`gripper_<hand_control_type>_controller` (default `effort`, which the gripper
server needs). Below that level they are switched on by hand:

```bash
ros2 control switch_controllers --activate effort_trajectory_controller gripper_position_controller
```

| Controller | real | mujoco |
|---|---|---|
| `joint_state_broadcaster` | active | active |
| `franka_robot_state_broadcaster` | active | — |
| `effort_trajectory_controller` | inactive | inactive |
| `velocity_trajectory_controller` | inactive | — |
| `position_trajectory_controller` | inactive | inactive |
| `gripper_position_controller` | — | inactive |
| `gripper_effort_controller` | — | inactive |

On the real robot the gripper is the `franka_gripper` node
(`/fer_gripper/gripper_action`), not a ros2_control controller.

Configuration:

- `config/fer_controllers.yaml` — shared by all backends
- `config/fer_controllers_real.yaml` — `hardware:=real`
- `config/fer_controllers_gripper.yaml` — `hardware:=mujoco`

## Description

`urdf/fer.urdf.xacro` includes the upstream
`franka_description/robots/fer/fer.urdf.xacro` unmodified and adds one
ros2_control overlay (component name `fer_hardware`):

- `urdf/control/fer_real.ros2_control.xacro` — `franka_hardware/FrankaMultiHardwareInterface`
- `urdf/control/fer_mujoco.ros2_control.xacro` — `mujoco_ros2_control/MujocoSystemInterface`,
  plus `urdf/mujoco/fer_mujoco_inputs.xacro` (actuators, joint dynamics, gravity compensation)

`hardware:=none` produces the plain upstream model. Upstream's own
`ros2_control` argument must stay `false`.

MuJoCo scenes live in `scenes/`; select one with `scene:=<path>` on
`fer_mujoco_ros2_control.launch.py` or `fer_moveit_and_control.launch.py`. `base_world.xml`
(default) is the floor only. `pick_place_world.xml` puts the robot on a table
(top at `base` z = 0, as `fer_world_model/config/fixtures.yaml`) with free
objects at the poses of `fer_world_model/config/mock_objects.yaml`.

## Tests

`test/test_description.py` checks that the model is identical to upstream for
every backend, that exactly one overlay is present and consistent, and that the
controller configuration only names existing joints. It runs in CI on every
push:

```bash
colcon test --packages-select fer_ros2_bringup --ctest-args -R test_description
colcon test-result --verbose
```

CI (`.github/workflows/fer_description.yml` at the repository root) clones
`franka_description` at the pin in `docker/Dockerfile`.

## License

Apache-2.0, see `LICENSE` at the repository root.
