# fer_gripper_server

Gripper server of the FER platform. Serves `MoveGripper`, `Grasp` and `Release` from
`fer_interfaces`, judges grasps from the measured width, and reports GRASPED, FREE and
LOST to the world model.

It is the only component that moves the gripper. It directly updates the world model's grasp state.

## Interfaces

Served:

| Interface | Kind | Name |
|---|---|---|
| `MoveGripper` | action | `/gripper/move` |
| `Grasp` | action | `/gripper/grasp` |
| `Release` | action | `/gripper/release` |

Used:

- `/world_model/query_objects`, `/world_model/set_object_status`
- TF `base ↔ fer_hand_tcp`
- the gripper hardware (see [Hardware](#hardware))

## Behaviour

- **Grasp:** the object must be FREE. Success when the measured width is within
  `tolerance` of `width` and above `min_hold_width` (real robot: and libfranka's
  `is_grasped`). Then GRASPED, `held_by: fer_hand_tcp`, pose relative to the hand.
  On failure the gripper reopens to its previous width → `GRASP_FAILED`, world model
  unchanged.
- **Release:** the object must be GRASPED. Opens, confirms the width, then FREE at the
  hand pose combined with the stored offset (`source: release_estimate`).
- **Watch:** while an object is held, it is set LOST as soon as the grip is gone —
  real: `is_grasped` false; MuJoCo: width more than `tolerance` away from the grasp
  width. Covers slipping, removal, and a `MoveGripper` opening the hand. Paused during
  `Grasp` and `Release`.
- **One goal at a time:** a new goal of any of the three actions ends the running one
  with `CANCELLED`; the old hardware command is stopped before the new one starts.
- **Outcomes:** invalid width or force → `INVALID_GOAL`; hardware error →
  `GRIPPER_FAILED` (`RELEASE_FAILED` in `Release`); world model not answering →
  `TIMEOUT`.

## Hardware

**Real** (`hardware:=real`) — the `franka_gripper` node `/fer_gripper` from `fer_ros2`:
actions `move`, `grasp` (epsilon = `tolerance`), topic `gripper_state` (needs
`fer_ros2` with `franka_msgs/GripperState`, 2026-09-28 or later). `franka_msgs` is a
runtime dependency of this mode only; a core+sim import builds without it.

**MuJoCo** (`hardware:=mujoco`) — `gripper_effort_controller` via
`control_msgs/GripperCommand`. The controller must be active before the server starts;
the server does not switch controllers:

```bash
ros2 launch fer_ros2_bringup fer_mujoco_ros2_control.launch.py hand_control_type:=effort
ros2 control switch_controllers --activate gripper_effort_controller
```

A grasp closes fully with `max_effort` = force and stalls on the object
(`allow_stalling: true` in `fer_controllers_gripper.yaml`). The width is twice
`fer_finger_joint1` from `/joint_states`.

## Launch

```bash
ros2 launch fer_gripper_server gripper_server.launch.py hardware:=mujoco
```

| Argument | Default | Meaning |
|---|---|---|
| `hardware` | `mujoco` | `real` or `mujoco`; sets `use_sim_time` for `mujoco` |
| `params_file` | `''` | `''` selects `config/gripper_<hardware>.yaml` |
| `log_level` | `info` | |

## Parameters

Both hardware options (`config/gripper_real.yaml`, `config/gripper_mujoco.yaml`):

| Parameter | Real | MuJoCo | Meaning |
|---|---|---|---|
| `hand_frame` | `fer_hand_tcp` | `fer_hand_tcp` | `held_by` link |
| `base_frame` | `base` | `base` | |
| `max_width` | 0.08 m | 0.08 m | |
| `max_force` | 70 N | 20 N | real: continuous hand force; sim: finger motor limit |
| `min_hold_width` | 0.002 m | 0.002 m | below this nothing is held |
| `move_tolerance` | 0.003 m | 0.003 m | `MoveGripper`, `Release` |
| `watch_rate` | 10 Hz | 10 Hz | |
| `tf_timeout` | 0.2 s | 0.2 s | |
| `startup_timeout` | 10 s | 10 s | wait for the hardware at startup |
| `world_model_timeout` | 2 s | 2 s | |
| `action_timeout` | 10 s | 10 s | per hardware command |
| `state_timeout` | 1 s | 1 s | wait for a fresh width |

Real only: `gripper_namespace` (`/fer_gripper`), `speed` (0.1 m/s).
MuJoCo only: `controller` (`gripper_effort_controller`), `joint` (`fer_finger_joint1`),
`joint_states_topic` (`/joint_states`), `move_force` (20 N).

## Layout

- `core/grasp.py` — no ROS: grasp checks, errors, the `Gripper` interface.
- `adapters/` — `franka_gripper.py` (real), `gripper_command.py` (MuJoCo),
  `world_model_client.py`, `actions.py` (waiting on futures and action goals).
- `gripper_server.py` — the node.

## Tests

```bash
colcon test --packages-select fer_gripper_server
colcon test-result --verbose --test-result-base build/fer_gripper_server
```

Core tests, a contract test (the server with a simulated hand, a fake world model and
TF in one process), franka adapter tests (run only with `franka_msgs/GripperState`),
flake8 and pep257.

## Known limits

- The held object is remembered in memory only; after a restart it is not watched
  until the next grasp.
- Poses use the latest TF: `Grasp` and `Release` assume the arm is at rest.
- If the world model does not accept GRASPED after a successful grasp, or the goal is
  cancelled between closing and reporting, the hand holds an object the world model
  still lists as FREE.

## License

Apache-2.0, see `LICENSE`.
