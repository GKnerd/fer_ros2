# FER ROS 2 Platform

State as of 2026-10-13. Describes what exists, what is tested, the repository layout, known
limitations and what is open. The migration that produced this state is recorded in
[`legacy/FER_ROS2_Consolidation_Plan.md`](legacy/FER_ROS2_Consolidation_Plan.md); current
work is tracked in [`STATUS.md`](STATUS.md).

## 1. What the platform is

A ROS 2 Jazzy stack for the Franka Emika Robot (FER / Panda, libfranka 0.9.2) that picks and
places objects with behavior trees. The same packages run on the real robot and in MuJoCo;
the hardware is a launch argument, `hardware:=real|mujoco`. The real robot is driven from a
Jetson Orin AGX (aarch64); the simulation runs on x86_64.

```
fer_behavior_trees        order, choices, retries (PickPlace, Pick, Place)
  │  fer_interfaces (actions, services, messages)
  ├─► motion backend      /motion/*       fer_moveit_motion_server, a client of move_group
  ├─► fer_gripper_server  /gripper/*      gripper commands, grasp evidence → object status
  ├─► fer_world_model     /world_model/*  objects: id, pose, box size, status
  └─► fer_grasp_planner   /grasp/*, /place/*   grasp and place poses for an object
ros2_control              controllers loaded inactive; activated at launch or by the operator
```

The trees never see a trajectory, a planner or a controller name. Servers never switch
controllers. The robot description is upstream `franka_description`, used unmodified plus
one hardware overlay.

## 2. What runs and is tested

| Check | MuJoCo (x86_64) | Real FER (Orin) |
|---|---|---|
| Fresh clone, `vcs import`, build from scratch | core + real + sim, 2026-10-13 | core + real, 2026-10-13 |
| Full stack starts (`fer_manipulation_bt.launch.py`) | yes | yes |
| `PickPlace` with mock perception | SUCCESS (2026-10-12, 2026-10-13) | SUCCESS (2026-10-12; stack run 2026-10-13) |
| `speed_scaling` from the goal payload | tested | default speed (0.3) tested |

`colcon test` passes for `fer_interfaces`, `fer_world_model`, `fer_gripper_server`,
`fer_grasp_planner`, `fer_moveit`, `fer_behavior_trees` and `fer_ros2_bringup` (unit tests,
contract tests against fake neighbours, `move_group` launch test, description invariants,
`ament_flake8` / `ament_pep257` / clang-format). The driver packages build and run but fail
`colcon test` on `copyright` and `lint_cmake`. `speed_and_separation_monitoring` is not
part of the tested stack.

Not yet verified: see §8.1.

## 3. Repository layout

`fer_ros2_docker` is the meta-repository: Docker image, DDS configuration, profile manifests
and documentation, no ROS packages. Component repositories are imported into `ros2_ws/src`
with `vcs import . < <profile>`.

| Profile | Contents | Imported on |
|---|---|---|
| `fer_core.repos` | own packages below, `franka_description` @ `72baf5b`, `BehaviorTree.ROS2` @ `6c6aa07` | every host |
| `fer_real.repos` | `fer_ros2_driver`, libfranka 0.9.2 into `deps/` | x86_64, Orin |
| `fer_sim.repos` | `mujoco_ros2_control` @ `375a5db`, `mujoco_vendor` @ `26187d6` | x86_64 only |

| Repository | Branch | Packages | Role |
|---|---|---|---|
| `fer_interfaces` | `jazzy_devel` | `fer_interfaces` | contract: actions, services, messages |
| `fer_world_model` | `main` | `fer_world_model` | world model server, mock perception |
| `fer_gripper_server` | `jazzy_devel` | `fer_gripper_server` | gripper server, real and MuJoCo adapters |
| `fer_grasp_planner` | `jazzy_devel` | `fer_grasp_planner` | grasp and place candidates |
| `fer_moveit` | `jazzy-devel` | `fer_moveit` | MoveIt configuration and the MoveIt motion backend |
| `fer_behavior_trees` | `main` | `fer_behavior_trees` | BT server `fer_bt_server`, trees |
| `fer_ros2_bringup` | `main` | `fer_ros2_bringup` | description overlays, controllers, launch levels, scenes |
| `fer_ros2_driver` | `jazzy` | `franka_hardware`, `franka_gripper`, `franka_msgs`, `franka_semantic_components`, `franka_robot_state_broadcaster` | real-robot driver, Jazzy port of multipanda_ros2 |
| `speed_and_separation_monitoring` | `main` | `speed_and_separation_monitoring` | safety prototype, not integrated |

Externals carry pins only in the profiles; own repositories track their branch. Dependencies
point one way: every server depends on `fer_interfaces` and standard messages only;
`fer_moveit` additionally on MoveIt and `franka_description`; `fer_ros2_bringup` sits on top
and nothing depends on it. `franka_msgs` is used only for `hardware:=real` and is not needed
to build a core + sim import.

**Image.** One `docker/Dockerfile` for x86_64 and aarch64, tag
`fer_ros2_docker/ros:jazzy_<arch>`. It builds libfranka from `deps/libfranka` when present,
runs `rosdep install` and `colcon build` over `ros2_ws/src`. `docker/run_container.sh` sets
real-time limits on every host and mounts `ros2_ws`, `env` and `data`. In the container:

```bash
colcon build --symlink-install --merge-install --cmake-args -DFranka_DIR:PATH=/home/fer_ros2/deps/libfranka/build
```

## 4. Running the stack

| Launch file (`fer_ros2_bringup`) | Adds |
|---|---|
| `fer_real_ros2_control.launch.py` / `fer_mujoco_ros2_control.launch.py` | description, `robot_state_publisher`, ros2_control, controllers (motion and gripper controllers inactive), real: `franka_gripper` node |
| `fer_moveit_and_control.launch.py` | `move_group` and RViz; no controller activated |
| `fer_manipulation.launch.py` | world model (mock perception), grasp planner, motion server, gripper server; activates `<arm_control_type>_trajectory_controller` and, in MuJoCo, `gripper_<hand_control_type>_controller` |
| `fer_manipulation_bt.launch.py` | BT server (`/execute_tree`) |

Each level declares every argument of the files it includes and forwards it, so the top level
decides. `hardware` defaults to `mujoco`; `use_sim_time` follows it. `robot_ip` is required
for `real`. The manipulation levels use `scenes/pick_place_world.xml` (robot on a table, the
mock objects in place).

```bash
ros2 launch fer_ros2_bringup fer_manipulation_bt.launch.py hardware:=mujoco
ros2 launch fer_ros2_bringup fer_manipulation_bt.launch.py hardware:=real robot_ip:=<FCI address>
ros2 action send_goal /execute_tree btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: PickPlace, payload: '{\"target_class\": \"box\", \"pick_attempts\": 2, \"speed_scaling\": 0.1}'}" --feedback
```

Controllers follow `<interface>_<type>_controller`:

| Controller | real | mujoco |
|---|---|---|
| `joint_state_broadcaster` | active | active |
| `franka_robot_state_broadcaster` | active | — |
| `effort_trajectory_controller` | inactive | inactive |
| `velocity_trajectory_controller` | inactive | — |
| `position_trajectory_controller` | inactive | inactive |
| `gripper_position_controller`, `gripper_effort_controller` | — (`franka_gripper` node) | inactive |

Diagnostics for the real robot: `fer_ros2_bringup/scripts/fer_sanity_check.py` (rates, TF,
control loop alive) and `fer_mode_switch_check.py` (effort ↔ velocity switch).

## 5. Components

**Contract (`fer_interfaces`).** SI units; gripper width is the full opening. Poses are
`PoseStamped` in any TF frame and are converted to `base` once, when a goal is accepted.
Every result starts with an `Outcome`; trees branch on `outcome.code` only. A new goal on a
server replaces the running one, which ends `CANCELLED`. Objects are boxes: an object pose is
the centre of its bounding box. Only perception and the gripper server write to the world
model; the blackboard holds object ids only.

| Interface | Kind | Name | Served by |
|---|---|---|---|
| `MoveToPose`, `MoveToJoints` | action | `/motion/move_to_pose`, `/motion/move_to_joints` | motion backend |
| `CheckReachable` | service | `/motion/check_reachable` | motion backend |
| `MoveGripper`, `Grasp`, `Release` | action | `/gripper/move`, `/gripper/grasp`, `/gripper/release` | `fer_gripper_server` |
| `DetectObjects`, `RefineObject` | action | `/world_model/detect_objects`, `/world_model/refine_object` | `fer_world_model` |
| `QueryObjects`, `SetObjectStatus` | service | `/world_model/query_objects`, `/world_model/set_object_status` | `fer_world_model` |
| `GetGraspCandidates`, `GetPlaceCandidates` | service | `/grasp/candidates`, `/place/candidates` | `fer_grasp_planner` |
| `WorldObjectArray` | topic, latched | `/world_model/objects` | `fer_world_model` |

**World model.** Takes `vision_msgs/Detection3DArray` on `/perception/detections`, associates
detections with known objects (same class, centre within 0.05 m), owns object ids, tracks
status FREE / GRASPED / LOST, removes objects missing for 30 s. A fixed table (top at `base`
z = 0) is loaded from `fixtures.yaml`. Perception is a mock: `mock_perception` publishes the
objects of `config/mock_objects.yaml` at 5 Hz. On the real robot the objects must stand at
those poses.

**Gripper server.** One adapter per hardware. Real: `franka_gripper` `Move` / `Grasp` on
`/fer_gripper`, grasp evidence from `is_grasped`. MuJoCo: `GripperCommand` on
`gripper_effort_controller`, grasp evidence from the finger width. A successful grasp marks
the object GRASPED, relative to `fer_hand_tcp`; a watch marks it LOST when the grip is gone.

**Grasp planner.** Top-down candidates from the object's box (0, 2 or 4 per object), force per
class from `config/object_catalog.yaml` (one catalog for real and MuJoCo, forces ≤ 20 N);
place candidates for a held object at a target. Reachability is checked by the trees.

**Motion backend (`fer_moveit_motion_server` in `fer_moveit`).** Validates the goal, syncs the
world model into `move_group`'s planning scene, plans plan-only (OMPL `RRTConnect` for free
paths and joint targets, Pilz `LIN` for straight paths), executes through `ExecuteTrajectory`,
and stops a replaced or cancelled motion with `"stop"` on `/trajectory_execution_event`;
`CANCELLED` is reported once the arm rests. `may_touch` allows contact with named objects for
one plan. `ROBOT_ERROR` (reflex, user stop) on the real robot only.

**Behavior trees.** `PickPlace` (`trees/manipulation/`): detect, query objects of
`target_class`, per object `Pick` (retried `pick_attempts` times; not picked → next object),
`Place` in a row of slots, then home. SUCCESS only if every object was placed. Payload keys:
`target_class`, `pick_attempts`, optional `speed_scaling` (default 0.3, BT server parameter).
Named poses `home` and `view` in `config/poses.yaml`.

## 6. Settings that shape behaviour

- `joint_limits.yaml` is set to a quarter of the FER datasheet; `cartesian_limits.yaml`
  (0.6 m/s, ±0.9 m/s²) is sized to it near the ready pose.
- MoveIt execution duration monitoring is off (`fer_move_group.launch.py`). The motion
  server's watchdog (2 × duration + 7 s) is the only time limit on a motion.
- The trajectory controllers have no goal tolerances yet (to be set from measurements on the
  real robot); an uncancelled motion that does not reach its target reports `OK`.
- Real gripper force up to 70 N; MuJoCo up to 20 N. The catalog uses 5 N for boxes because of
  MuJoCo contact softness (§7).

## 7. Known limitations

- **Perception** is a mock: fixed detections from a file. An object outside the camera's view
  would count as missed and be removed after 30 s; there is no real perception node yet.
- **Planning failures** are always reported as `NO_PATH`: `move_group`'s plan-only path
  replaces planner error codes, so `UNREACHABLE` is never seen.
- **Straight paths** (Pilz LIN) are checked against the quarter-datasheet joint limits; near
  a singularity, or at high `speed_scaling`, they end `NO_PATH`.
- **Motion contract is goal-terminated.** `speed_scaling` is fixed per motion; nothing can
  slow a running motion (needed for speed and separation monitoring, tracking, handover).
- **Stopping:** a cancel during planning takes effect after planning; `"stop"` stops every
  execution in `move_group`; the slowing-down after a stop is not collision-checked.
- **Scene and world model:** objects written to `move_group`'s scene survive a motion server
  restart; poses use the latest TF, so the arm must be at rest for detection and grasping.
- **Grasp state:** if `SetObjectStatus` fails, or a `Grasp` is cancelled after closing, the
  hand can hold an object the world model lists as FREE. A failed lift leaves the object
  attached (no release fallback); a failed `Release` that did let go is marked LOST.
- **One tree at a time:** a goal sent while a tree runs waits for it, but its payload is
  applied at once. A `Pick` that fails after the grasp retries with the object in the hand.
- **MuJoCo:** `fer_joint7` lags its reference by up to 0.28 rad and needs about 0.8 s to
  settle; gripper closing speed is tied to the grasp force; the gripper needs
  `hand_control_type:=effort`.
- **Sim/real parity ends at joint level:** the real driver also exports Cartesian interfaces
  and the robot model; MuJoCo does not. No wrench (force/torque) path yet.
- **Driver** is frozen on libfranka 0.9.2 (FER only, no FR3).
- **SSM is not a safety function**; protective stops go through the Franka safety system.

## 8. Open

### 8.1 Not yet verified

- Real hand: `is_grasped` while holding, object taken out → LOST (`GripperState` publisher of
  the driver).
- Real motion: cancel without reflex, user stop → `ROBOT_ERROR`, final joint error for goal
  tolerances.
- A failed grasp followed by a successful retry.
- `fer_moveit`'s `test_contract` failed once when run together with another package's tests
  (likely a shared DDS domain).

### 8.2 Refinement (observed in recent runs)

1. At `speed_scaling` 0.6 planning runs into joint limits and produces odd arm motions.
2. After two failed grasps on the first object the tree moves on to the next object. This is
   the current policy; aborting instead is a decision still to make.
3. Start-up race between the simulation and the gripper server on the first start; the motion
   server is also not ordered after controller activation (10 s startup timeout).
4. Pilz LIN hits joint acceleration limits at `speed_scaling` 1.0 (quarter-datasheet limits).
5. Planned trajectories work but are often far from optimal (long joint-7 turns to the
   pre-grasp; `FindReachableGrasp` takes the first reachable candidate).
6. Execution duration monitoring: re-enable with slack for MuJoCo only (per-controller
   `allowed_goal_duration_margin`).
7. Box grasp force 5 N is a MuJoCo workaround shared with the real robot.
8. Code: the motion server watchdog mixes sim and wall clock; server parameters are read at
   start-up only but not declared read-only; fixed executor thread counts and polling
   intervals; `Gripper` is a `typing.Protocol` (ABC undecided).
9. RViz logs `trajectory_visualization: No robot state or robot model loaded`.

### 8.3 Infrastructure

- **Repository structure:** decided 2026-10-13 to move back to a single repository; layout
  and migration to be planned.
- **CI:** only `fer_ros2_bringup` has a workflow (description invariants). Missing: build and
  test of the whole stack, a `hardware:=mock` backend for launch tests without MuJoCo or the
  robot, checks for duplicate package names and dependency direction.
- **Conventions:** branch names differ (`main`, `jazzy`, `jazzy-devel`, `jazzy_devel`); no
  version tags; only the driver has a CHANGELOG.
- **Releases:** no pinned snapshot yet. Providing the code outside needs public repositories,
  `https` URLs in every manifest (`speed_and_separation_monitoring` uses `git@`) and a
  release snapshot per profile.
- **Driver lint:** `copyright` (9 files without a license header) and `lint_cmake`.

### 8.4 Deferred features

Real perception (D405 node, hand–eye calibration), grasp network behind
`GetGraspCandidates`, grasp monitor from the external wrench, Cartesian impedance control
(decide the dynamics model first), a streaming motion contract (tracking, handover, speed
scaling of a running motion), an MPC backend, integration of speed and separation monitoring,
multi-host deployment (per-host compose files, cross-host topic contract).
