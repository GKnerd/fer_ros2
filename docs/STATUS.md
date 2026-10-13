# Status

Updated: 2026-10-13

## Current step

Phase 8.7 implemented and committed (2026-10-12): old stack removed (`fer_skills`, old BT
server, skills launch levels, MTC capability), `fer_moveit_config` renamed `fer_moveit`,
`fer_ros2` renamed `fer_ros2_driver`, MoveIt level `fer_moveit_and_control.launch.py` forwards
every argument. `colcon test` green for `fer_behavior_trees`, `fer_moveit`, `fer_ros2_bringup`;
PickPlace SUCCESS in MuJoCo. First stage of migration finished. 
Next Step: Monorepo discussion.

All own C++ code is formatted with clang-format 18 (braces on their own line) since
2026-10-12: one `.clang-format` in `fer_behavior_trees`, `fer_moveit` and `fer_ros2_driver`.

## Phase 8

| Step | State |
|---|---|
| 8.0 Groundwork | done |
| 8.1 `fer_world_model` | done, committed |
| 8.2 `fer_gripper_server` | done, committed |
| 8.3 `fer_moveit_motion_server` | done, committed; real-robot checks open |
| 8.4 `fer_grasp_planner` | grasp and place candidates done, committed |
| 8.5 `fer_behavior_trees` | implemented, committed, PickPlace runs in MuJoCo; a failed grasp followed by a successful retry not yet seen |
| 8.6 Bringup | done |
| 8.7 Removal and renames | implemented, committed, MuJoCo verified; real-robot check and GitHub renames done, Fresh import verified on PC and Orin |

Not committed: `fer_ros2_driver/Repo_Hotfixes.md` (kept local).

## Next

1. Monorepo: decide the layout of the single repository and how to move the component
   repositories into it (history, branches, profiles, image build). `docs/Platform.md`
   describes the current layout.
2. After the move: CI, conventions (branch names, versions, changelog) and a first release.
   Open points in `docs/Platform.md` §8.3.
3. `fer_ros2_driver` packages: `colcon test` still fails on `copyright` (8 files in
   `franka_hardware`, 1 in `franka_semantic_components` without a license header) and
   `lint_cmake` (trailing space, `franka_gripper/CMakeLists.txt:43`). Both predate the
   reformat.
4. Refinements observed in sim and real runs (`docs/Platform.md` §8.2) and the open items
   below. The three reviews of 2026-10-12 (code issues, tests and CI, reuse from
   `multipanda_ros2`) are not yet in the list below.

## Open items

- **Execution duration monitoring is off** in `fer_move_group.launch.py` (joint 7 lags its
  plan by up to 0.28 rad). The motion server watchdog (2×T + 7 s) is the only time limit. With
  MoveIt's defaults (on, 1.1×T + 0.5 s) PickPlace failed in MuJoCo on 2026-10-12: pre-grasp
  moves stopped at the limit. Options: a per-controller `allowed_goal_duration_margin` (about
  1.0 s) in `moveit_controllers_mujoco.yaml`, so only MuJoCo gets slack; global scaling 1.5;
  `FindReachableGrasp` prefers the least joint-7 travel.
- **Catalog `box: {force: 5.0}`** is shared with the real robot. Alternative: stiffer object
  contacts in `pick_place_world.xml` (solref 0.004 1, solimp 0.95 0.99 0.001), back to 15 N.
- Grasp closing speed is tied to force (finger trajectory controller later).
- A failed lift leaves the box attached (no release fallback).
- A failed Release makes the watch mark the object LOST.
- World model can disagree with the physical grasp if `SetObjectStatus` fails or a Grasp is
  cancelled after closing.
- `Gripper` in `core/grasp.py` is a `typing.Protocol`; switching to an ABC is undecided.
- `MultiThreadedExecutor(num_threads=…)` fixed in both servers; named constant for the
  0.05 s / 0.1 s stop-poll intervals.
- Real robot: 8.3 checks, `is_grasped`.
- magic numbers on the polling interval in `fer_gripper_server` and in `fer_world_model`. 
- Protocol used for the Gripper instead of ABC, check if is worth to change it up.
- Fixed executor thread count
- grasp-versus-world-model mismatch - needs to be watched at runtime
- MuJoCo scene objects and how they are added to the world_model when sim is used
- `fer_moveit` `test_contract` failed 2 of 17 once when its tests ran in the same
  container together with `fer_behavior_trees`, and passed in every separate run (2026-10-12).
  Likely tests of different packages sharing a DDS domain; not yet investigated.
- Speed and separation monitoring: `speed_scaling` is fixed per motion when it is planned and
  cannot change during execution, so SSM (speed depending on the distance to a person) needs a
  mechanism that scales a running motion. How `speed_and_separation_monitoring` fits is open.
- RViz logs `trajectory_visualization: No robot state or robot model loaded` repeatedly in the
  MuJoCo run of 2026-10-12; not known whether it appeared before 8.7.

## Known limits

- `move_group` in plan-only mode replaces planner error codes with FAILURE, so every planning
  failure reports NO_PATH. Trees never branch on UNREACHABLE.
- Objects written to `move_group`'s scene survive a motion server restart.
- MuJoCo gripper needs `hand_control_type:=effort`; closed on nothing, `fer_finger_joint1` sits
  ~0.85 mm below its lower limit.

## Decisions

| Date | Decision | Reason |
|---|---|---|
| 2026-09-26 | Skip Phase 2 (all sim command interfaces) | Servers pick their controller at startup; no runtime switching needed |
| 2026-09-28 | Servers never switch controllers | Activation stays explicit, not hidden in adapters |
| 2026-09-29 | Motion server is a `move_group` client | Keeps the MotionPlanning panel, one parameter set, tested MoveIt execution |
| 2026-09-30 | Pick retried `pick_attempts`; failed Place ends the tree; SUCCESS only if all placed | Simple first policy, refine later |
| 2026-09-30 | Place target orientation = object's full orientation; `place_clearance` 0.005 | Literal contract; must be >0 (attached box vs table) |
| 2026-10-02 | Robot not raised in the MuJoCo scene | Would need a description change or a MuJoCo-only offset |
| 2026-10-02 | MuJoCo damping unchanged | Same dynamics worked with the old stack |
| 2026-10-12 | Trees take `speed_scaling` from the goal payload, default a BT server parameter (0.3) | Slower runs (first real-robot runs) without editing trees |
| 2026-10-12 | A top-level launch file declares and forwards every argument of what it includes | The top level has authority over all lower levels |
| 2026-10-12 | C++ formatting by clang-format 18, `{` on its own line everywhere | Formatting comes from the editor (format on save) and matches the `colcon test` check |
| 2026-10-12 | `fer_moveit_config` → `fer_moveit`; `fer_moveit_launch.py` → `fer_move_group.launch.py` | The package holds the MoveIt configuration and the MoveIt motion backend, not only configuration |
| 2026-10-12 | Bringup MoveIt level → `fer_moveit_and_control.launch.py`, kept as its own level | Hardware and `move_group` without servers or automatic controller activation, for manual planning and first controller runs |
| 2026-10-12 | `move_group` loads no MTC capability | Only `fer_skills` used it |
| 2026-10-12 | Execution duration monitoring stays off in 8.7 | MoveIt's defaults failed PickPlace in MuJoCo on the joint-7 lag; changing it is an open item |
