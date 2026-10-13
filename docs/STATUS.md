# Status

Updated: 2026-10-13

## Current step

Single repository `fer_ros2` (2026-10-13): the eight package repositories are merged with
their full history into `ros2_ws/src`; the state before the merge is tag `jazzy-0.1.0`. The
Dockerfile clones libfranka and the externals at their pins into `~/deps` and `~/deps_ws`;
MuJoCo is built with `WITH_MUJOCO=1` (default on x86_64). Image built from scratch and the
stack verified on x86_64 (PickPlace SUCCESS in MuJoCo) and on the Orin (real robot).

Released `jazzy-0.2.0` (2026-10-13): `main` is the default branch. Rulesets protect
`jazzy_devel` (pull request, 1 approval, check `description-invariant`), `main` (updates by
Admin only) and the `jazzy-*` tags. The eight package repositories were deleted on GitHub;
their history is in `ros2_ws/src`.

CI for the whole stack (2026-10-13): `.github/workflows/fer_ros2.yml`, job `build-and-test`,
builds `docker/Dockerfile` with the GitHub Actions layer cache and runs `colcon test` for every
package inside the image, on x86_64 (`WITH_MUJOCO=1`) and on aarch64 (`ubuntu-24.04-arm`,
`WITH_MUJOCO=0`, the Orin image). Triggers: pull requests, pushes to `jazzy_devel` and `main`,
by hand, and weekly without cache. First run green on both architectures. `colcon test` passes
for every package, the driver included (license headers added).

`hardware:=mock` (2026-10-13): `mock_components/GenericSystem`, position only, no physics,
accepted up to `fer_manipulation.launch.py`. The launch test
`fer_ros2_bringup/test/manipulation_mock_test.launch.py` starts `fer_manipulation` on mock
and waits for the world model, grasp planner, motion server and gripper server; it passes in
a local container and runs in CI with the rest of `colcon test`.
Next step: package versions and changelog.

All own C++ code is formatted with clang-format 18 (braces on their own line) since
2026-10-12: one `.clang-format` in `ros2_ws/src`.

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


## Next

1. CI follow-up: add `build-and-test (x86_64)` and `build-and-test (aarch64)` as required
   checks on `jazzy_devel`; watch the first GitHub runs of the mock launch test (the motion
   server waits 10 s for `move_group`, see `docs/Platform.md` §8.2 item 3); later remove
   `description-invariant` and `fer_description.yml`, whose test `build-and-test` also runs,
   and update the CI lines in `CLAUDE.md` and `CONTRIBUTING.md`.
2. Package versions and changelog (all `package.xml` are at 0.0.0); decide before the next
   release. Open points in `docs/Platform.md` §8.3.
3. Refinements observed in sim and real runs (`docs/Platform.md` §8.2) and the open items
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
  Likely tests of different packages sharing a DDS domain; not yet investigated. CI tests
  one package at a time (`--executor sequential`).
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
| 2026-10-13 | One repository `fer_ros2` with all own packages, driver included; histories merged | The packages always change together; one place for CI, review and releases; the driver can be split out again with its history |
| 2026-10-13 | Externals cloned by the Dockerfile at pinned commits into `~/deps_ws`; no vcs, no submodules | A pin changes in the same commit as the code that needs it; code changes do not rebuild the externals |
| 2026-10-13 | `main` = releases tagged `jazzy-<version>`; `jazzy_devel` = development through pull requests | A plain clone gets verified code; one branch for all work |
| 2026-10-13 | One `.clang-format` in `ros2_ws/src` | The container sees only `ros2_ws`; the clang-format tests read it by path |
| 2026-10-13 | Old package repositories deleted, not archived | They served no purpose; their history is in `ros2_ws/src` |
| 2026-10-13 | CI builds `docker/Dockerfile` and runs `colcon test` in the image | The Dockerfile is the only place the pins live; CI tests the image the robot runs, and a red run reproduces with `build_image.sh` |
| 2026-10-13 | CI images: x86_64 with MuJoCo, aarch64 without | The two images that are used; other combinations would block merges over setups nobody runs |
| 2026-10-13 | CI tests run with Cyclone defaults (`CYCLONEDDS_URI` unset) | `env/` is not in the image, and the config's 10 MB minimum receive buffer needs a raised `net.core.rmem_max` that GitHub runners do not have |
| 2026-10-13 | Driver license headers by origin: Franka where the file or its header already names it, Seongjin Bien (multipanda_ros2) otherwise | `ament_copyright` passes; credit follows the upstream history |
| 2026-10-13 | Own packages carry no per-file license headers; the always-skipped `test_copyright.py` removed from the Python packages | The repository `LICENSE` covers them; the CMake packages already switch the check off |
| 2026-10-13 | `hardware:=mock` with an own overlay (`mock_components/GenericSystem`, position only) | Launch tests without MuJoCo or the robot; upstream's `use_fake_hardware` block has no finger joint, so no gripper controller could run |
