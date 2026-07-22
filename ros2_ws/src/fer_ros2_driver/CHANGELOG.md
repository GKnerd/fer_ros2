## 2026-07-22

### franka_robot_state_broadcaster
- Fixed both broadcasters deactivating shortly after activation (state + model).
  `update()` returned `return_type::ERROR` on a failed `RealtimePublisher::trylock()`;
  in ros2_control 4.x (Jazzy) a single ERROR from `update()` deactivates the
  controller. A `trylock()` miss is normal backpressure (the non-realtime publish
  thread holds the mutex that cycle), not a fault. Changed the `else` branch to
  `return OK` (skip the cycle) in both `src/franka_robot_state_broadcaster.cpp` and
  `src/franka_robot_model_broadcaster.cpp`. (TODO item 1)
- Same fix applied to the read-failure branch: `get_values_as_message()` returning
  false now returns `OK` instead of `ERROR`. After the 2026-07-20 `get_optional()`
  migration this branch also fires on a *transient* interface-read miss, which should
  skip the cycle, not deactivate the controller. A genuinely missing interface still
  fails earlier at `on_activate`. Log level lowered `RCLCPP_ERROR` →
  `RCLCPP_WARN_THROTTLE` (1 s) so a persistent failure stays visible without flooding
  the log at the publish rate. (TODO item 1a)

### franka_hardware
- Fixed thread/state handling in `src/real/robot.cpp` after a `franka::ControlException`
  (e.g. a timing violation on a non-RT kernel). (TODO item 3)
  - Every control/read thread's `catch` block now sets `stopped_ = true` in addition to
    `setError(true)` (5 control inits + both catches in `initializeContinuousReading`).
    Previously `stopped_` was left `false` when the loop died, so the object reported an
    active control loop that no longer existed — `read()` then called `readOnce()` on a
    still-unwinding libfranka context, cascading `InvalidOperationException` and pushing
    the robot into Reflex mode.
  - `stopRobot()` reworked so the thread is joined on **both** exit paths (guarded by
    `joinable()`). Setting `stopped_ = true` in the catch means the old
    `if (!stopped_)` body was skipped, leaving the finished thread unjoined — the next
    `initialize*()` then replaced (destroyed) a joinable `std::thread`, which calls
    `std::terminate()`. The fix moves the join out from behind that guard.
  - On the error path `stopRobot()` no longer calls `robot_->stop()` a second time (the
    libfranka session already closed when `control()` threw); it only reaps the thread.
    The redundant `stop()` was a per-cycle network round-trip that caused ~4 ms read
    overruns.
  - `stopped_` changed from plain `bool` to `std::atomic_bool` (it is now written from
    the control thread and read from the hardware thread). `has_error_` was left as-is —
    it is already guarded by `error_mutex_` via `hasError()`/`setError()`.
- NOT YET VERIFIED ON HARDWARE. The `std::terminate` and redundant-`stop()` fixes follow
  from static/caller analysis; the `readOnce()` cascade depends on libfranka unwinding
  internals. Confirm on the real arm (force a timing violation, expect clean recovery,
  not Reflex). `read()` ordering (`readOnce()` before `stopRobot()`) was left unchanged.

### fer_bringup (new package — real-robot bringup)

Not part of `fer_ros2`; lives at `src/fer_bringup` (sibling of the fork). Documented here
to keep the migration history in one place. **NOT YET LAUNCHED ON HARDWARE** — pending a
`colcon build` and an expand/inspect of the generated URDF.

- **New real-robot bringup launch** (`launch/fer_bringup.launch.py`): builds the URDF from a
  local xacro overlay, then starts `robot_state_publisher`, `joint_state_publisher`,
  `ros2_control_node`, the controller spawners, the `franka_gripper` action-server node, and
  (optionally) RViz. `OpaqueFunction` + declared-args structure; sim/gazebo/fake-hardware
  paths intentionally excluded (real arm only).

- **Local URDF overlay to resolve the description/hardware lineage mismatch**
  The migrated upstream `franka_description` emits `<plugin>franka_hardware/FrankaHardwareInterface</plugin>`
  with a param/interface contract this workspace's multipanda-fork hardware does not implement.
  Overlay instead of forking the description:
  - `urdf/fer.xacro` — includes upstream `franka_robot.xacro` for Bucket A
    (links/visuals/limits/inertials/kinematics) but passes `ros2_control:=false` to suppress
    upstream's incompatible `<ros2_control>` block, then adds our own. Upstream stays untouched.
  - `urdf/fer_ros2_control.xacro` — emits a `<ros2_control>` matched to
    `franka_hardware::FrankaMultiHardwareInterface`: `robot_count=1`, `ns1`, `robot_ip1`, and
    7 joints each with exactly 3 command (effort/position/velocity) + 3 state
    (position/velocity/effort) interfaces, per what `on_init` validates. Cartesian/model/
    robot_state interfaces are created programmatically by the hardware, so they are NOT
    declared here. Joint names are `<arm_id>_joint1..7`; `arm_id` is derived from `robot_type`
    (+ `arm_prefix`) because the hardware's `get_ns()` parses the namespace from the joint name
    — `arm_id` must equal `ns1`, so it is deliberately not an independent knob.
  - Launch now expands `fer_bringup/urdf/fer.xacro` (not the upstream `robots/fer/fer.urdf.xacro`).
    Overlay is launch-driven: `robot_ip` direct, `arm_id`/`ns1` via `robot_type`/`arm_prefix`.

- **joint_states wiring for the real arm.** `ros2_control_node` remaps
  `joint_states -> fer/joint_states` (the `joint_state_broadcaster` runs inside that process),
  and `joint_state_publisher`'s `source_list` merges `fer/joint_states` (arm) +
  `fer_gripper/joint_states` (gripper node, named `[arm_id]_gripper`) into `/joint_states` for
  `robot_state_publisher`. Replaces the stale `franka/…` + `panda_gripper/…` names. The `fer`
  hardcoding is logged under TODO item 5 (dynamic naming).

- **Controllers** (`config/controllers.yaml`, spawned via a single `controller_spawner` helper
  with `--param-file` so `generate_parameter_library` controllers get their params — TODO item 2):
  - Active on start: `joint_state_broadcaster`, `franka_robot_state_broadcaster`.
  - Loaded inactive (switch at runtime): `effort_joint_trajectory_controller`,
    `vel_joint_trajectory_controller`. No motion controller is active at launch, so the arm
    comes up uncommanded until an operator activates one.
  - `franka_robot_model_broadcaster` is intentionally not spawned — nothing here consumes the
    dynamics-model topic (the JTCs run their own PID). Add it only for a model-based controller.

- **Packaging:** `CMakeLists.txt` now installs `launch config rviz urdf`; `package.xml` gained
  the runtime deps (xacro, franka_description, controller_manager, franka_hardware,
  joint_state/joint_trajectory controllers, franka_robot_state_broadcaster, franka_gripper,
  robot_state_publisher, joint_state_publisher, rviz2, ros2launch).

- **Gripper has no ros2_control controller** — expected; it's driven by the `franka_gripper`
  action server, outside the controller_manager. Flagged for investigation as TODO item 8.

## 2026-07-20

### franka_semantic_components
- `src/franka_robot_state.cpp` — `get_robot_state_ptr()`, `get_values_as_message()`:
  replaced deprecated `LoanedStateInterface::get_value()` with `get_optional()`.
  Why: `get_value()` collapses a failed read into quiet NaN; this code `bit_cast`s
  that value into a `franka::RobotState*`, so a failed read became a wild pointer
  dereference. `get_optional()` lets us detect the failure before casting.
  Also converted the interface-existence check to an early return.
- `src/franka_robot_model.cpp` - `update_state_and_model()` also has the same changes in
  `src/franka_robot_model.cpp:66-67`.

### franka_hardware
- Removed stale `#include <hardware_interface/visibility_control.h>` from
  `include/franka_hardware/real/franka_multi_hardware_interface.hpp`.

### franka_robot_state_broadcaster
- **Behavioral change:** default `arm_id` changed from `"panda"` to `"fer"` in both
  broadcasters (`on_init()`, `auto_declare<std::string>`) and in
  `config/franka_robot_state_broadcaster_parameters.yaml` (`default_value`).
- Moved `franka_robot_state_broadcaster_parameters.yaml` from `src/` to `config/`, and
  updated the `generate_parameter_library()` path in `CMakeLists.txt` to match.
  The build breaks if only one of the two is changed.
- Updated deprecated header from `#include <realtime_tools/realtime_publisher.h>` to
  `#include <realtime_tools/realtime_publisher.hpp>` (`.h` form removed in Jazzy),
  in both `franka_robot_state_broadcaster.hpp` and `franka_robot_model_broadcaster.hpp`.
- Removed deprecated header `#include <rclcpp/qos_event.hpp>` from
  `src/franka_robot_state_broadcaster.cpp` and `src/franka_robot_model_broadcaster.cpp`.

