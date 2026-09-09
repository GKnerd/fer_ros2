# TODO

Known issues and parked decisions for this project. 

---

## 1. Re-apply: broadcaster deactivates after ~2.7 s (`trylock()` failure returns ERROR)

**Status:** DONE 2026-07-22 (both broadcasters). Builds clean.

**Files:**
- `franka_robot_state_broadcaster/src/franka_robot_state_broadcaster.cpp` — `update()`, `else` branch
- `franka_robot_state_broadcaster/src/franka_robot_model_broadcaster.cpp` — `update()`, `else` branch (~line 120)

**Cause:** `RealtimePublisher::trylock()` returns `false` whenever the non-realtime
publishing thread momentarily holds the mutex. At 1000 Hz this happens on virtually
every publish cycle. The `else` branch returns `return_type::ERROR`, and in Jazzy's
ros2_control 4.x a single ERROR from `update()` causes immediate controller
deactivation — hence the broadcaster surviving ~2.7 s and then dying.

**Fix:** in the `else` branch, `return ERROR` → `return OK`. A failed `trylock()` is
normal backpressure, not an error; skip this cycle and publish on the next one.

**Applies to both broadcasters** — the original fix covered only the state broadcaster.

### 1a. Related risk introduced by the 2026-07-20 `get_optional()` migration

**Status:** DONE 2026-07-22. Read-failure branch now returns `OK` + throttled WARN in
both broadcasters, matching the trylock decision below.

Both `update()` methods have a *second* `return ERROR`, on
`get_values_as_message()` returning false. That path changed meaning today:

- Before: `get_value()` returned quiet NaN on a failed read and
  `get_values_as_message()` still returned `true`. Only a *missing interface*
  returned false.
- Now: a transient failed read (`get_optional()` returning `nullopt` after its 10
  retries) returns `false` → `update()` returns ERROR → controller deactivation.

So the hardening may reintroduce the same "dies after a few seconds" symptom through
a different door. Decide whether a transient read failure should be ERROR
(deactivate) or OK (log and skip the cycle, like the `trylock` case). A missing
interface is genuinely fatal; a transient lock miss is not. These two probably
should not share a return path.

---

## 2. Re-check: controllers using `generate_parameter_library` need `--param-file` on the spawner

**Status:** NOT APPLICABLE AS WRITTEN — `cartesian_impedance_controller.launch.py`
does not exist in this tree, and no launch file under `fer_ros2` currently passes
`--param-file`. Re-evaluate during the `franka_bringup` refactor.

**Cause:** in ros2_control 4.x (Jazzy), controllers built with
`generate_parameter_library` must receive parameters via `--param-file` on the
spawner. The Humble-era implicit inheritance from the controller_manager's parameter
server no longer works for required parameters such as `joints`.
`franka_robot_state_broadcaster` was unaffected because it uses `auto_declare<>`
with defaults.

**Fix:** add `--param-file <bringup controllers.yaml>` to the spawner arguments for
any controller that actually consumes a generated parameter library.

**Note:** see item 4 — no franka package in this workspace currently *consumes* a
generated parameter library, so this only bites once one does.

---

## 3. Re-apply: `robot.cpp` thread/state handling after `ControlException`

**Status:** DONE 2026-07-22 (all three fixes + `stopped_` made `std::atomic_bool`).
**PARTIALLY VERIFIED ON HARDWARE 2026-09-09.** See the correction below — an earlier
edit of this entry claimed full verification and was wrong.

Provoked via collision reflex rather than a timing violation: dropped the force/torque
thresholds through `/fer_param_service_server/set_force_torque_collision_behavior`,
pushed the arm by hand with `effort_joint_trajectory_controller` active, and the arm
entered Reflex. `/fer_error_recovery_service_server/error_recovery` returned success and
`ros2 control list_controllers` showed the controller active again.

**What that actually proved:** the thread/state handling in this item is sound. No
`InvalidOperationException` cascade, no `std::terminate`, no stuck Reflex. The trigger
differs from the original report but the code path does not — a reflex and a comms
timeout both surface as `franka::ControlException` out of `robot_->control(...)`, caught
at `robot.cpp:160-163`.

**What it did NOT prove, and what the first version of this entry wrongly asserted:**
that the arm could be commanded afterwards. It could not. Recovery restarted the arm in
*continuous reading* — read-only — and it stayed there. `robot_mode` was `IDLE` with
`control_command_success_rate: 0.0` while the controller reported `active` and all 7
effort interfaces reported `[claimed]`. Discovered ~40 minutes later when a T4 motion
run commanded a trajectory and the arm did not move. Root cause was item 8, not this
item. "The controller came back up" was true only at the ros2_control layer.

Lesson recorded in `fer_hardware_validation`: controller state is not evidence that the
arm is commandable. Only `FrankaState.robot_mode` / `control_command_success_rate` are.
T1 and T4 now assert this.

`read()` ordering (`readOnce()` before `stopRobot()`, `robot.cpp:88-98`) was deliberately
left unchanged and produced no cascade under this test — leaving it as is.

Still unexercised (low risk, structurally identical catch blocks): the joint-position,
joint-velocity, and cartesian control modes. A repeated/sustained comms timeout, as
opposed to this one-shot reflex, is also untested. **Re-run T3 after the item 8/9/10
fixes** — the whole sequence changed.

**File:** `franka_hardware/src/real/robot.cpp`

Three related fixes:

1. **Set `stopped_ = true` in every `catch` block** of all control and reading
   threads: `initializeTorqueControl`, `initializeJointPositionControl`,
   `initializeJointVelocityControl`, `initializeCartesianPositionControl`,
   `initializeCartesianVelocityControl`, `initializeContinuousReading`.
   Currently each catch only calls `setError(true)` (e.g. line ~153).
   When a thread exits via `ControlException` (e.g. a timing violation on a
   non-RT kernel), `stopped_` is left `false`, so `Robot::read()` calls
   `robot_->readOnce()` on a still-unwinding libfranka context — producing a
   cascade of `InvalidOperationException` and pushing the robot into Reflex mode.

2. **Refactor `stopRobot()` to always join the thread if joinable**, regardless of
   `stopped_`. Currently (line ~106) the whole body is guarded by `if (!stopped_)`
   and calls `control_thread_->join()` unconditionally inside it. This prevents
   `std::terminate()` from the destructor of an unjoined thread when
   `initializeTorqueControl()` replaces it.

3. **Separate the two stop cases.** If `stopped_` was already set by a catch block,
   only join the orphaned thread — do not call `robot_->stop()` again. Repeated
   `stop()` calls cause network round-trips at 1 kHz and were responsible for 4 ms
   read overruns.

---

## 4. Parked: adopt `generate_parameter_library` in `franka_robot_state_broadcaster`

**Status:** PARKED deliberately on 2026-07-20. Revisit during the `franka_bringup`
refactor. Nothing is broken. cleanup.

`config/franka_robot_state_broadcaster_parameters.yaml` is built by
`generate_parameter_library()` and linked, but never consumed. Both broadcasters
use `auto_declare()` + `get_node()->get_parameter()`, and nothing includes the
generated header or instantiates a `ParamListener`. Its `default_value` therefore has
no runtime effect.

Why it was parked rather than finished:

- **Not a local slip — inherited half-migration.** The same dead wiring exists in
  three franka packages: `franka_robot_state_broadcaster`,
  `franka_example_controllers` (`src/comless/model_example_controller_parameters.yaml`,
  also still `"panda"`), and `franka_multi_mode_controller` (declares the dependency
  with no schema yaml at all).
- **`arm_id` is inherently per-instance.** `dual_multimode.yaml` and
  `mixed_quad_multimode.yaml` run up to eight instances of these two controller
  classes, each with a different `arm_id` (`rl_left`, `rl_right`, `mj_left`, …).
  A single shared `arm_id` is not a meaningful goal.
- **The real source of truth is the bringup configs.** `franka_bringup/config/**.yaml`
  sets `arm_id` per controller instance via `ros__parameters`, which already overrides
  the C++ defaults. That mechanism works today and needs no `ParamListener`.

Adopting it would still buy typed/validated params and remove three dead files —
just not a shared `arm_id`. Decide as part of the bringup refactor.

---

## 5. Stale `arm_id: panda` in single-arm bringup configs

**Status:** OPEN.

- `franka_bringup/config/real/single_multimode.yaml` — `arm_id: panda` for both
  `franka_robot_state_broadcaster` and `franka_robot_model_broadcaster`
- `franka_bringup/config/real/single_controllers.yaml` — `arm_id: panda` for the
  state broadcaster and ~7 example controllers

`panda` appears in four distinct roles in these files; this is not one
find-replace:**

| Role | Example | Rename to `fer`? |
|---|---|---|
| `arm_id` values | `franka_robot_state_broadcaster: {arm_id: panda}` | yes — this is the actual fix |
| Joint names | `joints: [panda_joint1 … panda_joint7]` | only if the description generates `fer_joint*` |
| Gains keys | `panda_joint1: {p: 600., d: 30.}` | must track the joint names exactly |
| Controller instance names | `panda_joint_impedance_controller` under `multi_mode_controller` | separate decision — registered names, not arm ids |

Note that joint names do not come from `arm_id`. In `franka_description` they come
from `${robot_type}` (`robots/common/utils.xacro:202`,
`params="robot_type:=fer root:=fer_joint1 tip:=fer_joint7"`). `arm_id` and
`robot_type` are two independent knobs that both happened to be `panda`. Confirm the
bringup passes a consistent value to both rather than assuming.

**2026-07-22 — surfaced in `fer_bringup`.** The joint_states wiring in
`fer_bringup/launch/fer_bringup.launch.py` now hardcodes `fer`: the `ros2_control_node`
remaps `joint_states -> fer/joint_states`, and `joint_state_publisher`'s `source_list` is
`['fer/joint_states', 'fer_gripper/joint_states']`. The gripper node name
(`franka_gripper/launch/gripper.launch.py`) is `[arm_id, '_gripper']`, so it is
`fer_gripper` only because `arm_id` defaults to `fer`. This is the "dynamic naming" pain
point: the namespace should be derived from a single arm-id knob, not string-literal `fer`
scattered across launch files. Parked by user decision — revisit and make it dynamic.

---

## 6. franka runtime safety params (collision / impedance) do not come from the URDF

**Status:** OPEN. Decide during the bringup migration to the upstream `franka_description`.

Context: migrating to build the URDF from the upstream `franka_description` xacros, whose
joint/collision/impedance defaults are community-recommended and accurate. But only *some*
of those reach the arm.

**Two buckets:**

- **Bucket A — kinematics/dynamics + joint limits** (link masses, inertias, `<limit>`
  position/velocity/effort). Consumed by `robot_state_publisher`, MoveIt, and
  ros2_control joint-limit enforcement / controllers via `/robot_description`. These flow
  correctly from the URDF. No action needed.
- **Bucket B — franka runtime safety params** (joint impedance, cartesian impedance,
  collision thresholds). **NOT read from the URDF anywhere.** `franka_hardware`'s
  `on_init` only reads `robot_count`, `ns*`, `robot_ip*`, and joint names/interface types
  from `info_` (verified by grep of `franka_multi_hardware_interface.cpp`). The actual
  values are **hardcoded** in `Robot::setDefaultParams()`
  (`franka_hardware/src/real/robot.cpp:428`), called from the `Robot` constructor at
  connect time — so any collision/impedance defaults placed in the URDF are silently
  overwritten by these C++ literals.

**The trap:** carefully-chosen URDF collision/impedance values just don't take effect, with
no warning.

**Options to actually apply Bucket B values:**
1. Edit the literals in `setDefaultParams()` to match the upstream numbers (crude, but it's
   what runs today).
2. Call the existing `Set{Joint,Cartesian}Stiffness` / `Set…CollisionBehavior` services
   (`robot.cpp:325-421`) at bringup — must happen *after* connect, or `setDefaultParams()`
   in the constructor clobbers them.
3. Wire `setDefaultParams()` to read from `info_.hardware_parameters` and add matching
   `<param>`s to the URDF (the "correct" fix, most work).

Also note: `<param name="initial_value">` on state interfaces is consumed only by
ros2_control mock/fake hardware, not by the real franka plugin — don't expect it to do
anything on the real arm.

---

## 7. Local URDF overlay in `fer_bringup`

**Status:** DONE 2026-09-09 (referenced by `fer_bringup/urdf/*.xacro`; see CHANGELOG).
Upstream `franka_description`'s `ros2_control` block targets
`franka_hardware/FrankaHardwareInterface`, which is not registered in this workspace.
It is suppressed via `ros2_control:=false` and replaced by
`fer_bringup/urdf/fer_ros2_control.xacro`.

---

## 8. FIXED 2026-09-09: `Robot::control_mode_` was never set; recovery guessed a mode

**Status:** FIXED. Root cause of the "arm does not move after error recovery" incident.

`Robot::setControlMode()` had **zero callers** in the entire workspace, and
`ControlMode control_mode_;` (`robot.hpp`) had no initializer. `getControlMode()`
returned indeterminate memory that read as `0` = `ControlMode::None`, so
`FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery` always took its
continuous-reading branch while logging "Restarting the control loop. Current cm: ...".

The authoritative mode is `ArmContainer::control_mode_`
(`franka_multi_hardware_interface.hpp:55`), which is correctly initialized and correctly
maintained — the error path never touches it, so after a reflex it still read
`JointTorque`. The recovery service simply cannot see it: it is constructed with only
`arm.robot_` (`franka_multi_hardware_interface.cpp:123`). Two copies of the same
concept, one authoritative and one dead.

**Fix:** initialize `control_mode_{ControlMode::None}`, and remove the mode-guessing
branch from the recovery service entirely — it now always restores read-only continuous
reading and logs the re-activation commands. Policy: **a fault never auto-resumes a
commanding loop.** That deletes the second copy of the state rather than synchronizing
it.

**Consequence to know about:** recovery is now a 3-step operator sequence. Documented in
`fer_hardware_validation/HARDWARE_ACCEPTANCE.md`.

---

## 9. FIXED 2026-09-09: `write()` returned OK for commands it discarded

**Status:** FIXED. Root cause of the incident being *invisible* for 102 seconds.

`FrankaMultiHardwareInterface::write()` returned `return_type::OK` when
`arm.robot_->hasError()`. `ResourceManager::write()` uses that value to populate
`HardwareReadWriteStatus::failed_hardware_names` and deactivate failed components, so
returning OK asserted a healthy write. Controller stayed `active`, interfaces stayed
`[claimed]`, `joint_states` stayed at 1 kHz — and no torque reached the arm.

**Fix:** return `return_type::DEACTIVATE` plus a throttled `RCLCPP_ERROR`. `DEACTIVATE`
not `ERROR`: it routes through `on_deactivate()`, which resets `arm.control_mode_` to
`None` and calls `stopRobot()`, leaving the state machine somewhere a later switch will
accept. `ERROR` uses `on_error()` and skips that.

No transient case exists to justify the old behaviour: every `setError(true)` in a
control thread is paired with `stopped_ = true` on the following line, and the only
`setError(false)` is in the recovery service. `hasError()` means the loop is gone until
a human intervenes.

**Open question for MoveIt:** a mid-trajectory fault now aborts the goal and deactivates
the controller. Decide whether the planner layer should also cancel/replan, or whether
the operator sequence is acceptable.

---

## 10. FIXED 2026-09-09: `assert(isStopped())` compiled out in Release

**Status:** FIXED. Caused a hard `SIGABRT` of `ros2_control_node`.

All six `Robot::initialize*()` methods end by reassigning `control_thread_`, which calls
`std::terminate()` if the previous `std::thread` is still joinable. The precondition was
guarded only by `assert(isStopped())` — a no-op under `NDEBUG`, i.e. in the build that
actually runs. `on_activate()` (`franka_multi_hardware_interface.cpp:221-222`) calls
`initializeContinuousReading()` **without** a preceding `stopRobot()`, so re-activating
the hardware component while a reading loop ran aborted the process
(`terminate called without an active exception`).

**Fix:** new `Robot::ensureStopped()` (robot.hpp) replaces the assert in all six methods,
making them idempotent regardless of caller.

Same failure class as item 3's `stopRobot()` join fix. That one removed a route to it;
this removes the unenforced precondition. Only reachable once item 9 was fixed — before
that the component never self-deactivated, so nothing ever re-activated it.

---

## 11. Deprecated `on_init(const HardwareInfo&)` overload

**Status:** OPEN. Warning only; will break when the overload is removed.

`franka_multi_hardware_interface.cpp:21` calls
`hardware_interface::SystemInterface::on_init(info)`. Jazzy deprecates it in favour of
`on_init(const HardwareComponentInterfaceParams& params)`
(`hardware_component_interface.hpp:140`). Pre-dates this session's work.

---

## 12. Multi-arm latent bugs in `FrankaMultiHardwareInterface`

**Status:** OPEN. Harmless at `robot_count: 1`; both would misbehave with 2+ arms.

1. `prepare_command_mode_switch` classifies interface types with
   `all_of_element_has_string(start_interfaces, ...)` / `(stop_interfaces, ...)` — the
   **full** lists — rather than the per-arm `arm_start_interfaces` /
   `arm_stop_interfaces` it just built. Two arms switching to different modes in one
   call would be misclassified.
2. `write()`'s error branch `return`s out of the whole function rather than
   `continue`ing, so an error on one arm stops the write loop for every later arm.
   Currently benign — the return value deactivates the whole component anyway.

---

## 13. `ff_velocity_scale` is 0.0 for the velocity trajectory controller

**Status:** OPEN. Not yet tested on hardware.

`joint_trajectory_controller_parameters.hpp:100` defaults `ff_velocity_scale` to `0.0`
("Feed-forward scaling k_ff of velocity"), and `fer_bringup/config/controllers.yaml`
does not set it. JTC's velocity-mode command is

```
cmd[i] = ff_velocity_scale[i] * desired_velocity[i] + PID(pos_error[i], vel_error[i], dt)
```

so at `ff = 0.0` the PID output *is* the entire velocity command and the trajectory's own
velocity profile is multiplied by zero.

The low gains (`p: 2.0, d: 0.0`) are deliberate — high outer-loop gains in velocity mode
fight the FER's internal velocity controller. But with `ff = 0.0` those low gains are
also the only thing producing motion, which is the wrong end of the trade-off.

**To evaluate:** run the same trajectory at `ff = 0.0, p = 2.0` and `ff = 1.0, p = 2.0`
and compare tracking error *and* command smoothness (std-dev of successive command
deltas, sign-reversal count). If `ff = 1.0` tracks better and commands more smoothly, `p`
can drop further. Record the outcome here either way.

Effort mode is unaffected: `ff_velocity_scale: 0.0` is correct there, since feeding a
velocity into a torque command is dimensionally meaningless.

---

## 14. Untested paths in the driver

**Status:** OPEN. Carried over from the deleted hardware-validation package.

- **Only the joint control modes are exercised.** Cartesian pose/velocity and
  joint-position modes have structurally identical catch blocks in `robot.cpp` but have
  never been run on hardware.
- **A sustained or repeated comms fault** — as opposed to a single one-shot — is
  untested. Use libfranka's `communication_test` for link quality.
- **`franka_hardware`'s error path after the item 9 change** has been verified for a
  reflex only. The `Robot` constructor's `CommandException` path (`robot.cpp:49-53`) is
  untested.
- **T3/T4 as automated checks were dropped.** The reflex procedure is manual by nature
  (a human has to push the arm) and its old assertions became wrong once recovery
  stopped restoring the controller. If you re-run it, follow the recovery sequence in
  the README rather than expecting the controller to come back on its own.
