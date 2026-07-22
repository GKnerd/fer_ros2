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
Builds clean. **NOT YET VERIFIED ON HARDWARE** — force a timing violation on the real
arm and confirm clean recovery instead of Reflex. `read()` ordering (`readOnce()` before
`stopRobot()`) was deliberately left unchanged; revisit if a cascade still appears.

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
