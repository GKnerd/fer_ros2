# fer_ros2

ROS 2 **Jazzy** driver stack for the real Franka Emika Robot
("FER" / "Panda"). Firmware version 4.2.1, `libfranka` version 0.9.2. 

## Provenance

This repository started as unmodified import of the packages below from
[`tenfoldpaper/multipanda_ros2`][multipanda], branch `humble`, commit
[`ad741e3`][pinned-commit], with every subsequent change committed on top.
That keeps the diff against upstream inspectable and lets upstream fixes
be cherry-picked deliberately.

Packages carried over from multipanda_ros2:
`franka_bringup`, `franka_example_controllers`, `franka_gripper`,
`franka_hardware`, `franka_msgs`, `franka_multi_mode_controller`,
`franka_robot_state_broadcaster`, `franka_semantic_components`.

multipanda_ros2 is itself built on top of [mcbed's Humble port][mcbed] of
`franka_ros2`, which was originally authored by **Franka Emika GmbH**. See
[`NOTICE`](./NOTICE) for the full attribution chain and license notices.

## Why a Jazzy port

multipanda_ros2 targets ROS 2 Humble and ros2_control 3.x. This port:

- Updates to ROS 2 Jazzy / ros2_control 4.x APIs.
- Strips simulation: MuJoCo, `franka_hardware/src/sim`, sim launch/config,
  and the `mujoco_ros2_control` CMake dependency.
- Drops (for now) multi-arm bringup surface (single FER only).
- Re-applies known Jazzy compatibility fixes (realtime publisher
  `trylock()` handling in `franka_robot_state_broadcaster`, `--param-file`
  on controller spawners) that are either absent or solved differently
  upstream.

MoveIt configuration is maintained separately in [`fer_moveit_config`][fer-moveit],
not `franka_moveit_config` from upstream.

## Information

Operational notes for running this stack on a real arm. These document behaviour that is
specific to this fork and not obvious from the code.

### An "active" controller does not mean the arm is commandable

This is the single most important thing on this page. `ros2 control list_controllers`
showing `active`, and `ros2 control list_hardware_interfaces` showing `[claimed]`, are
**not** evidence that commands reach the robot. Both can be true while no libfranka
control loop exists, in which case `write()` stores torques into a buffer that no thread
reads and the arm silently ignores everything.

The only reliable check:

```bash
ros2 topic echo /franka_robot_state_broadcaster/robot_state --once \
  | grep -E "robot_mode|control_command_success_rate"
```

- `robot_mode: 2` (MOVE) with a non-zero success rate → a commanding loop is running.
- `robot_mode: 1` (IDLE) with `0.0` → read-only continuous reading. The arm will not move.

`robot_mode` values: 0 OTHER, 1 IDLE, 2 MOVE, 3 GUIDING, 4 REFLEX, 5 USER_STOPPED,
6 AUTOMATIC_ERROR_RECOVERY.

### Recovering from a fault

A hardware fault (reflex, collision, comms violation) deactivates the controller **and**
the hardware component. This is deliberate: a fault never auto-resumes a commanding loop.
You will see:

```
[ERROR] Arm 'fer' is in an error state; commands are not reaching the robot. Deactivating.
[ERROR] Deactivating controllers [effort_joint_trajectory_controller ] as their command
        interfaces are tied to DEACTIVATEing hardware components
```

Recovery is three steps, **in this order**:

```bash
# 0. Clear the physical cause first. The arm must not still be in contact, or
#    automaticErrorRecovery() will simply fail.

# 1. Clear the robot error. Also restarts read-only continuous reading.
ros2 service call /fer_error_recovery_service_server/error_recovery \
  franka_msgs/srv/ErrorRecovery {}

# 2. Bring the hardware component back.
ros2 control set_hardware_component_state fer_FrankaMultiHardwareInterface active

# 3. Now the controller can claim its interfaces.
ros2 control switch_controllers --activate effort_joint_trajectory_controller

# 4. Verify a COMMANDING loop exists -- not just an active controller.
ros2 run fer_bringup fer_sanity_check.py --duration 5
```

**Order matters in both directions.** Run step 2 before step 1 and the next `write()`
cycle sees `hasError()` still true and deactivates the component again immediately. Run
step 3 before step 2 and you get `Could not switch controllers since prepare command
mode switch was rejected` — a controller cannot claim interfaces belonging to an inactive
component.

What each step does inside the driver:

| Step | Effect |
|---|---|
| 1 | `doAutomaticErrorRecovery()`, `setError(false)`, `initializeContinuousReading()` — read-only |
| 2 | `on_activate()` → `arm.control_mode_ = None`, restart continuous reading, zero effort commands |
| 3 | `prepare_command_mode_switch` sees `control_mode_ == None` so the start guard passes → `perform_command_mode_switch` → `stopRobot()` + `initializeTorqueControl()` |

Step 3's precondition is why steps 1 and 2 are not optional ceremony: they put the state
machine back where the switch will be accepted.

### Switching control modes

Only one control mode can be active per arm, and all 7 joints must switch together. The
switch is **atomic at the ros2_control level** but a genuine stop/restart at the
libfranka level — `stopRobot()` joins the control thread and calls `robot_->stop()` over
the network before the new loop starts.

Put both lists in a **single** request. Activating without deactivating is rejected,
because `prepare_command_mode_switch` refuses a start while `control_mode_ != None`:

```bash
ros2 control switch_controllers \
  --deactivate effort_joint_trajectory_controller \
  --activate vel_joint_trajectory_controller
```

Two sequential calls also work, but leave the arm briefly in continuous reading.

### Velocity controller gains

`vel_joint_trajectory_controller` runs low PID gains (`p: 2.0, d: 0.0`) deliberately —
high outer-loop gains in velocity mode fight the FER's internal velocity controller.

Be aware, though, that `ff_velocity_scale` defaults to **0.0** and `controllers.yaml`
does not set it. JTC's velocity-mode command is

```
cmd[i] = ff_velocity_scale[i] * desired_velocity[i] + PID(pos_error[i], vel_error[i], dt)
```

so at `ff = 0.0` the PID output *is* the entire velocity command and the trajectory's own
velocity profile is discarded. Setting `ff_velocity_scale: 1.0` would let the trajectory
velocity pass through as the primary command, with the PID reduced to a drift trim.
**Untested on hardware** — see TODO.

### Diagnostic scripts

```bash
ros2 run fer_bringup fer_sanity_check.py [--duration 30]   # passive; moves nothing
ros2 run fer_bringup fer_mode_switch_check.py [--cycles 3] # effort <-> velocity
```

`fer_sanity_check.py` checks topic rates, joint validity, the TF chain (`base` →
`fer_hand_tcp` spans `/tf_static` and `/tf`, so `ros2 topic echo /tf` alone misses both
ends), control-loop liveness, and that controllers *stay* active over the hold period.
That last part matters: the failure it guards against is a broadcaster that comes up
active and dies seconds later, which a single sample at t=0 passes straight through.

`fer_mode_switch_check.py` asserts the start-without-stop guard and measures the dead
window across an atomic switch.

For communication quality, use libfranka's own `communication_test` example rather than
anything in this repo — it measures success rate and missed packets directly.

## License

Apache License 2.0 — see [`LICENSE`](./LICENSE) and [`NOTICE`](./NOTICE).

[multipanda]: https://github.com/tenfoldpaper/multipanda_ros2/tree/humble
[pinned-commit]: https://github.com/tenfoldpaper/multipanda_ros2/tree/ad741e3ffcbe44cf47f7138ac8659802a3c9e43b
[mcbed]: https://github.com/mcbed/franka_ros2/tree/humble
[fer-moveit]: https://github.com/GKnerd/fer_moveit_config
