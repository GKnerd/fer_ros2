# 2026-10-26 Observations: lift failure with a held box on the real robot

## Context

PickPlace on the real FER (2026-10-26, about 17:35) ended FAILURE. The robot was left holding
`box_1` (25 mm, uncommitted change in `mock_objects.yaml`). The trace below is read-only: logs
and live queries, no changes.

## What happened (from logs and live read-only queries)

1. Pick attempt 1: `Grasp` closed on nothing (`measured 0.0003 m, expected 0.0250 m`), backed
   out to the pre-grasp. Attempt 2: grasp succeeded at t=528.88.
2. Lift (`pick.xml:25`, `MoveToPose path="straight"`) failed at t=529.05:
   `CheckStartStateCollision failed: box_1 - table, fer_link0 - table`.
3. Then three `MoveToJoints` failed on the same start-state collision. With `pick_attempts=2`
   these are box_2 attempts 1 and 2 (view pose), then `home`, so the tree fails. Joint targets
   get no contact allowance by design.
4. No release fallback (known open item), so the box stays in the hand and in the scene as
   attached.

Live state after the failure: world model `box_1` GRASPED by `fer_hand_tcp`.
`/check_state_validity` (group `fer_arm`): `table`–`box_1` penetration **1.1e-5 m**. Without a
group it adds `fer_link0`–`table` at 4e-8 m. That pair only appears in the adapter's ungrouped
contact printout (link0 is outside `fer_arm`), so it is a **red herring**.

## Root cause

`fer_moveit/src/motion_server.cpp:109` calls `plan(target, …, std::nullopt, …)`. In
`move_group_client.cpp:plan`, two separate checks then each sample move_group's *current*
monitored state:

- `held_contacts(start)` → `/check_state_validity` at time t1 decides which held-object
  contacts are allowed;
- `CheckStartStateCollision` inside `/move_action` at t2 > t1 checks the start state again.

The box is attached at its world-model pose, bottom exactly at the table top (z=0.0125, half
size 0.0125). The contact depth is therefore ≈0 and its sign flips with µm-level drift of the
real arm after the grasp (effort controller, grasp force settling). At t1 there was no contact,
so `box_1–table` was not allowed. At t2 the box was in contact (11 µm afterwards), so the
start-state check rejected the plan. The contact filter itself is correct: live body types
WORLD_OBJECT=1 / ROBOT_ATTACHED=2 match `move_group_client.cpp:243-254`. This is inferred:
`held_contacts` does not log its result, but it is the only way the pair could be missing.

## Fix (proposed)

1. `fer_moveit/src/motion_server.cpp:109`: sample the start state once with
   `current_joints()` and pass it to `plan(...)` instead of `std::nullopt`, as
   `CheckReachable` already does (`motion_server.cpp:387`). Then `held_contacts` and the plan
   request evaluate the identical `request.start_state`, so the allowance and the start-state
   check always agree.
2. `move_group_client.cpp:held_contacts`: log the allowed pairs at debug/info, so the next
   trace does not need inference.
3. (Optional, separate decision) Allow pairs within a small distance as well, not only
   penetrating ones. That covers the zero-margin case even if the state moves between planning
   and execution. Not needed for this bug.

## Recovery of the robot state (by hand)

Motions are blocked until the box is released: every start state collides. The box rests on
the table, so opening in place leaves it there; the arm does not move.

1. Release (opens, sets `box_1` FREE at its current pose, clears the gripper server's held
   state, `gripper_server.py:264`):
   `ros2 action send_goal /gripper/release fer_interfaces/action/Release "{object_id: box_1, open_width: 0.08}"`
2. Check: `ros2 service call /world_model/query_objects fer_interfaces/srv/QueryObjects "{}"`
   → `box_1` status 0, `held_by` ''. move_group detaches it at the next motion goal
   (`sync_scene`); after that `/check_state_validity` (group `fer_arm`) → `valid=True`.
3. Move away with a joint goal (home); open fingers (80 mm) clear the 25 mm box.

Fallback if Release fails: `ros2 action send_goal /fer_gripper/move franka_msgs/action/Move
"{width: 0.08, speed: 0.05}"`, then `/world_model/set_object_status` with `status: 0`,
`held_by: ''` and the box's real pose in `base`, or restart `fer_manipulation`.

## Side findings (not causal)

- The first grasp missed with the new 25 mm boxes: check that `mock_objects.yaml` poses match
  the physical boxes.

