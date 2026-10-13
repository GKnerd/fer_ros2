# fer_behavior_trees

Behavior trees of the FER platform and the server that runs them. The trees decide order,
choices and retries; every step is a call to an interface of
[`fer_interfaces`](../fer_interfaces). The trees never see a trajectory, a planner or a
controller name.

Built on BehaviorTree.CPP 4 and BehaviorTree.ROS2: `fer_bt_server` is a
`TreeExecutionServer` and every node is a ROS action or service client.

## Interfaces

Served:

| Interface | Kind | Name |
|---|---|---|
| `btcpp_ros2_interfaces/ExecuteTree` | action | `/execute_tree` |

Used (one tree node each, default name in the node's `action_name` port):

| Node | Kind | Name |
|---|---|---|
| `MoveToPose`, `MoveToJoints` | action | `/motion/move_to_pose`, `/motion/move_to_joints` |
| `MoveGripper`, `Grasp`, `Release` | action | `/gripper/move`, `/gripper/grasp`, `/gripper/release` |
| `DetectObjects`, `RefineObject` | action | `/world_model/detect_objects`, `/world_model/refine_object` |
| `QueryObjects` | service | `/world_model/query_objects` |
| `GetGraspCandidates`, `GetPlaceCandidates` | service | `/grasp/candidates`, `/place/candidates` |
| `FindReachableGrasp`, `FindReachablePlace` | service | `/motion/check_reachable` |

## Trees (`trees/manipulation/`)

- **`PickPlace`**: detects, queries the FREE objects of `target_class`, then per object
  `Pick` (tried `pick_attempts` times) and `Place` in a row of slots on the table, then
  `home`. A failed pick moves on to the next object; a failed place ends the tree. SUCCESS
  only if every object was placed; no objects is SUCCESS.
- **`Pick`** (`object_id`): `view` pose, open the hand, refine the object, take the first
  reachable top-down grasp, pre-grasp, straight approach, grasp, straight lift. A failed
  grasp backs out to the pre-grasp and fails.
- **`Place`** (`object_id`, `x`, `y`): the first reachable place candidate with the
  object's bottom center at (x, y) in `base`, pre-place, straight down, release, straight
  retreat.

## Behaviour

- **Goal payload:** a flat JSON object of strings and numbers, written to the global
  blackboard. `PickPlace` reads `target_class`, `pick_attempts` and the optional
  `speed_scaling`; without it the parameter `speed_scaling` applies. Keys of the previous
  goal are removed first. A payload that is not a flat object, or that sets `home` or
  `view`, rejects the goal.
- **Outcome:** every node writes the `Outcome` of its call to its `outcome` port. `OK` is
  SUCCESS, anything else FAILURE. Without a result from the server: a rejected goal or
  request gives `INVALID_GOAL`, a cancelled goal `CANCELLED`, an unreachable server or a
  missed timeout `TIMEOUT`.
- **Halt** cancels the running goal of an action node.
- **Reachability:** `FindReachableGrasp` / `FindReachablePlace` try the candidates in order
  with `CheckReachable` (approach free, target and retreat straight, the hand allowed to
  touch the object) and output the first that passes.
- **Named poses** `home` and `view` are loaded onto the global blackboard at start-up; a
  missing pose or one without 7 values stops the server.
- **Logging:** every node status change is printed to stdout. Groot2 can attach on port
  `groot2_port`.

## Launch

Started by `fer_ros2_bringup`'s `fer_manipulation_bt.launch.py`, or on its own:

```bash
ros2 launch fer_behavior_trees fer_bt_server.launch.py
ros2 action send_goal /execute_tree btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: PickPlace, payload: '{\"target_class\": \"box\", \"pick_attempts\": 2, \"speed_scaling\": 0.1}'}" --feedback
```

| Argument | Default | Meaning |
|---|---|---|
| `params_file` | `config/fer_bt_server.yaml` | server parameters |
| `poses_file` | `config/poses.yaml` | named arm configurations |
| `use_sim_time` | `true` | simulated clock |
| `log_level` | `info` | |

## Parameters (`config/fer_bt_server.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `action_name` | `execute_tree` | name of the served action |
| `tick_frequency` | 100 Hz | |
| `groot2_port` | 1667 | |
| `behavior_trees` | `fer_behavior_trees/trees/manipulation` | tree folders loaded at start-up |
| `action_timeout` | 2000 ms | until an action server accepts a goal |
| `service_timeout` | 5000 ms | until a service answers |
| `check_reachable_timeout` | 60000 ms | `CheckReachable` plans three motions |
| `speed_scaling` | 0.3 | (0, 1], when the goal payload has none |

`config/poses.yaml` holds `poses.home` and `poses.view` (7 joint values each).

## Layout

- `src/core/payload.cpp` — no ROS: goal payload → blackboard.
- `src/nodes/` — one node per interface; `outcome_nodes.hpp` holds the action and
  service node bases that write `outcome`; `find_reachable.hpp` the candidate search.
- `src/server.cpp` — the server and the node registration (`register_nodes`).
- `src/main.cpp`
- `trees/manipulation/` — `pick_place.xml`, `pick.xml`, `place.xml`.

## Tests

```bash
colcon test --packages-select fer_behavior_trees
colcon test-result --verbose --test-result-base build/fer_behavior_trees
```

`test_payload` (payload rules), `test_trees` (the three trees against scripted fake nodes,
no ROS), `test_nodes` (each node against fake servers in one process, own DDS domain),
clang-format and the ament linters.

## License

Apache-2.0, see `LICENSE` at the repository root.
