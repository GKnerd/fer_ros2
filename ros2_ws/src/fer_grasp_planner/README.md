# fer_grasp_planner

Grasp planner of the FER platform. Serves `GetGraspCandidates` of
[`fer_interfaces`](https://github.com/GKnerd/fer_interfaces) on `/grasp/candidates`:
top-down grasp candidates computed from the object's bounding box in the world model.

A grasp network later replaces the computation behind the same service.

## Interface

| Name | Kind | Type |
|---|---|---|
| `/grasp/candidates` | service (served) | `fer_interfaces/srv/GetGraspCandidates` |
| `/world_model/query_objects` | service (called) | `fer_interfaces/srv/QueryObjects` |
| `/grasp/debug/candidates` | topic, latched (published) | `geometry_msgs/PoseArray` — debug only, not part of the contract |

| Case | Outcome |
|---|---|
| unknown id | `NOT_FOUND` |
| object not FREE, or fixed (the table) | `INVALID_STATE` |
| world model not answering within `world_model_timeout` | `TIMEOUT` |
| no side of the object fits the gripper | `OK`, empty list |
| otherwise | `OK`, candidates best first |

`NO_DATA` is never returned by this version.

## Candidates

Poses are poses of `fer_hand_tcp`, in the object's frame (`base` for FREE objects).

- **Up axis:** the box axis closest to vertical; the other two are the sides the fingers
  can close across.
- **Per side** no wider than `max_object_width`: two candidates, the hand turned 180°
  apart (joint 7 may reach only one of them). 0, 2 or 4 candidates per object.
- **Orientation:** hand z points down, hand y (the finger direction) across the side.
- **Grasp height:** the box center, raised to `max_grasp_depth` below the top for tall
  objects (keeps the palm off the object) and at least `min_tip_clearance` above the
  bottom (the fingertips reach 9.5 mm below `fer_hand_tcp`).
- **Pre-grasp** `pregrasp_distance` and **lift** `lift_height` straight above the grasp,
  same orientation.
- **width** = the side's size, goes straight into `Grasp.width`; **force** from the
  object catalog, goes straight into `Grasp.force`.
- **Order:** narrower side first (`score = 1 − width / max_object_width`).
  `max_candidates` > 0 cuts the list.

Whether the arm can execute a candidate is not checked here; the behavior tree does that
with `CheckReachable`.

## Configuration

`config/grasp_planner.yaml`:

| Parameter | Default | |
|---|---|---|
| `max_object_width` | 0.07 m | the 0.08 m opening minus 1 cm to pass the fingers over the object |
| `pregrasp_distance` | 0.10 m | above the grasp |
| `lift_height` | 0.10 m | above the grasp |
| `max_grasp_depth` | 0.04 m | TCP at most this far below the object top |
| `min_tip_clearance` | 0.012 m | TCP at least this far above the object bottom |
| `world_model_timeout` | 2.0 s | |

`config/object_catalog.yaml` — grasp force per detected class; a class not listed uses
`default`. One catalog serves real and MuJoCo, so forces stay within MuJoCo's 20 N.

```yaml
default: {force: 15.0}
classes:
  box: {force: 15.0}
```

The node exits at startup when the catalog has no `default` or a force is not positive.

## Run

```bash
ros2 launch fer_world_model world_model.launch.py perception:=mock
ros2 launch fer_grasp_planner grasp_planner.launch.py
ros2 action send_goal /world_model/detect_objects fer_interfaces/action/DetectObjects "{}"
ros2 service call /grasp/candidates fer_interfaces/srv/GetGraspCandidates "{object_id: box_1}"
```

Launch arguments: `use_sim_time` (default `true`), `log_level`, `params_file`,
`catalog_file`.

In RViz, the `GraspCandidates` display of `fer_ros2_bringup` (PoseArray, shape Axes)
shows each candidate of the latest answer as a frame.

## Layout

- `core/` — geometry and catalog, no ROS imports.
- `adapters/` — world-model client, message conversions.
- `grasp_planner_server.py` — the node; client and server are built in `main`.

## Tests

```bash
colcon test --packages-select fer_grasp_planner
```

Geometry and catalog (pytest), contract test (the node with a fake world model in one
process), flake8, pep257.
