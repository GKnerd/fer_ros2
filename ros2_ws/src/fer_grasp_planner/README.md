# fer_grasp_planner

Grasp planner of the FER platform. Serves two services of
[`fer_interfaces`](https://github.com/GKnerd/fer_interfaces):
- `GetGraspCandidates` on `/grasp/candidates`: top-down grasp candidates computed from
  the object's bounding box in the world model;
- `GetPlaceCandidates` on `/place/candidates`: hand poses that put the held object down
  at a target.

A grasp network later replaces the computation behind the same service.

## Interface

| Name | Kind | Type |
|---|---|---|
| `/grasp/candidates` | service (served) | `fer_interfaces/srv/GetGraspCandidates` |
| `/place/candidates` | service (served) | `fer_interfaces/srv/GetPlaceCandidates` |
| `/world_model/query_objects` | service (called) | `fer_interfaces/srv/QueryObjects` |
| `/tf`, `/tf_static` | topics (subscribed) | place target to `base` |
| `/grasp/debug/candidates` | topic, latched (published) | `geometry_msgs/PoseArray` — debug only, not part of the contract |

`/grasp/candidates`:

| Case | Outcome |
|---|---|
| unknown id | `NOT_FOUND` |
| object not FREE, or fixed (the table) | `INVALID_STATE` |
| world model not answering within `world_model_timeout` | `TIMEOUT` |
| no side of the object fits the gripper | `OK`, empty list |
| otherwise | `OK`, candidates best first |

`NO_DATA` is never returned by this version.

`/place/candidates`:

| Case | Outcome |
|---|---|
| target frame unknown to TF within `tf_timeout`, or orientation not a rotation | `INVALID_GOAL` |
| world model not answering within `world_model_timeout` | `TIMEOUT` |
| unknown id | `NOT_FOUND` |
| object not GRASPED, or held by a frame other than `fer_hand_tcp` | `INVALID_STATE` |
| otherwise | `OK`, 2 candidates |

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

## Place candidates

The target is the bottom center of the object's box at the place and the object's
orientation there, in any TF frame; it is converted to `base` once, with the latest
transform. Poses are poses of `fer_hand_tcp`, in `base`.

- **Held object:** its pose relative to `fer_hand_tcp` from the world model (written by
  `fer_gripper_server` at the grasp).
- **Place:** the box in the target orientation, its bottom `place_clearance` above the
  target; the hand pose follows from the object's pose in the hand. The gap keeps the
  attached box off the surface while MoveIt plans, since boxes that touch count as a
  collision.
- **Pre-place** `preplace_distance` and **retreat** `retreat_height` straight above the
  place pose, same orientation.
- **Order:** the target orientation, then the object turned 180° about the vertical
  (joint 7 may reach only one of them).

Known limit: the target orientation is taken as the object's full orientation. An object
held upright (a top-down grasp) with a yaw-only target keeps the hand pointing down; an
object held with another box axis vertical is stood up, and the hand tilts to do so.

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
| `place_clearance` | 0.005 m | box bottom above the place target when released |
| `preplace_distance` | 0.10 m | above the place pose |
| `retreat_height` | 0.10 m | above the place pose |
| `tf_timeout` | 0.2 s | place target transform to `base` |

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

Place candidates need a GRASPED object (after a `Grasp`, or set by hand with
`SetObjectStatus`):

```bash
ros2 service call /place/candidates fer_interfaces/srv/GetPlaceCandidates \
  "{object_id: box_1, target: {header: {frame_id: base}, pose: {position: {x: 0.35, y: 0.30}, orientation: {w: 1.0}}}}"
```

Launch arguments: `use_sim_time` (default `true`), `log_level`, `params_file`,
`catalog_file`.

In RViz, the `GraspCandidates` display of `fer_ros2_bringup` (PoseArray, shape Axes)
shows each candidate of the latest answer as a frame.

## Layout

- `core/` — geometry (`top_down.py`, `place.py`, shared `geometry.py`) and catalog, no
  ROS imports.
- `adapters/` — world-model client, message conversions.
- `grasp_planner_server.py` — the node and the grasp server; `place_server.py` — the place
  server. Client, TF buffer and both servers are built in `main`.

## Tests

```bash
colcon test --packages-select fer_grasp_planner
```

Geometry of grasps and places, catalog (pytest), contract test (both servers with a fake
world model and a static transform in one process), flake8, pep257.
