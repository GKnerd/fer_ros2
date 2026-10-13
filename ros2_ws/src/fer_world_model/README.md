# fer_world_model

World model of the FER platform. Builds and keeps a representation of the objects in the robot's workspace
from detections and from gripper reports, and serves them through `fer_interfaces`.

Objects are assumed to be received from some kind of perception node. That is why objects are boxes: every object is its bounding box (`shape` is always a `BOX`),
centered on and oriented with its pose.

## Interfaces

Served:

| Interface | Kind | Name |
|---|---|---|
| `DetectObjects` | action | `/world_model/detect_objects` |
| `RefineObject` | action | `/world_model/refine_object` |
| `QueryObjects` | service | `/world_model/query_objects` |
| `SetObjectStatus` | service | `/world_model/set_object_status` |
| `WorldObjectArray` | topic, latched | `/world_model/objects` |

Used:

- `/perception/detections` — `vision_msgs/Detection3DArray`, stamped at capture, any
  camera frame (best effort).
- TF — detection frame → `base` at the detection stamp; request poses → `base`.

## Behaviour

- **Snapshot:** `DetectObjects` and `RefineObject` use the first detection message
  stamped after the goal was accepted. No usable message within `detection_timeout` →
  `TIMEOUT`. Take the snapshot with the arm at rest.
- **Association:** a detection matches a FREE object of the same class within
  `association_gate`, nearest pairs first. An unmatched detection takes over a LOST
  object of the same class; otherwise it becomes a new object `<class>_<n>`. Ids are
  never reused.
- **Held objects:** detections at a GRASPED object's position (the object seen in the
  hand) are dropped. GRASPED and fixed objects are never changed by detection.
- **Not seen and removal:** an object that is not seen is listed in `not_seen` and not
  changed. A 1 Hz sweep removes FREE and LOST objects missing for longer than
  `removal_timeout`; the clock starts at the first snapshot that missed the object (for
  LOST: when it became LOST), so nothing is removed while no detection runs.
- **One detection goal at a time:** a new `DetectObjects` or `RefineObject` goal ends the
  running one with `CANCELLED`.
- **Revision:** `/world_model/objects` and every answer carry `revision`, +1 on every
  change.
- **State lives in memory:** restarting the node clears all objects and restarts ids.

## Launch

```bash
ros2 launch fer_world_model world_model.launch.py perception:=mock
```

| Argument | Default | Meaning |
|---|---|---|
| `perception` | `none` | `mock` also starts `mock_perception` |
| `mock_objects_file` | `config/mock_objects.yaml` | detections published by the mock |
| `fixtures_file` | `config/fixtures.yaml` | fixed objects; `''` for none |
| `params_file` | `config/world_model.yaml` | node parameters |
| `use_sim_time` | `true` | simulated clock |
| `log_level` | `info` | |

## Parameters (`config/world_model.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `detection_topic` | `/perception/detections` | detection input |
| `detection_source` | `perception` | recorded as `source` of detected objects (`mock_perception` with the mock) |
| `base_frame` | `base` | frame of every FREE and LOST pose |
| `association_gate` | 0.05 m | largest center distance for the same object |
| `refine_gate` | 0.10 m | search radius around the object for `RefineObject` |
| `min_score` | 0.5 | detections below are ignored |
| `detection_timeout` | 5.0 s | wait for a snapshot |
| `tf_timeout` | 0.1 s | |
| `removal_timeout` | 30.0 s | missing objects are removed after this |
| `fixtures_file` | `''` | set by the launch file |

`config/fixtures.yaml` holds the table the FER stands on (top at `base` z = 0).
Fixtures are never matched, changed or removed.

## Mock perception

`mock_perception` publishes `config/mock_objects.yaml` as `Detection3DArray` at 5 Hz:
class, score, center pose and size, no ids. Parameters: `objects_file` (required),
`frame_id` (`base`), `rate` (5.0 Hz), `detection_topic`.

## Layout

- `core/` — no ROS: `WorldObject`, `WorldModel`, association, config parsing, results.
- `adapters/ros_conversions.py` — core types ⇄ messages.
- `world_model_server.py` — the node; the model is built by `build_world_model()` and
  passed in.
- `mock_perception_node.py`

## Tests

```bash
colcon test --packages-select fer_world_model
colcon test-result --verbose --test-result-base build/fer_world_model
```

Unit tests on `core/`, a contract test (the node with fake detections and TF in one
process), flake8 and pep257. `test/conftest.py` gives each run its own DDS domain on
localhost.

## License

Apache-2.0, see `LICENSE`.
