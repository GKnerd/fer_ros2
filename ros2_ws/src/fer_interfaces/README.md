# fer_interfaces

Messages, services and actions of the FER manipulation platform. The contract between
the behavior trees and the servers that implement motion, gripper, world model and
grasp candidates. Depends only on `action_msgs`, `builtin_interfaces`,
`geometry_msgs`, `shape_msgs` and `std_msgs`.

This package is a public contract: renaming a field, changing its type or meaning, or
renumbering a constant is a breaking change.

## Interfaces

| Interface | Kind | Name | Served by | Called by |
|---|---|---|---|---|
| `MoveToPose` | action | `/motion/move_to_pose` | motion backend | BT |
| `MoveToJoints` | action | `/motion/move_to_joints` | motion backend | BT |
| `CheckReachable` | service | `/motion/check_reachable` | motion backend | BT |
| `MoveGripper` | action | `/gripper/move` | `fer_gripper_server` | BT |
| `Grasp` | action | `/gripper/grasp` | `fer_gripper_server` | BT |
| `Release` | action | `/gripper/release` | `fer_gripper_server` | BT |
| `DetectObjects` | action | `/world_model/detect_objects` | `fer_world_model` | BT |
| `RefineObject` | action | `/world_model/refine_object` | `fer_world_model` | BT |
| `QueryObjects` | service | `/world_model/query_objects` | `fer_world_model` | BT, motion backend, gripper server, grasp planner |
| `SetObjectStatus` | service | `/world_model/set_object_status` | `fer_world_model` | `fer_gripper_server` |
| `GetGraspCandidates` | service | `/grasp/candidates` | `fer_grasp_planner` | BT |
| `WorldObjectArray` | topic, latched | `/world_model/objects` | `fer_world_model` | RViz, monitoring |

Exactly one motion backend runs at a time; launch selects it.

## Rules

- **Units:** SI — m, rad, N, s.
- **Gripper width:** the full opening between the fingers, never the travel of one
  finger.
- **Frames:** poses are `PoseStamped` in any frame known to TF. A server converts a pose
  to `base` (the TF root) once, when it accepts the goal; a pose in `fer_hand_tcp` is relative to
  the hand at that moment.
- **Object pose:** the center of its `shape`, as defined by `shape_msgs/SolidPrimitive`.
  A GRASPED object's pose is in the frame of `held_by`.
- **Joint configuration:** `float64[7]`, joint 1 to 7, in rad.
- **Results:** every result starts with `Outcome`. Callers branch on `outcome.code`;
  `outcome.message` is for logs only.
- **Replacement:** a new goal on a server replaces the running one; the old goal ends
  `CANCELLED`.
- **World model writers:** only detection (`DetectObjects`, `RefineObject`) and
  `fer_gripper_server` (`SetObjectStatus`) change the world model. Everything else
  reads.

## Validation

Servers reject what the message types cannot exclude:

| Rule | Checked by | Outcome |
|---|---|---|
| `speed_scaling` in (0, 1] | motion backend | `INVALID_GOAL` |
| gripper width in [0, 0.08] m | gripper server | `INVALID_GOAL` |
| pose frame unknown to TF | motion backend | `INVALID_GOAL` |
| unknown object id | motion backend, gripper server, world model | `NOT_FOUND` |
| `Grasp` on a non-FREE object, `Release` on a non-GRASPED object, `RefineObject` on a GRASPED object | gripper server, world model | `INVALID_STATE` |

Every implementation of an interface passes the contract test of its server package.

## Behavior

- **Grasp:** marks the object GRASPED only when the measured width is within
  `tolerance` of `width`. On failure the gripper reopens and the world model is
  unchanged.
- **Release:** opens and confirms the width first, then marks the object FREE at its
  estimated pose (`source: release_estimate`).
- **Detection:** an object that is not seen is never removed or changed; it is listed
  in `not_seen`. GRASPED objects are never changed by detection.
- **CheckReachable:** plans the chained targets without moving; the first starts at the
  current state. Returns the joint configuration at each target.

## Use

```xml
<depend>fer_interfaces</depend>
```

```cmake
find_package(fer_interfaces REQUIRED)
```

```python
from fer_interfaces.action import MoveToPose
from fer_interfaces.msg import Outcome
```

## License

Apache-2.0, see `LICENSE`.
