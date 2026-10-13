"""Conversions between the core types and ROS messages."""
from __future__ import annotations

from builtin_interfaces.msg import Time as TimeMsg
from fer_interfaces.msg import Outcome
from fer_interfaces.msg import WorldObject as WorldObjectMsg
from fer_world_model.core.result import WMResult, WMStatus
from fer_world_model.core.world_object import (
    Detection,
    Point,
    Pose,
    Quaternion,
    Size,
    WorldObject,
)
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Quaternion as QuaternionMsg
from shape_msgs.msg import SolidPrimitive
from vision_msgs.msg import Detection3D

OUTCOME_CODES = {
    WMStatus.OK: Outcome.OK,
    WMStatus.NOT_FOUND: Outcome.NOT_FOUND,
    WMStatus.INVALID: Outcome.INVALID_GOAL,
    WMStatus.CONFLICT: Outcome.INVALID_STATE,
}


def pose_to_msg(pose: Pose) -> PoseMsg:
    return PoseMsg(
        position=PointMsg(x=pose.position.x, y=pose.position.y, z=pose.position.z),
        orientation=QuaternionMsg(
            x=pose.orientation.x, y=pose.orientation.y,
            z=pose.orientation.z, w=pose.orientation.w),
    )


def pose_from_msg(msg: PoseMsg) -> Pose:
    return Pose(
        position=Point(x=msg.position.x, y=msg.position.y, z=msg.position.z),
        orientation=Quaternion(
            x=msg.orientation.x, y=msg.orientation.y,
            z=msg.orientation.z, w=msg.orientation.w),
    )


def seconds_to_msg(seconds: float | None) -> TimeMsg:
    if seconds is None:
        return TimeMsg()
    nanoseconds = int(round(seconds * 1e9))
    return TimeMsg(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def object_to_msg(obj: WorldObject) -> WorldObjectMsg:
    msg = WorldObjectMsg()
    msg.id = obj.object_id
    msg.class_id = obj.class_id
    msg.score = float(obj.score)
    msg.fixed = obj.fixed
    msg.status = int(obj.status)
    msg.held_by = obj.held_by
    msg.pose.header.frame_id = obj.frame
    msg.pose.pose = pose_to_msg(obj.pose)
    msg.shape = SolidPrimitive(
        type=SolidPrimitive.BOX, dimensions=[obj.size.x, obj.size.y, obj.size.z])
    msg.source = obj.source
    msg.last_observed = seconds_to_msg(obj.last_observed)
    return msg


def detection_from_msg(msg: Detection3D, pose_in_base: PoseMsg) -> Detection | None:
    """
    Build a Detection from the best hypothesis of `msg`.

    Returns None if `msg` has no hypothesis or no positive size.

    :param msg: Detection as published by perception.
    :param pose_in_base: `msg.bbox.center`, already transformed into the base frame.
    """
    if not msg.results:
        return None
    best = max(msg.results, key=lambda r: r.hypothesis.score)
    size = msg.bbox.size
    if min(size.x, size.y, size.z) <= 0.0:
        return None
    return Detection(
        class_id=best.hypothesis.class_id,
        score=float(best.hypothesis.score),
        pose=pose_from_msg(pose_in_base),
        size=Size(x=size.x, y=size.y, z=size.z),
    )


def outcome(code: int, message: str = '') -> Outcome:
    return Outcome(code=code, message=message)


def outcome_from_result(result: WMResult) -> Outcome:
    return outcome(OUTCOME_CODES[result.status], result.message)
