"""Conversions between the core types and ROS messages."""
from __future__ import annotations

from builtin_interfaces.msg import Time
from fer_grasp_planner.core.geometry import Box, Pose
from fer_grasp_planner.core.place import Placement
from fer_grasp_planner.core.top_down import Candidate
from fer_interfaces.msg import GraspCandidate, PlaceCandidate, WorldObject
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geometry_msgs.msg import Pose as PoseMsg


def box_from_msg(obj: WorldObject) -> Box:
    p = obj.pose.pose.position
    q = obj.pose.pose.orientation
    return Box(
        position=(p.x, p.y, p.z),
        orientation=(q.x, q.y, q.z, q.w),
        size=tuple(float(d) for d in obj.shape.dimensions[:3]),
    )


def pose_from_msg(msg: PoseMsg) -> Pose:
    p = msg.position
    q = msg.orientation
    return Pose(position=(p.x, p.y, p.z), orientation=(q.x, q.y, q.z, q.w))


def pose_to_msg(pose: Pose) -> PoseMsg:
    x, y, z = pose.position
    qx, qy, qz, qw = pose.orientation
    return PoseMsg(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))


def _stamped(pose: Pose, frame_id: str, stamp: Time) -> PoseStamped:
    msg = PoseStamped(pose=pose_to_msg(pose))
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    return msg


def candidate_to_msg(
    candidate: Candidate, frame_id: str, stamp: Time, force: float
) -> GraspCandidate:
    return GraspCandidate(
        pregrasp_pose=_stamped(candidate.pregrasp, frame_id, stamp),
        grasp_pose=_stamped(candidate.grasp, frame_id, stamp),
        lift_pose=_stamped(candidate.lift, frame_id, stamp),
        width=candidate.width,
        force=force,
        score=candidate.score,
    )


def placement_to_msg(placement: Placement, frame_id: str, stamp: Time) -> PlaceCandidate:
    return PlaceCandidate(
        preplace_pose=_stamped(placement.preplace, frame_id, stamp),
        place_pose=_stamped(placement.place, frame_id, stamp),
        retreat_pose=_stamped(placement.retreat, frame_id, stamp),
    )
