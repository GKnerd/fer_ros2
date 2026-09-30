"""Conversions between the core types and ROS messages."""
from __future__ import annotations

from builtin_interfaces.msg import Time
from fer_grasp_planner.core.top_down import Box, Candidate, Pose
from fer_interfaces.msg import GraspCandidate, WorldObject
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


def pose_to_msg(pose: Pose) -> PoseMsg:
    x, y, z = pose.position
    qx, qy, qz, qw = pose.orientation
    return PoseMsg(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))


def candidate_to_msg(
    candidate: Candidate, frame_id: str, stamp: Time, force: float
) -> GraspCandidate:
    def stamped(pose: Pose) -> PoseStamped:
        msg = PoseStamped(pose=pose_to_msg(pose))
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        return msg

    return GraspCandidate(
        pregrasp_pose=stamped(candidate.pregrasp),
        grasp_pose=stamped(candidate.grasp),
        lift_pose=stamped(candidate.lift),
        width=candidate.width,
        force=force,
        score=candidate.score,
    )
