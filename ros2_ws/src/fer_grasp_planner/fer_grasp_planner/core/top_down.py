"""
Top-down grasp candidates from an object's bounding box.

All poses are in the box's frame, whose z axis points up (`base`). The hand points down
(hand z = -z), the fingers close along the hand's y axis across one horizontal side of
the box. Poses are poses of `fer_hand_tcp`.
"""
from __future__ import annotations

from dataclasses import dataclass

from fer_grasp_planner.core.geometry import (
    Box,
    matrix_from_quaternion,
    Pose,
    Quaternion,
    quaternion_from_matrix,
)
import numpy as np

DOWN = np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class TopDownParams:
    max_object_width: float   # m, wider sides are not grasped
    pregrasp_distance: float  # m above the grasp
    lift_height: float        # m above the grasp
    max_grasp_depth: float    # m, TCP at most this far below the box top
    min_tip_clearance: float  # m, TCP at least this far above the box bottom


@dataclass(frozen=True)
class Candidate:
    pregrasp: Pose
    grasp: Pose
    lift: Pose
    width: float
    score: float


def top_down_candidates(box: Box, params: TopDownParams) -> list[Candidate]:
    """
    Return the top-down candidates for `box`, best first.

    Per horizontal side that fits the gripper, two candidates with hand yaws 180 deg
    apart: 0, 2 or 4 in total.
    """
    rotation = matrix_from_quaternion(box.orientation)
    size = np.asarray(box.size, dtype=float)
    center = np.asarray(box.position, dtype=float)

    up = int(np.argmax(np.abs(rotation[2, :])))
    half_height = float(np.sum(np.abs(rotation[2, :]) * size) / 2.0)
    top = center[2] + half_height
    bottom = center[2] - half_height
    grasp_z = max(center[2], top - params.max_grasp_depth)
    grasp_z = max(grasp_z, bottom + params.min_tip_clearance)
    grasp_position = np.array([center[0], center[1], grasp_z])

    candidates = []
    for axis in (i for i in range(3) if i != up):
        width = float(size[axis])
        if width > params.max_object_width:
            continue
        closing = rotation[:, axis].copy()
        closing[2] = 0.0
        closing /= np.linalg.norm(closing)
        for sign in (1.0, -1.0):
            hand_y = sign * closing
            hand_x = np.cross(hand_y, DOWN)
            orientation = quaternion_from_matrix(np.column_stack((hand_x, hand_y, DOWN)))
            candidates.append(Candidate(
                pregrasp=_pose(grasp_position + [0.0, 0.0, params.pregrasp_distance],
                               orientation),
                grasp=_pose(grasp_position, orientation),
                lift=_pose(grasp_position + [0.0, 0.0, params.lift_height], orientation),
                width=width,
                score=1.0 - width / params.max_object_width,
            ))
    return sorted(candidates, key=lambda c: c.score, reverse=True)


def _pose(position: np.ndarray, orientation: Quaternion) -> Pose:
    return Pose(position=tuple(float(v) for v in position), orientation=orientation)
