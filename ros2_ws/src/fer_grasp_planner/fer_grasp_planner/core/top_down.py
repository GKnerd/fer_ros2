"""
Top-down grasp candidates from an object's bounding box.

All poses are in the box's frame, whose z axis points up (`base`). The hand points down
(hand z = -z), the fingers close along the hand's y axis across one horizontal side of
the box. Poses are poses of `fer_hand_tcp`.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

Vector = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w

DOWN = np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class Box:
    position: Vector
    orientation: Quaternion
    size: Vector


@dataclass(frozen=True)
class Pose:
    position: Vector
    orientation: Quaternion


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
    rotation = _matrix_from_quaternion(box.orientation)
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
            orientation = _quaternion_from_matrix(np.column_stack((hand_x, hand_y, DOWN)))
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


def _matrix_from_quaternion(q: Quaternion) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _quaternion_from_matrix(m: np.ndarray) -> Quaternion:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s,
             s / 4.0)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (s / 4.0, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
             (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, s / 4.0, (m[1, 2] + m[2, 1]) / s,
             (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4.0,
             (m[1, 0] - m[0, 1]) / s)
    return tuple(float(v) for v in q)
