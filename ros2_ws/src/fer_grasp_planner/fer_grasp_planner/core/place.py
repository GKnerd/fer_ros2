"""
Place candidates for a held box.

The target is in a frame whose z axis points up (`base`): the bottom center of the box at
the place and the box's orientation there. The held box is given by its pose relative to
`fer_hand_tcp`. Poses are poses of `fer_hand_tcp`.
"""
from __future__ import annotations

from dataclasses import dataclass

from fer_grasp_planner.core.geometry import (
    matrix_from_quaternion,
    Pose,
    Quaternion,
    quaternion_from_matrix,
    Vector,
)
import numpy as np

TURN = np.diag([-1.0, -1.0, 1.0])  # 180 deg about the vertical


@dataclass(frozen=True)
class PlaceParams:
    place_clearance: float    # m, box bottom above the target at the place pose
    preplace_distance: float  # m above the place pose
    retreat_height: float     # m above the place pose


@dataclass(frozen=True)
class Placement:
    preplace: Pose
    place: Pose
    retreat: Pose


def place_candidates(
    size: Vector, in_hand: Pose, target: Pose, params: PlaceParams
) -> list[Placement]:
    """
    Return the place candidates for the held box, best first.

    The box in the target orientation, then turned 180 deg about the vertical.
    """
    size = np.asarray(size, dtype=float)
    in_hand_rotation = matrix_from_quaternion(in_hand.orientation)
    in_hand_position = np.asarray(in_hand.position, dtype=float)
    target_rotation = matrix_from_quaternion(target.orientation)
    target_position = np.asarray(target.position, dtype=float)

    placements = []
    for rotation in (target_rotation, TURN @ target_rotation):
        half_height = float(np.sum(np.abs(rotation[2, :]) * size) / 2.0)
        center = target_position + [0.0, 0.0, half_height + params.place_clearance]
        hand_rotation = rotation @ in_hand_rotation.T
        hand_position = center - hand_rotation @ in_hand_position
        orientation = quaternion_from_matrix(hand_rotation)
        placements.append(Placement(
            preplace=_pose(hand_position + [0.0, 0.0, params.preplace_distance], orientation),
            place=_pose(hand_position, orientation),
            retreat=_pose(hand_position + [0.0, 0.0, params.retreat_height], orientation),
        ))
    return placements


def _pose(position: np.ndarray, orientation: Quaternion) -> Pose:
    return Pose(position=tuple(float(v) for v in position), orientation=orientation)
