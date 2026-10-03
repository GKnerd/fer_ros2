"""Geometry of the place candidates."""
import math

from fer_grasp_planner.core.geometry import matrix_from_quaternion, Pose
from fer_grasp_planner.core.place import place_candidates, PlaceParams
import numpy as np
import pytest

PARAMS = PlaceParams(place_clearance=0.005, preplace_distance=0.10, retreat_height=0.12)
IDENTITY = (0.0, 0.0, 0.0, 1.0)
HAND_DOWN = (1.0, 0.0, 0.0, 0.0)  # 180 deg about x: an upright box held from above
CUBE = (0.05, 0.05, 0.05)


def about_z(angle: float) -> tuple:
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def about_y(angle: float) -> tuple:
    return (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0))


def held_box(hand: Pose, in_hand: Pose) -> tuple:
    """Return the box center and rotation in base for the hand at `hand`."""
    hand_rotation = matrix_from_quaternion(hand.orientation)
    rotation = hand_rotation @ matrix_from_quaternion(in_hand.orientation)
    center = np.asarray(hand.position) + hand_rotation @ np.asarray(in_hand.position)
    return center, rotation


def bottom(center: np.ndarray, rotation: np.ndarray, size: tuple) -> float:
    return float(center[2] - np.sum(np.abs(rotation[2, :]) * np.asarray(size)) / 2.0)


def test_upright_box_held_from_above():
    in_hand = Pose(position=(0.0, 0.0, 0.0), orientation=HAND_DOWN)
    target = Pose(position=(0.35, 0.30, 0.0), orientation=IDENTITY)
    placements = place_candidates(CUBE, in_hand, target, PARAMS)
    assert len(placements) == 2
    hand_x = []
    for p in placements:
        center, rotation = held_box(p.place, in_hand)
        assert center[:2] == pytest.approx([0.35, 0.30])
        assert bottom(center, rotation, CUBE) == pytest.approx(0.005)
        assert p.place.position == pytest.approx((0.35, 0.30, 0.03))
        hand = matrix_from_quaternion(p.place.orientation)
        assert hand[:, 2] == pytest.approx([0.0, 0.0, -1.0])
        hand_x.append(hand[:, 0])
    assert hand_x[0] == pytest.approx([1.0, 0.0, 0.0])
    assert hand_x[1] == pytest.approx([-1.0, 0.0, 0.0])


def test_box_takes_the_target_orientation():
    in_hand = Pose(position=(0.0, 0.0, 0.0), orientation=HAND_DOWN)
    angle = math.radians(30.0)
    target = Pose(position=(0.35, 0.30, 0.0), orientation=about_z(angle))
    first, second = place_candidates(CUBE, in_hand, target, PARAMS)
    _, rotation = held_box(first.place, in_hand)
    assert rotation == pytest.approx(matrix_from_quaternion(about_z(angle)))
    _, rotation = held_box(second.place, in_hand)
    assert rotation == pytest.approx(matrix_from_quaternion(about_z(angle + math.pi)))


def test_tall_box_held_above_its_center():
    size = (0.04, 0.04, 0.10)
    # Grasped 0.04 below the top: the box center is 0.01 below the hand (hand z down).
    in_hand = Pose(position=(0.0, 0.0, 0.01), orientation=HAND_DOWN)
    target = Pose(position=(0.35, 0.30, 0.0), orientation=IDENTITY)
    for p in place_candidates(size, in_hand, target, PARAMS):
        assert p.place.position == pytest.approx((0.35, 0.30, 0.065))
        center, rotation = held_box(p.place, in_hand)
        assert bottom(center, rotation, size) == pytest.approx(0.005)


def test_tilted_target_uses_the_vertical_extent():
    size = (0.10, 0.04, 0.04)
    in_hand = Pose(position=(0.0, 0.0, 0.0), orientation=IDENTITY)
    target = Pose(position=(0.35, 0.30, 0.02), orientation=about_y(math.pi / 2.0))
    for p in place_candidates(size, in_hand, target, PARAMS):
        assert p.place.position == pytest.approx((0.35, 0.30, 0.075))


def test_preplace_and_retreat_straight_above():
    in_hand = Pose(position=(0.0, 0.0, 0.0), orientation=HAND_DOWN)
    target = Pose(position=(0.35, 0.30, 0.0), orientation=IDENTITY)
    for p in place_candidates(CUBE, in_hand, target, PARAMS):
        x, y, z = p.place.position
        assert p.preplace.position == pytest.approx((x, y, z + 0.10))
        assert p.retreat.position == pytest.approx((x, y, z + 0.12))
        assert p.preplace.orientation == p.place.orientation == p.retreat.orientation
