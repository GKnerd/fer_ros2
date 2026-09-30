"""Geometry of the top-down candidates."""
import math

from fer_grasp_planner.core.top_down import Box, Pose, top_down_candidates, TopDownParams
import numpy as np
import pytest

PARAMS = TopDownParams(
    max_object_width=0.07,
    pregrasp_distance=0.10,
    lift_height=0.10,
    max_grasp_depth=0.04,
    min_tip_clearance=0.012,
)
IDENTITY = (0.0, 0.0, 0.0, 1.0)


def rotate(q: tuple, v) -> np.ndarray:
    """Rotate `v` by the quaternion `q` (x, y, z, w)."""
    u = np.array(q[:3])
    v = np.array(v, dtype=float)
    return v + 2.0 * q[3] * np.cross(u, v) + 2.0 * np.cross(u, np.cross(u, v))


def hand_axes(pose: Pose) -> tuple:
    return tuple(rotate(pose.orientation, axis) for axis in np.eye(3))


def about_z(angle: float) -> tuple:
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def about_y(angle: float) -> tuple:
    return (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0))


def test_upright_box():
    box = Box(position=(0.4, 0.1, 0.025), orientation=IDENTITY, size=(0.05, 0.05, 0.05))
    candidates = top_down_candidates(box, PARAMS)
    assert len(candidates) == 4
    closing = []
    for c in candidates:
        _, y, z = hand_axes(c.grasp)
        assert z == pytest.approx([0.0, 0.0, -1.0])
        closing.append(tuple(np.round(y, 6)))
        assert c.grasp.position == pytest.approx((0.4, 0.1, 0.025))
        assert c.pregrasp.position == pytest.approx((0.4, 0.1, 0.125))
        assert c.lift.position == pytest.approx((0.4, 0.1, 0.125))
        assert c.pregrasp.orientation == c.grasp.orientation == c.lift.orientation
        assert c.width == pytest.approx(0.05)
    assert sorted(closing) == sorted([(1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
                                      (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)])


def test_hand_follows_the_box_yaw():
    angle = math.radians(30.0)
    box = Box(position=(0.4, 0.0, 0.025), orientation=about_z(angle), size=(0.05, 0.05, 0.05))
    c, s = math.cos(angle), math.sin(angle)
    expected = sorted([(c, s), (-c, -s), (-s, c), (s, -c)])
    closing = sorted(tuple(np.round(hand_axes(k.grasp)[1][:2], 6))
                     for k in top_down_candidates(box, PARAMS))
    assert np.allclose(closing, expected, atol=1e-6)


def test_sides_wider_than_the_gripper_are_skipped():
    box = Box(position=(0.4, 0.0, 0.025), orientation=IDENTITY, size=(0.05, 0.09, 0.05))
    candidates = top_down_candidates(box, PARAMS)
    assert len(candidates) == 2
    for c in candidates:
        assert c.width == pytest.approx(0.05)
        assert abs(hand_axes(c.grasp)[1][0]) == pytest.approx(1.0)

    too_wide = Box(position=(0.4, 0.0, 0.025), orientation=IDENTITY, size=(0.09, 0.09, 0.05))
    assert top_down_candidates(too_wide, PARAMS) == []


def test_tall_object_is_grasped_below_the_top():
    box = Box(position=(0.5, 0.0, 0.05), orientation=IDENTITY, size=(0.04, 0.04, 0.10))
    for c in top_down_candidates(box, PARAMS):
        assert c.grasp.position[2] == pytest.approx(0.06)


def test_flat_object_keeps_the_tips_above_its_bottom():
    box = Box(position=(0.5, 0.0, 0.005), orientation=IDENTITY, size=(0.05, 0.05, 0.01))
    for c in top_down_candidates(box, PARAMS):
        assert c.grasp.position[2] == pytest.approx(0.012)


def test_lying_cylinder_uses_the_most_vertical_box_axis():
    # Box z (the cylinder's long axis) lies along base x; box x points down.
    box = Box(position=(0.5, 0.0, 0.02), orientation=about_y(math.pi / 2.0),
              size=(0.04, 0.04, 0.10))
    candidates = top_down_candidates(box, PARAMS)
    assert len(candidates) == 2
    for c in candidates:
        assert c.width == pytest.approx(0.04)
        assert abs(hand_axes(c.grasp)[1][1]) == pytest.approx(1.0)
        assert c.grasp.position[2] == pytest.approx(0.02)


def test_narrower_side_first():
    box = Box(position=(0.4, 0.0, 0.025), orientation=IDENTITY, size=(0.06, 0.03, 0.05))
    widths = [c.width for c in top_down_candidates(box, PARAMS)]
    assert widths == pytest.approx([0.03, 0.03, 0.06, 0.06])
