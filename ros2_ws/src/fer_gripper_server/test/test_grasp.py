from fer_gripper_server.core.grasp import (
    force_valid,
    grasp_succeeded,
    release_confirmed,
    still_holding,
    width_valid,
)


def test_width_and_force_limits():
    assert width_valid(0.0, 0.08) and width_valid(0.08, 0.08)
    assert not width_valid(-0.001, 0.08) and not width_valid(0.081, 0.08)
    assert force_valid(70.0, 70.0) and not force_valid(0.0, 70.0)
    assert not force_valid(70.1, 70.0)


def test_grasp_needs_the_expected_width():
    assert grasp_succeeded(0.052, 0.05, 0.005, 0.002)
    assert not grasp_succeeded(0.056, 0.05, 0.005, 0.002)
    assert not grasp_succeeded(0.030, 0.05, 0.005, 0.002)


def test_grasp_on_nothing_fails_even_for_tiny_objects():
    assert not grasp_succeeded(0.001, 0.0, 0.005, 0.002)


def test_release_and_hold_checks():
    assert release_confirmed(0.078, 0.08, 0.003)
    assert not release_confirmed(0.070, 0.08, 0.003)
    assert still_holding(0.051, 0.05, 0.005)
    assert not still_holding(0.0, 0.05, 0.005)
    assert not still_holding(0.08, 0.05, 0.005)
