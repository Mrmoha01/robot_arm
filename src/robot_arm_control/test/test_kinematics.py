"""Unit tests for the kinematics and the pick-and-place plan (no ROS needed)."""

import math
import random

import pytest

from robot_arm_control import kinematics as kin
from robot_arm_control.pick_place_plan import plan_pick_place

GEOM = kin.DEFAULT_GEOMETRY
LIMITS = kin.DEFAULT_LIMITS


def test_zero_pose_points_straight_up():
    pose = kin.forward((0.0, 0.0, 0.0, 0.0))
    assert pose.x == pytest.approx(0.0, abs=1e-12)
    assert pose.y == pytest.approx(0.0, abs=1e-12)
    expected_z = GEOM.shoulder_height + GEOM.l_upper + GEOM.l_fore + GEOM.l_tool
    assert pose.z == pytest.approx(expected_z)


def test_shoulder_forward_90_degrees_points_along_x():
    pose = kin.forward((0.0, math.pi / 2, 0.0, 0.0))
    assert pose.x == pytest.approx(kin.max_reach())
    assert pose.y == pytest.approx(0.0, abs=1e-12)
    assert pose.z == pytest.approx(GEOM.shoulder_height)


def test_base_yaw_rotates_about_z():
    pose = kin.forward((math.pi / 2, math.pi / 2, 0.0, 0.0))
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(kin.max_reach())


def test_inverse_round_trip_random_poses():
    rng = random.Random(0)
    checked = 0
    for _ in range(2000):
        q = tuple(rng.uniform(lo, hi) for lo, hi in zip(LIMITS.lower, LIMITS.upper))
        pose = kin.forward(q)
        solutions = kin.inverse_all(*pose, GEOM, LIMITS)
        assert solutions, f"no IK solution for reachable pose from q={q}"
        for sol in solutions:
            back = kin.forward(sol)
            assert back.x == pytest.approx(pose.x, abs=1e-9)
            assert back.y == pytest.approx(pose.y, abs=1e-9)
            assert back.z == pytest.approx(pose.z, abs=1e-9)
            assert kin.angle_difference(back.pitch, pose.pitch) < 1e-9
            assert LIMITS.contains(sol)
        checked += 1
    assert checked == 2000


def test_inverse_finds_original_joints_among_solutions():
    q = (0.3, 0.7, 0.9, -0.4)
    pose = kin.forward(q)
    solutions = kin.inverse_all(*pose, GEOM, LIMITS)
    assert any(all(abs(a - b) < 1e-9 for a, b in zip(s, q)) for s in solutions)


def test_unreachable_point_returns_none():
    assert kin.inverse(1.0, 0.0, 0.1, math.pi) is None
    assert kin.inverse_auto(1.0, 0.0, 0.1) is None


def test_seed_selects_closest_solution():
    q = (0.0, 0.7, 0.9, -0.4)
    pose = kin.forward(q)
    seed_down = (0.0, 0.2, -0.9, 0.5)
    sol_up = kin.inverse(*pose, seed=q)
    sol_other = kin.inverse(*pose, seed=seed_down)
    assert sol_up is not None and sol_other is not None
    assert all(abs(a - b) < 1e-9 for a, b in zip(sol_up, q))


def test_auto_pitch_reaches_close_point_that_vertical_cannot():
    # Close to the base the gripper cannot point straight down with +-90 degree joints,
    # but tilting the tool a little makes the point reachable.
    assert kin.inverse(0.20, 0.0, 0.02, math.pi) is None
    result = kin.inverse_auto(0.20, 0.0, 0.02)
    assert result is not None
    q, pitch = result
    back = kin.forward(q)
    assert back.x == pytest.approx(0.20, abs=1e-9)
    assert back.z == pytest.approx(0.02, abs=1e-9)
    assert abs(pitch - math.pi) > 0.0


def test_point_too_close_to_base_is_unreachable():
    assert kin.inverse_auto(0.15, 0.0, 0.02) is None


def test_pick_place_plan_is_reachable_and_within_limits():
    steps = plan_pick_place()
    assert len(steps) == 11
    for step in steps:
        if step.arm is not None:
            assert LIMITS.contains(step.arm), step.name
    grasp = next(s for s in steps if s.name == "lower to object")
    pose = kin.forward(grasp.arm)
    assert pose.x == pytest.approx(0.25, abs=1e-6)
    assert pose.y == pytest.approx(-0.10, abs=1e-6)
    assert pose.z == pytest.approx(0.02, abs=1e-6)


def test_pick_place_plan_rejects_unreachable_points():
    with pytest.raises(ValueError):
        plan_pick_place(object_xy=(0.9, 0.0))
