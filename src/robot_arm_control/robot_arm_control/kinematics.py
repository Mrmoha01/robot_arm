"""Closed-form kinematics for the 4-joint arm (base yaw + 3 pitch joints).

Pure Python (standard library only), so it can be unit-tested without ROS.

Conventions (these match the URDF in ``robot_arm_description``)
----------------------------------------------------------------
* Frame: ``base_link``. x forward, y left, z up, origin on the ground under
  the base axis.
* ``q1`` rotates the whole arm about +z (base yaw).
* ``q2``, ``q3``, ``q4`` rotate about +y (shoulder, elbow, wrist pitch).
  With all joints at zero the arm points straight up.
* A positive pitch rotation tilts a link from vertical towards +x, so a link
  at absolute pitch ``a`` points along ``(sin a, cos a)`` in the (r, z) plane.
* ``a1 = q2``, ``a2 = q2 + q3`` and the tool pitch ``phi = q2 + q3 + q4``.
  ``phi = pi`` means the gripper points straight down.

Forward kinematics (tool tip, in the arm's vertical plane)::

    r = l_upper*sin(a1) + l_fore*sin(a2) + l_tool*sin(phi)
    z = h0 + l_upper*cos(a1) + l_fore*cos(a2) + l_tool*cos(phi)
    x = r*cos(q1),  y = r*sin(q1)

Inverse kinematics: remove the tool from the target to get the wrist point,
then solve the planar two-link problem with the law of cosines.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Sequence, Tuple

HALF_PI = math.pi / 2.0
Joints = Tuple[float, float, float, float]


@dataclass(frozen=True)
class ArmGeometry:
    """Link dimensions in metres. Keep in sync with the URDF xacro."""

    shoulder_height: float = 0.08  # ground to shoulder axis
    l_upper: float = 0.15  # shoulder axis to elbow axis
    l_fore: float = 0.15  # elbow axis to wrist axis
    l_tool: float = 0.08  # wrist axis to fingertip centre


@dataclass(frozen=True)
class JointLimits:
    """Joint limits in radians (hobby servos: about +-90 degrees)."""

    lower: Joints = (-HALF_PI, -HALF_PI, -HALF_PI, -HALF_PI)
    upper: Joints = (HALF_PI, HALF_PI, HALF_PI, HALF_PI)

    def contains(self, q: Sequence[float], tol: float = 1e-9) -> bool:
        return all(lo - tol <= v <= hi + tol for v, lo, hi in zip(q, self.lower, self.upper))


class Pose(NamedTuple):
    x: float
    y: float
    z: float
    pitch: float  # tool pitch phi = q2 + q3 + q4


DEFAULT_GEOMETRY = ArmGeometry()
DEFAULT_LIMITS = JointLimits()


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(a: float, b: float) -> float:
    """Smallest absolute difference between two angles."""
    return abs(wrap_angle(a - b))


def forward(q: Sequence[float], geom: ArmGeometry = DEFAULT_GEOMETRY) -> Pose:
    """Tool-tip position and tool pitch for joint angles ``q`` (q1..q4)."""
    q1, q2, q3, q4 = q
    a1 = q2
    a2 = q2 + q3
    phi = a2 + q4
    r = geom.l_upper * math.sin(a1) + geom.l_fore * math.sin(a2) + geom.l_tool * math.sin(phi)
    z = (
        geom.shoulder_height
        + geom.l_upper * math.cos(a1)
        + geom.l_fore * math.cos(a2)
        + geom.l_tool * math.cos(phi)
    )
    return Pose(r * math.cos(q1), r * math.sin(q1), z, phi)


def inverse_all(
    x: float,
    y: float,
    z: float,
    pitch: float,
    geom: ArmGeometry = DEFAULT_GEOMETRY,
    limits: JointLimits = DEFAULT_LIMITS,
) -> List[Joints]:
    """Every joint solution (within limits) that puts the tool tip at the target.

    Up to four solutions exist: elbow up / down, and the arm facing the target
    or leaning backwards over the base (base yaw turned by pi, negative reach).
    """
    q1_front = math.atan2(y, x) if (x != 0.0 or y != 0.0) else 0.0
    reach = math.hypot(x, y)

    l1, l2 = geom.l_upper, geom.l_fore
    solutions: List[Joints] = []

    for q1, r in ((q1_front, reach), (wrap_angle(q1_front + math.pi), -reach)):
        if not (limits.lower[0] - 1e-9 <= q1 <= limits.upper[0] + 1e-9):
            continue

        # Wrist point: remove the tool segment from the target.
        wr = r - geom.l_tool * math.sin(pitch)
        wh = z - geom.l_tool * math.cos(pitch) - geom.shoulder_height

        c3 = (wr * wr + wh * wh - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if c3 > 1.0 + 1e-9 or c3 < -1.0 - 1e-9:
            continue
        c3 = max(-1.0, min(1.0, c3))

        for sign in (1.0, -1.0):
            q3 = sign * math.acos(c3)  # q3 > 0: elbow up
            a1 = math.atan2(wr, wh) - math.atan2(l2 * math.sin(q3), l1 + l2 * math.cos(q3))
            a1 = wrap_angle(a1)
            q4 = wrap_angle(pitch - (a1 + q3))
            q = (q1, a1, q3, q4)
            if limits.contains(q) and not any(_close(q, s) for s in solutions):
                solutions.append(q)

    return solutions


def inverse(
    x: float,
    y: float,
    z: float,
    pitch: float,
    geom: ArmGeometry = DEFAULT_GEOMETRY,
    limits: JointLimits = DEFAULT_LIMITS,
    seed: Optional[Sequence[float]] = None,
) -> Optional[Joints]:
    """Best joint solution for a tool pose, or ``None`` if unreachable.

    With a ``seed`` (e.g. the current joint angles) the solution closest to it
    is chosen, which keeps motion continuous. Without one, elbow-up and the
    smallest base yaw are preferred.
    """
    solutions = inverse_all(x, y, z, pitch, geom, limits)
    if not solutions:
        return None
    if seed is not None:
        return min(solutions, key=lambda s: sum((a - b) ** 2 for a, b in zip(s, seed)))
    return min(solutions, key=lambda s: (s[2] < 0.0, abs(s[0])))


def inverse_auto(
    x: float,
    y: float,
    z: float,
    preferred_pitch: float = math.pi,
    max_deviation: float = math.radians(90.0),
    step: float = math.radians(2.0),
    geom: ArmGeometry = DEFAULT_GEOMETRY,
    limits: JointLimits = DEFAULT_LIMITS,
    seed: Optional[Sequence[float]] = None,
) -> Optional[Tuple[Joints, float]]:
    """Like :func:`inverse`, but relax the tool pitch until a solution exists.

    Tries ``preferred_pitch`` first, then pitches further and further away (up
    to ``max_deviation``). Returns ``(joints, pitch_used)`` or ``None``.
    """
    steps = int(max_deviation / step)
    for k in range(steps + 1):
        offsets = (0.0,) if k == 0 else (k * step, -k * step)
        for off in offsets:
            pitch = preferred_pitch + off
            q = inverse(x, y, z, pitch, geom, limits, seed)
            if q is not None:
                return q, pitch
    return None


def max_reach(geom: ArmGeometry = DEFAULT_GEOMETRY) -> float:
    """Longest straight-line reach of the tool tip from the shoulder axis."""
    return geom.l_upper + geom.l_fore + geom.l_tool


def _close(a: Sequence[float], b: Sequence[float], tol: float = 1e-9) -> bool:
    return all(abs(u - v) < tol for u, v in zip(a, b))
