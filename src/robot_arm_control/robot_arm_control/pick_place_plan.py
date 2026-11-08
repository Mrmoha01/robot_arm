"""Pure-Python pick-and-place plan (no ROS), so it can be unit-tested.

The plan is a list of :class:`Step` objects. Each step moves the arm to a
joint target, sets the gripper opening, or both, and says how long to wait.
All arm targets come from inverse kinematics with the gripper pointing down
(the pitch is relaxed automatically if a point cannot be reached that way).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from . import kinematics as kin

GRIPPER_OPEN = 0.025  # metres per finger
GRIPPER_CLOSED = 0.0


@dataclass(frozen=True)
class Step:
    name: str
    arm: Optional[Tuple[float, float, float, float]]  # joint targets, or None
    gripper: Optional[float]  # finger position in metres, or None
    duration: float  # seconds to wait after sending the command


def plan_pick_place(
    object_xy: Sequence[float] = (0.25, -0.10),
    place_xy: Sequence[float] = (0.25, 0.10),
    home_xyz: Sequence[float] = (0.25, 0.0, 0.14),
    approach_z: float = 0.10,
    grasp_z: float = 0.02,
    place_z: float = 0.025,
    move_time: float = 2.5,
    geom: kin.ArmGeometry = kin.DEFAULT_GEOMETRY,
    limits: kin.JointLimits = kin.DEFAULT_LIMITS,
) -> List[Step]:
    """Build the pick-and-place sequence. Raises ``ValueError`` if a point is unreachable."""
    last: List[Optional[Tuple[float, ...]]] = [None]

    def solve(name: str, x: float, y: float, z: float) -> Tuple[float, float, float, float]:
        result = kin.inverse_auto(
            x, y, z, preferred_pitch=math.pi, geom=geom, limits=limits, seed=last[0]
        )
        if result is None:
            raise ValueError(f"Point '{name}' ({x:.3f}, {y:.3f}, {z:.3f}) m is out of reach")
        q, _pitch = result
        last[0] = q
        return q

    ox, oy = object_xy
    px, py = place_xy
    hx, hy, hz = home_xyz

    home = solve("home", hx, hy, hz)
    above_obj = solve("above object", ox, oy, approach_z)
    at_obj = solve("grasp", ox, oy, grasp_z)
    above_place = solve("above place", px, py, approach_z)
    at_place = solve("place", px, py, place_z)

    t = move_time
    return [
        Step("open gripper", None, GRIPPER_OPEN, 1.0),
        Step("go home", home, None, t),
        Step("above object", above_obj, None, t),
        Step("lower to object", at_obj, None, t * 0.8),
        Step("close gripper", None, GRIPPER_CLOSED, 1.5),
        Step("lift object", above_obj, None, t * 0.8),
        Step("above place", above_place, None, t),
        Step("lower to place", at_place, None, t * 0.8),
        Step("open gripper", None, GRIPPER_OPEN, 1.0),
        Step("lift away", above_place, None, t * 0.8),
        Step("return home", home, None, t),
    ]
