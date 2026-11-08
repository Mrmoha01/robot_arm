#!/usr/bin/env python3
"""Check that the URDF agrees with the kinematics code, and estimate servo torque.

Usage (from the repository root, after `source install/setup.bash`)::

    xacro src/robot_arm_description/urdf/robot_arm.urdf.xacro > /tmp/robot_arm.urdf
    python3 tools/urdf_check.py /tmp/robot_arm.urdf

What it does
------------
1. Computes forward kinematics straight from the URDF joint origins and axes
   and compares the tool tip against ``robot_arm_control.kinematics.forward``
   for many random joint angles. A mismatch means the xacro dimensions and
   ``ArmGeometry`` have drifted apart.
2. Computes the static gravity torque on every joint for the arm stretched out
   horizontally (the worst case), with and without a payload, and compares it
   with the effort limits in the URDF. Servo stall torque is the real limit,
   so treat the margins as a design check, not a guarantee.

Requires numpy.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "robot_arm_control"))
from robot_arm_control import kinematics as kin  # noqa: E402

G = 9.81
ARM_JOINTS = ["base_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    v = 1 - c
    return np.array(
        [
            [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
        ]
    )


def homogeneous(rot, pos):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = pos
    return t


class Urdf:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {}
        self.child_of = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None, (0, 0, 0))
            rpy = _floats(origin.get("rpy") if origin is not None else None, (0, 0, 0))
            axis = j.find("axis")
            limit = j.find("limit")
            info = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": np.array(xyz),
                "rpy": rpy,
                "axis": _floats(axis.get("xyz") if axis is not None else None, (0, 0, 1)),
                "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
            }
            self.joints[info["name"]] = info
            self.child_of[info["child"]] = info
        self.links = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            origin = inertial.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None, (0, 0, 0))
            self.links[link.get("name")] = (float(inertial.find("mass").get("value")), np.array(xyz))

    def chain_to(self, link):
        chain = []
        while link in self.child_of:
            j = self.child_of[link]
            chain.append(j)
            link = j["parent"]
        return list(reversed(chain))

    def transforms(self, link, q):
        """Joint frames and the link frame, expressed in the root frame."""
        t = np.eye(4)
        joint_frames = {}
        for j in self.chain_to(link):
            t = t @ homogeneous(rpy_matrix(*j["rpy"]), j["xyz"])
            joint_frames[j["name"]] = t.copy()
            if j["type"] in ("revolute", "continuous"):
                t = t @ homogeneous(axis_angle(j["axis"], q.get(j["name"], 0.0)), (0, 0, 0))
            elif j["type"] == "prismatic":
                ax = np.array(j["axis"], dtype=float)
                t = t @ homogeneous(np.eye(3), ax / np.linalg.norm(ax) * q.get(j["name"], 0.0))
        return joint_frames, t

    def gravity_torques(self, q, payload_kg=0.0, payload_link="tool_tip"):
        torques = {}
        masses = []  # (link name, mass, world position)
        for name, (mass, com) in self.links.items():
            _, t = self.transforms(name, q)
            masses.append((name, mass, (t @ np.append(com, 1.0))[:3]))
        if payload_kg:
            _, t = self.transforms(payload_link, q)
            masses.append((payload_link, payload_kg, t[:3, 3]))
        for jname in ARM_JOINTS:
            frames, _ = self.transforms(self.joints[jname]["child"], q)
            frame = frames[jname]
            axis_world = frame[:3, :3] @ (np.array(self.joints[jname]["axis"], dtype=float))
            origin = frame[:3, 3]
            tau = 0.0
            for link, mass, pos in masses:
                # Only links beyond this joint load it.
                if not any(j["name"] == jname for j in self.chain_to(link)):
                    continue
                tau += float(np.dot(axis_world, np.cross(pos - origin, np.array([0, 0, -mass * G]))))
            torques[jname] = abs(tau)
        return torques


def check_fk(urdf, samples=500):
    rng = random.Random(1)
    worst = 0.0
    for _ in range(samples):
        q = [rng.uniform(lo, hi) for lo, hi in zip(kin.DEFAULT_LIMITS.lower, kin.DEFAULT_LIMITS.upper)]
        _, t = urdf.transforms("tool_tip", dict(zip(ARM_JOINTS, q)))
        pose = kin.forward(q)
        err = np.linalg.norm(t[:3, 3] - np.array([pose.x, pose.y, pose.z]))
        worst = max(worst, err)
    return worst


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("urdf", help="expanded URDF file (output of xacro)")
    parser.add_argument("--payload", type=float, default=0.05, help="payload in kg (default 0.05)")
    args = parser.parse_args()

    urdf = Urdf(args.urdf)

    err = check_fk(urdf)
    print(f"[1] URDF forward kinematics vs kinematics.py: worst error {err * 1000:.4f} mm")
    ok = err < 1e-6
    print("    PASS" if ok else "    FAIL: URDF dimensions and ArmGeometry do not match")

    stretched = {"base_yaw_joint": 0.0, "shoulder_joint": math.pi / 2, "elbow_joint": 0.0, "wrist_joint": 0.0}
    print(f"\n[2] Static gravity torque, arm stretched out horizontally (worst case), payload {args.payload * 1000:.0f} g")
    print(f"    {'joint':<18}{'no payload':>12}{'with payload':>14}{'effort limit':>14}{'margin':>9}")
    bare = urdf.gravity_torques(stretched)
    loaded = urdf.gravity_torques(stretched, args.payload)
    for name in ARM_JOINTS[1:]:
        limit = urdf.joints[name]["effort"]
        margin = limit / loaded[name] if loaded[name] > 1e-9 else float("inf")
        print(f"    {name:<18}{bare[name]:>10.3f} Nm{loaded[name]:>12.3f} Nm{limit:>12.2f} Nm{margin:>8.1f}x")
        if margin < 1.0:
            ok = False
    print("\n    A margin below about 1.5x is risky for hobby servos (see docs/design.md).")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
