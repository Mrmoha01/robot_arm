# Kinematics

Everything here is implemented in
[`kinematics.py`](../src/robot_arm_control/robot_arm_control/kinematics.py) and covered by
[`test_kinematics.py`](../src/robot_arm_control/test/test_kinematics.py).

## Conventions

- Frame `base_link`: x forward, y left, z up, origin on the ground under the base axis.
- `q1` is the base yaw (about +z). `q2`, `q3`, `q4` are shoulder, elbow and wrist pitch (about +y).
- All joints at zero: the arm points straight up.
- A positive pitch tilts a link from vertical towards +x, so a link at absolute pitch `a` points along
  `(sin a, cos a)` in the vertical plane of the arm.
- Absolute pitches: `a1 = q2`, `a2 = q2 + q3`, tool pitch `phi = q2 + q3 + q4`.
  `phi = 180°` means the gripper points straight down.

## Dimensions

| Symbol | Meaning | Value |
|---|---|---|
| `h0` | ground to shoulder axis | 80 mm |
| `L1` | shoulder to elbow | 150 mm |
| `L2` | elbow to wrist | 150 mm |
| `L3` | wrist to fingertip centre | 80 mm |

## Forward kinematics

In the arm's vertical plane, with `r` the horizontal distance from the base axis:

```
r = L1 sin(a1) + L2 sin(a2) + L3 sin(phi)
z = h0 + L1 cos(a1) + L2 cos(a2) + L3 cos(phi)
x = r cos(q1)        y = r sin(q1)
```

## Inverse kinematics

Given a target tip position `(x, y, z)` and a tool pitch `phi`:

1. **Base yaw:** `q1 = atan2(y, x)`, and `r = sqrt(x² + y²)`.
2. **Wrist point:** remove the tool segment from the target:
   `wr = r − L3 sin(phi)`, `wh = z − L3 cos(phi) − h0`.
3. **Elbow (law of cosines):**
   `cos q3 = (wr² + wh² − L1² − L2²) / (2 L1 L2)`. If the magnitude exceeds 1 the point is out of reach.
   `q3 = +acos(...)` is the elbow-up solution and `−acos(...)` the elbow-down one.
4. **Shoulder:** `q2 = atan2(wr, wh) − atan2(L2 sin q3, L1 + L2 cos q3)`.
5. **Wrist:** `q4 = phi − (q2 + q3)`, wrapped to (−180°, 180°].

A second family of solutions turns the base by 180° and uses a negative `r` (the arm leans back over its
base). `inverse_all` returns every solution that respects the joint limits, and `inverse` picks one:
the closest to a seed (the current joints) if given, otherwise elbow-up with the smallest base yaw.

### Automatic tool pitch

`inverse_auto` tries the preferred pitch first (straight down), then tilts the tool 2° at a time, up to
90° either way, until a solution exists inside the joint limits.

## Workspace (±90° joint limits)

Measured with the code (`y = 0`, tool tip at 20 mm height):

| Case | Reach in front of the base |
|---|---|
| Gripper pointing straight down | about 212 – 298 mm |
| Tool tilt allowed (up to 90° from vertical) | about 155 – 372 mm |

At `x = 250 mm` the tool tip can reach heights from the ground up to about 325 mm.
The elbow limit is what stops very close points: reaching 150 mm in front of the base would need an
elbow bend of more than 90°.

## How correctness is checked

- Known poses (straight up, stretched forward, base turned 90°).
- 2000 random joint configurations: forward kinematics, then every inverse solution, must reproduce the
  same tip position and tool pitch to 1e-9 m.
- `tools/urdf_check.py` computes forward kinematics straight from the URDF and compares it with
  `kinematics.py`, so the xacro and the code cannot drift apart unnoticed.
