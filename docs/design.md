# Design notes

## Mechanical layout

Base yaw servo in the base, then shoulder, elbow and wrist pitch servos, then a 2-finger gripper.
Links are plywood bars; the model uses simple boxes and cylinders (no meshes), so there is nothing to
export when you change a dimension.

To change a dimension, edit the `xacro:property` values in
[`robot_arm.urdf.xacro`](../src/robot_arm_description/urdf/robot_arm.urdf.xacro) **and** the
`ArmGeometry` defaults in `kinematics.py`. Then run `tools/urdf_check.py`: it fails if the two disagree.

## Mass and torque estimate

Masses in the URDF are estimates: MG996R servo 55 g, MG90S 14 g, plywood links 25 – 30 g each.
`tools/urdf_check.py` computes the static gravity torque with the arm stretched out horizontally
(the worst case). With the default model and a 50 g payload:

| Joint | Torque | URDF effort limit | Margin |
|---|---|---|---|
| shoulder | about 0.62 N·m | 1.0 N·m | about 1.6× |
| elbow | about 0.29 N·m | 1.0 N·m | about 3.4× |
| wrist | about 0.07 N·m | 0.5 N·m | about 7.5× |

The effort limits are roughly the stall torque of the servos (an MG996R is about 1 N·m, i.e. 10 kg·cm).
Holding a load near stall for long overheats a hobby servo, so a margin under about 2× is a risk.

**If the real shoulder struggles:**
- Use two MG996R servos on the shoulder, one on each side of the same axis.
- Add a small counterweight behind the shoulder, or a rubber band / spring that offsets the arm weight.
- Shorten the forearm, or use lighter links (thin plywood, hollowed out).
- Keep the payload small. The gripper is meant for light objects such as a cube of a few tens of grams.

## Simulation-to-hardware plan

Only the hardware layer changes:

- **Sim:** `gz_ros2_control/GazeboSimSystem` in `robot_arm.ros2_control.xacro`.
- **Real arm:** a hardware interface (or a small ROS 2 node) that sends joint angles over USB serial to
  the microcontroller, which drives the servos.

What stays: the URDF, TF, the IK node, the trajectory planning, the controllers, the demo.

What you will need to add on the real arm (simulation hides these): servo calibration (zero position,
direction, limits), backlash and sag compensation, a proper 5 – 6 V supply for the servos with a large
capacitor, software limits against self-collision, and force/position limits on the gripper.

## Known simplifications

- The demo moves in joint space between waypoints, so the tool does not travel in a perfectly straight
  line. The approach and lift moves are short and mostly vertical, which is good enough for a pick.
- Position control in Gazebo is ideal compared with a hobby servo (no backlash, no jitter).
- The gripper is a position-controlled pair of fingers; grasp success depends on Gazebo contact tuning.
