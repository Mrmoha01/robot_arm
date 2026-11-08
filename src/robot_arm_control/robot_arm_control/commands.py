"""Small helpers shared by the ROS nodes (message construction only)."""

from typing import Sequence

from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["base_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
ARM_TRAJECTORY_TOPIC = "/arm_controller/joint_trajectory"
GRIPPER_COMMAND_TOPIC = "/gripper_controller/commands"


def make_trajectory(q: Sequence[float], seconds: float) -> JointTrajectory:
    """One-point trajectory that reaches joint targets ``q`` after ``seconds``."""
    msg = JointTrajectory()
    msg.joint_names = list(ARM_JOINTS)
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in q]
    whole = int(seconds)
    point.time_from_start = Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))
    msg.points.append(point)
    return msg


def make_gripper_command(opening: float) -> Float64MultiArray:
    """Both fingers to ``opening`` metres (the right finger's axis is mirrored in the URDF)."""
    msg = Float64MultiArray()
    msg.data = [float(opening), float(opening)]
    return msg
