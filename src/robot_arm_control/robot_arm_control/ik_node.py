"""Inverse-kinematics node: send the tool tip to a point in space.

Subscribes to ``target_point`` (geometry_msgs/PointStamped, in ``base_link``)
and publishes a joint trajectory for ``arm_controller``.

Parameters
----------
tool_pitch_deg : float, default 180.0
    Desired tool pitch (180 = gripper pointing straight down).
auto_pitch : bool, default True
    If the point is unreachable at ``tool_pitch_deg``, tilt the tool until it is.
move_time : float, default 2.0
    Seconds the arm takes to reach the target.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from . import kinematics as kin
from .commands import ARM_TRAJECTORY_TOPIC, make_trajectory


class ArmIkNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_ik_node")
        self.declare_parameter("tool_pitch_deg", 180.0)
        self.declare_parameter("auto_pitch", True)
        self.declare_parameter("move_time", 2.0)

        self._last_q: Optional[kin.Joints] = None
        self._pub = self.create_publisher(JointTrajectory, ARM_TRAJECTORY_TOPIC, 10)
        self.create_subscription(PointStamped, "target_point", self._on_target, 10)
        self.get_logger().info(
            "Ready. Publish a geometry_msgs/PointStamped (frame base_link) on 'target_point'."
        )

    def _on_target(self, msg: PointStamped) -> None:
        if msg.header.frame_id not in ("", "base_link"):
            self.get_logger().warn(
                f"Target frame '{msg.header.frame_id}' is not base_link; treating it as base_link."
            )

        p = msg.point
        pitch = math.radians(self.get_parameter("tool_pitch_deg").value)
        auto = self.get_parameter("auto_pitch").value
        move_time = float(self.get_parameter("move_time").value)

        if auto:
            result = kin.inverse_auto(p.x, p.y, p.z, preferred_pitch=pitch, seed=self._last_q)
            q, used = (result if result is not None else (None, None))
        else:
            q = kin.inverse(p.x, p.y, p.z, pitch, seed=self._last_q)
            used = pitch

        if q is None:
            self.get_logger().warn(
                f"Target ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m is out of reach "
                f"(max reach about {kin.max_reach():.2f} m)."
            )
            return

        self._last_q = q
        self.get_logger().info(
            f"Target ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m, tool pitch {math.degrees(used):.0f} deg "
            f"-> joints [{', '.join(f'{v:.3f}' for v in q)}] rad"
        )
        self._pub.publish(make_trajectory(q, move_time))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmIkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
