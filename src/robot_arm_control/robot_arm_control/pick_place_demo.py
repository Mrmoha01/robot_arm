"""Scripted pick-and-place demo.

Plans the whole sequence with inverse kinematics first (so an unreachable
point is reported before anything moves), then plays it back step by step.

Parameters
----------
object_xy, place_xy : [x, y] in metres (base_link frame)
home_xyz            : [x, y, z] tool-tip position of the home pose
approach_z, grasp_z, place_z : heights in metres
move_time           : seconds for a normal move
repeat              : run the sequence again when it finishes
start_delay         : seconds to wait before starting (controllers need to be up)
"""

from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from .commands import (
    ARM_TRAJECTORY_TOPIC,
    GRIPPER_COMMAND_TOPIC,
    make_gripper_command,
    make_trajectory,
)
from .pick_place_plan import Step, plan_pick_place


class PickPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_demo")
        self.declare_parameter("object_xy", [0.25, -0.10])
        self.declare_parameter("place_xy", [0.25, 0.10])
        self.declare_parameter("home_xyz", [0.25, 0.0, 0.14])
        self.declare_parameter("approach_z", 0.10)
        self.declare_parameter("grasp_z", 0.02)
        self.declare_parameter("place_z", 0.025)
        self.declare_parameter("move_time", 2.5)
        self.declare_parameter("repeat", True)
        self.declare_parameter("start_delay", 3.0)

        get = lambda name: self.get_parameter(name).value  # noqa: E731
        try:
            self._steps: List[Step] = plan_pick_place(
                object_xy=get("object_xy"),
                place_xy=get("place_xy"),
                home_xyz=get("home_xyz"),
                approach_z=get("approach_z"),
                grasp_z=get("grasp_z"),
                place_z=get("place_z"),
                move_time=get("move_time"),
            )
        except ValueError as err:
            self.get_logger().error(f"Cannot plan the demo: {err}")
            raise

        self._repeat = bool(get("repeat"))
        self._arm_pub = self.create_publisher(JointTrajectory, ARM_TRAJECTORY_TOPIC, 10)
        self._gripper_pub = self.create_publisher(Float64MultiArray, GRIPPER_COMMAND_TOPIC, 10)

        self._index = -1
        self._next_time = self.get_clock().now().nanoseconds * 1e-9 + float(get("start_delay"))
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Planned {len(self._steps)} steps. Starting shortly...")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        if self._now() < self._next_time:
            return
        if self._arm_pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for arm_controller...", throttle_duration_sec=2.0)
            return

        self._index += 1
        if self._index >= len(self._steps):
            if not self._repeat:
                self.get_logger().info("Demo finished.")
                self._timer.cancel()
                return
            self._index = 0

        step = self._steps[self._index]
        self.get_logger().info(f"[{self._index + 1}/{len(self._steps)}] {step.name}")
        if step.arm is not None:
            self._arm_pub.publish(make_trajectory(step.arm, step.duration))
        if step.gripper is not None:
            self._gripper_pub.publish(make_gripper_command(step.gripper))
        self._next_time = self._now() + step.duration + 0.3


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = PickPlaceDemo()
    except ValueError:
        rclpy.shutdown()
        return
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
