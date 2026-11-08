"""Run the scripted pick-and-place demo (start sim.launch.py first)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("repeat", default_value="true", description="Loop the demo"),
            Node(
                package="robot_arm_control",
                executable="pick_place_demo",
                parameters=[
                    {
                        "use_sim_time": True,
                        "repeat": ParameterValue(LaunchConfiguration("repeat"), value_type=bool),
                    }
                ],
                output="screen",
            ),
        ]
    )
