"""View the arm in RViz2 and move its joints with sliders (no simulation)."""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("robot_arm_description")
    xacro_file = PathJoinSubstitution([pkg, "urdf", "robot_arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "display.rviz"])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, " use_sim:=false"]),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
