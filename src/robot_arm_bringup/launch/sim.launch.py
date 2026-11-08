"""Start Gazebo with the pick-and-place world, spawn the arm and its controllers.

Arguments
---------
rviz:=true       also open RViz2
start_ik:=false  do not start the inverse-kinematics node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("robot_arm_bringup")
    description = FindPackageShare("robot_arm_description")

    world = PathJoinSubstitution([bringup, "worlds", "pick_place.sdf"])
    controllers_file = PathJoinSubstitution([bringup, "config", "controllers.yaml"])
    xacro_file = PathJoinSubstitution([description, "urdf", "robot_arm.urdf.xacro"])
    rviz_config = PathJoinSubstitution([description, "rviz", "display.rviz"])

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_sim:=true controllers_file:=",
                controllers_file,
            ]
        ),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "robot_arm", "-z", "0.0"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    arm_controller = spawner("arm_controller")
    gripper_controller = spawner("gripper_controller")

    ik_node = Node(
        package="robot_arm_control",
        executable="ik_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_ik")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false", description="Open RViz2"),
            DeclareLaunchArgument("start_ik", default_value="true", description="Start the IK node"),
            gz_sim,
            clock_bridge,
            robot_state_publisher,
            spawn_robot,
            # Controllers start one after another once the robot exists in Gazebo.
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[arm_controller, gripper_controller],
                )
            ),
            ik_node,
            rviz,
        ]
    )
