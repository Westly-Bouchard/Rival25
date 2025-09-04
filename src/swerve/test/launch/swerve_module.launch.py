from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("swerve"), "urdf", "swerve_module.urdf.xacro"]
            )
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("swerve"),
            "config",
            "swerve_module_controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", robot_controllers]
    )

    azimuth_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--param-file", robot_controllers]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        drive_controller_spawner,
        azimuth_controller_spawner
    ]

    return LaunchDescription(nodes)
