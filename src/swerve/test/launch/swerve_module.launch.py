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
            "swerve_controller.yaml"
        ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node"
    )

    test_node = Node(
        package="swerve",
        executable="testPublisher"
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

    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller", "--param-file", robot_controllers]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", robot_controllers]
    )

    nodes = [
        joy_node,
        test_node,
        control_node,
        robot_state_pub_node,
        swerve_controller_spawner,
        # joint_state_broadcaster
    ]

    return LaunchDescription(nodes)
