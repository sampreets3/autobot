import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments

    description_pkg = os.path.join(get_package_share_directory("autobot_description"))
    xacro_file = os.path.join(description_pkg, "xacro", "autobot_core.xacro")

    autobot_params = os.path.join(
        get_package_share_directory("autobot_bringup"), "config", "autobot_params.yaml"
    )

    robot_description_config = Command(["xacro ", xacro_file])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "autobot"],
        output="screen",
    )

    # ROS2_CONTROL bits
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broadcast_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    laser_scan_merger_node = Node(
        package="autobot_laser_tools",
        executable="laser_scan_merger",
        output="screen",
        parameters=[autobot_params],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            gazebo_node,
            spawn_entity_node,
            diff_drive_spawner,
            joint_broadcast_spawner,
            laser_scan_merger_node,
        ]
    )
