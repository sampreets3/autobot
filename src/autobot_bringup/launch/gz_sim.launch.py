import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world = os.path.join(
        get_package_share_directory("autobot_gazebo"), "worlds", "empty_world.world"
    )

    description_pkg = os.path.join(get_package_share_directory("autobot_description"))
    xacro_file = os.path.join(description_pkg, "xacro", "autobot_core.xacro")

    autobot_params = os.path.join(
        get_package_share_directory("autobot_bringup"), "config", "autobot_params.yaml"
    )

    bridge_params = os.path.join(
        get_package_share_directory("autobot_gazebo"),
        "config",
        "autobot_gz_bridge.yaml",
    )

    robot_description_config = Command(["xacro ", xacro_file])

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}],
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_robot)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
