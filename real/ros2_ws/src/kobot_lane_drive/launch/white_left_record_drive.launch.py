import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kobot_lane_drive"),
        "config",
        "white_left_drive.yaml",
    )
    armed = LaunchConfiguration("armed")
    start_perception = LaunchConfiguration("start_perception")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    return LaunchDescription([
        DeclareLaunchArgument(
            "armed",
            default_value="false",
            description="Must be true before the motor bridge process exists.",
        ),
        DeclareLaunchArgument(
            "start_perception",
            default_value="true",
            description="Start white-left perception in this launch.",
        ),
        DeclareLaunchArgument(
            "max_speed_mps",
            default_value="0.20",
            description="Hard speed cap for the recording drive.",
        ),
        Node(
            package="kobot_lane_drive",
            executable="white_left_drive",
            name="white_left_drive",
            parameters=[config, {"speed_mps": max_speed_mps}],
            condition=IfCondition(start_perception),
            output="screen",
        ),
        Node(
            package="kobot_lane_drive",
            executable="white_left_drive_bridge",
            name="white_left_drive_bridge",
            parameters=[
                config,
                {"armed": True, "max_speed_mps": max_speed_mps},
            ],
            condition=IfCondition(armed),
            output="screen",
        ),
    ])
