import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kobot_lane_drive"),
        "config",
        "fast_lane.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        Node(
            package="kobot_lane_drive",
            executable="fast_lane_perception",
            name="fast_lane_perception",
            parameters=[LaunchConfiguration("config")],
            output="screen",
        ),
    ])
