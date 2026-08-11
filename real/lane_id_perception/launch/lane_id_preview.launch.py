import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lane_id_perception"),
        "config",
        "lane_id.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            Node(
                package="lane_id_perception",
                executable="lane_id_preview",
                name="lane_id_perception",
                output="screen",
                parameters=[config, {"image_topic": LaunchConfiguration("image_topic")}],
            ),
        ]
    )
