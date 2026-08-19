import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("kobot_lane_drive")
    perception_config = os.path.join(share, "config", "fast_lane.yaml")
    control_config = os.path.join(share, "config", "fast_lane_control.yaml")
    armed = LaunchConfiguration("armed")
    start_perception = LaunchConfiguration("start_perception")
    max_speed_mps = LaunchConfiguration("max_speed_mps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "armed",
            default_value="false",
            description="Must be explicitly true to publish /xycar_motor.",
        ),
        DeclareLaunchArgument(
            "start_perception",
            default_value="true",
            description="Start fast_lane_perception in this launch.",
        ),
        DeclareLaunchArgument(
            "max_speed_mps",
            default_value="0.70",
            description="Hard cap shared by controller and armed bridge.",
        ),
        Node(
            package="kobot_lane_drive",
            executable="fast_lane_perception",
            name="fast_lane_perception",
            parameters=[perception_config],
            condition=IfCondition(start_perception),
            output="screen",
        ),
        Node(
            package="kobot_lane_drive",
            executable="fast_lane_controller",
            name="fast_lane_controller",
            parameters=[control_config, {"max_speed_mps": max_speed_mps}],
            output="screen",
        ),
        Node(
            package="kobot_lane_drive",
            executable="fast_lane_drive_bridge",
            name="fast_lane_drive_bridge",
            parameters=[
                control_config,
                {"armed": True, "max_speed_mps": max_speed_mps},
            ],
            condition=IfCondition(armed),
            output="screen",
        ),
    ])
