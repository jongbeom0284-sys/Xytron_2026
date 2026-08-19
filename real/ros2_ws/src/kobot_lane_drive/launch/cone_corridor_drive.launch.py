import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kobot_lane_drive"),
        "config",
        "twinmixing_center.yaml",
    )
    armed = LaunchConfiguration("armed")
    max_speed_mps = LaunchConfiguration("max_speed_mps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "armed",
            default_value="false",
            description="Must be true before the motor bridge process exists.",
        ),
        DeclareLaunchArgument(
            "max_speed_mps",
            default_value="0.25",
            description="Speed cap for cone corridor drive.",
        ),
        Node(
            package="kobot_lane_drive",
            executable="cone_corridor_drive",
            name="cone_corridor_drive",
            parameters=[
                config,
                {
                    "cone_drive_speed_mps": ParameterValue(
                        max_speed_mps, value_type=float
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="kobot_lane_drive",
            executable="twinmixing_center_drive_bridge",
            name="cone_corridor_drive_bridge",
            parameters=[
                config,
                {
                    "armed": True,
                    "max_speed_mps": ParameterValue(max_speed_mps, value_type=float),
                    "drive_preview_topic": "/kobot_lane_drive/cone_corridor/drive_preview",
                    "status_topic": "/kobot_lane_drive/cone_corridor/status",
                },
            ],
            condition=IfCondition(armed),
            output="screen",
        ),
    ])
