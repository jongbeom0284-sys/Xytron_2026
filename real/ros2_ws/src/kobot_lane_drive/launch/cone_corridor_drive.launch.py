"""Cone-corridor preview plus the explicitly armed motor bridge.

Runtime contract:
* ``config_file`` selects one complete, reviewable tuning profile.
* ``max_speed_mps`` is a hard cap passed to both planner and bridge.
* ``auto_enable`` defaults false, so arming creates the bridge but does not move
  the car until ``record_drive/enabled`` is explicitly true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("kobot_lane_drive"),
        "config",
        "cone_corridor_robust.yaml",
    )
    config_file = LaunchConfiguration("config_file")
    armed = LaunchConfiguration("armed")
    auto_enable = LaunchConfiguration("auto_enable")
    max_speed_mps = LaunchConfiguration("max_speed_mps")
    enable_heartbeat_timeout_sec = LaunchConfiguration("enable_heartbeat_timeout_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description=(
                    "Complete cone tuning profile. Use a copied YAML here for "
                    "repeatable tests instead of editing the team-shared file."
                ),
            ),
            DeclareLaunchArgument(
                "armed",
                default_value="false",
                description="Must be true before the motor bridge process exists.",
            ),
            DeclareLaunchArgument(
                "auto_enable",
                default_value="false",
                description=(
                    "Publish enable from the planner at startup. Keep false for "
                    "normal tests; true deliberately removes the human enable step."
                ),
            ),
            DeclareLaunchArgument(
                "max_speed_mps",
                default_value="1.50",
                description=(
                    "Hard cap for every planner mode and the motor bridge; this "
                    "does not overwrite straight/corner/fallback profile ratios."
                ),
            ),
            DeclareLaunchArgument(
                "enable_heartbeat_timeout_sec",
                default_value="0.0",
                description=(
                    "Opt-in bridge enable lease. Zero preserves dashboard "
                    "latching; automated harnesses should pass a positive value."
                ),
            ),
            LogInfo(
                msg=[
                    "CONE_RUNTIME profile=",
                    config_file,
                    " cap_mps=",
                    max_speed_mps,
                    " auto_enable=",
                    auto_enable,
                    " armed=",
                    armed,
                    " enable_lease_sec=",
                    enable_heartbeat_timeout_sec,
                ]
            ),
            Node(
                package="kobot_lane_drive",
                executable="cone_corridor_drive",
                name="cone_corridor_drive",
                parameters=[
                    config_file,
                    {
                        "auto_enable": ParameterValue(
                            auto_enable, value_type=bool
                        ),
                        "max_speed_mps": ParameterValue(
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
                    config_file,
                    {
                        "armed": True,
                        "max_speed_mps": ParameterValue(
                            max_speed_mps, value_type=float
                        ),
                        "enable_heartbeat_timeout_sec": ParameterValue(
                            enable_heartbeat_timeout_sec, value_type=float
                        ),
                        "drive_preview_topic": (
                            "/kobot_lane_drive/cone_corridor/drive_preview"
                        ),
                        "status_topic": "/kobot_lane_drive/cone_corridor/status",
                    },
                ],
                condition=IfCondition(armed),
                output="screen",
            ),
        ]
    )
