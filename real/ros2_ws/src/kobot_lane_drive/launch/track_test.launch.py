import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kobot_lane_drive"), "config", "lane_drive.yaml"
    )
    armed = LaunchConfiguration("armed")
    calibration_ready = LaunchConfiguration("calibration_ready")
    return LaunchDescription([
        DeclareLaunchArgument("armed", default_value="false"),
        DeclareLaunchArgument("calibration_ready", default_value="false"),
        DeclareLaunchArgument("camera_x_m", default_value="0.0"),
        DeclareLaunchArgument("camera_y_m", default_value="0.0"),
        DeclareLaunchArgument("camera_z_m", default_value="0.0"),
        DeclareLaunchArgument("camera_roll_rad", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch_rad", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument("config", default_value=config),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_mount",
            arguments=[
                "--x", LaunchConfiguration("camera_x_m"),
                "--y", LaunchConfiguration("camera_y_m"),
                "--z", LaunchConfiguration("camera_z_m"),
                "--roll", LaunchConfiguration("camera_roll_rad"),
                "--pitch", LaunchConfiguration("camera_pitch_rad"),
                "--yaw", LaunchConfiguration("camera_yaw_rad"),
                "--frame-id", "base_link",
                "--child-frame-id", "camera_link",
            ],
            condition=IfCondition(calibration_ready),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_mount_to_optical",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "-1.57079632679", "--pitch", "0.0", "--yaw", "-1.57079632679",
                "--frame-id", "camera_link",
                "--child-frame-id", "default_cam",
            ],
            condition=IfCondition(calibration_ready),
        ),
        Node(
            package="kobot_lane_drive",
            executable="lane_perception",
            name="lane_perception",
            parameters=[LaunchConfiguration("config")],
            output="screen",
        ),
        Node(
            package="kobot_lane_drive",
            executable="drive_bridge",
            name="lane_drive_bridge",
            parameters=[LaunchConfiguration("config"), {"armed": True}],
            condition=IfCondition(armed),
            output="screen",
        ),
    ])
