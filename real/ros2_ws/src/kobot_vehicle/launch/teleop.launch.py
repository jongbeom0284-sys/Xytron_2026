import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("kobot_vehicle")
    sensors_launch = os.path.join(share, "launch", "sensors.launch.py")
    motor_launch = os.path.join(share, "launch", "motor.launch.py")

    start_sensors = LaunchConfiguration("start_sensors")
    armed = LaunchConfiguration("armed")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_sensors",
            default_value="false",
            description=(
                "Start sensors.launch.py in this process. Keep false on Kobot "
                "while kobot-sensors.service is active."
            ),
        ),
        DeclareLaunchArgument(
            "armed",
            default_value="false",
            description="Explicitly start the VESC and motor command chain.",
        ),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("ultrasonic", default_value="true"),
        DeclareLaunchArgument("imu", default_value="true"),
        DeclareLaunchArgument("lidar", default_value="true"),
        LogInfo(
            condition=UnlessCondition(start_sensors),
            msg=(
                "Reusing sensors from kobot-sensors.service. Set "
                "start_sensors:=true only when that service is stopped."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            condition=IfCondition(start_sensors),
            launch_arguments={
                "camera": LaunchConfiguration("camera"),
                "ultrasonic": LaunchConfiguration("ultrasonic"),
                "imu": LaunchConfiguration("imu"),
                "lidar": LaunchConfiguration("lidar"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motor_launch),
            launch_arguments={"armed": armed}.items(),
        ),
    ])
