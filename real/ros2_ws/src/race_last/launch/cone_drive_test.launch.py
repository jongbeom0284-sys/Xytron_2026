"""Isolated real-vehicle test for the cone controller used by race_last.

This launch starts only the LiDAR, Cone_Perception, the integrated ConeLink
adapter and the motor telemetry logger.  It does not start YOLOP, lane,
traffic-light, vehicle perception, the dashboard or the motor stack.

Start the motor stack in a separate terminal, then run:

  ros2 launch race_last cone_drive_test.launch.py \
    armed:=true max_speed_mps:=1.5 acceleration_limit_mps2:=4.0

Never run this together with race.launch.py, race_day.launch.py, the dashboard
motor test, or kobot_lane_drive's cone controller.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    kv_config = os.path.join(
        get_package_share_directory('kobot_vehicle'), 'config')

    armed = LaunchConfiguration('armed')
    max_speed = LaunchConfiguration('max_speed_mps')
    acceleration = LaunchConfiguration('acceleration_limit_mps2')
    cone_speed_scale = LaunchConfiguration('cone_speed_scale')
    start_lidar = LaunchConfiguration('start_lidar')
    wall_bias = LaunchConfiguration('wall_bias_cm')

    return LaunchDescription([
        DeclareLaunchArgument(
            'armed', default_value='false',
            description='true publishes real /xycar_motor commands'),
        DeclareLaunchArgument(
            'max_speed_mps', default_value='1.5',
            description='hard cap for this isolated cone test'),
        DeclareLaunchArgument(
            'acceleration_limit_mps2', default_value='4.0',
            description='controller acceleration limit for cone testing'),
        DeclareLaunchArgument(
            'cone_speed_scale', default_value='1.05',
            description='18 legacy cone units become about 1.5 m/s'),
        DeclareLaunchArgument(
            'start_lidar', default_value='true',
            description='start xycar_lidar_node inside this launch'),
        DeclareLaunchArgument(
            'wall_bias_cm', default_value='0.0',
            description='integrated cone wall target bias in centimetres'),

        Node(
            package='xycar_lidar', executable='xycar_lidar_node',
            name='xycar_lidar_node',
            parameters=[os.path.join(kv_config, 'lidar.yaml')],
            output='log', condition=IfCondition(start_lidar)),

        Node(
            package='race_last', executable='cone_perception',
            name='cone_perception',
            parameters=[{
                'force_enabled': True,
                'wall_bias_cm': ParameterValue(wall_bias, value_type=float),
            }],
            output='screen'),

        Node(
            package='race_last', executable='integrated_cone_test',
            name='integrated_cone_test',
            parameters=[{
                'armed': ParameterValue(armed, value_type=bool),
                'max_speed_mps': ParameterValue(max_speed, value_type=float),
                'acceleration_limit_mps2': ParameterValue(
                    acceleration, value_type=float),
                'cone_speed_scale': ParameterValue(
                    cone_speed_scale, value_type=float),
            }],
            output='screen', emulate_tty=True),

        Node(
            package='race_last', executable='motor_logger',
            name='motor_logger', output='log'),
    ])
