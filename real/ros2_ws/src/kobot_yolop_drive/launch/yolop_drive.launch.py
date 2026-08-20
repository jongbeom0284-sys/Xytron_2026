"""YOLOPv2 주행 노드 launch.

armed 는 기본 false 다. 모터 발행은 실차 승인 뒤에만 켠다.
    ros2 launch kobot_yolop_drive yolop_drive.launch.py armed:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('kobot_yolop_drive')
    config = os.path.join(share, 'config', 'yolop_drive.yaml')
    armed = LaunchConfiguration('armed')
    speed = LaunchConfiguration('speed_units')
    speed_mps = LaunchConfiguration('max_speed_mps')
    return LaunchDescription([
        DeclareLaunchArgument(
            'armed', default_value='false',
            description='true 여야 모터 토픽에 발행한다. 실차 승인 후에만.'),
        DeclareLaunchArgument(
            'speed_units', default_value='5.0',
            description='legacy 속도 단위. 5 = 약 0.40 m/s'),
        DeclareLaunchArgument(
            'max_speed_mps', default_value='0.0',
            description='0 보다 크면 이 m/s 값이 speed_units 를 대체한다'),
        Node(
            package='kobot_yolop_drive',
            executable='yolop_drive',
            name='yolop_drive',
            output='screen',
            parameters=[
                config,
                {'armed': ParameterValue(armed, value_type=bool)},
                {'speed_units': ParameterValue(speed, value_type=float)},
                {'max_speed_mps': ParameterValue(speed_mps,
                                                 value_type=float)},
            ],
        ),
    ])
