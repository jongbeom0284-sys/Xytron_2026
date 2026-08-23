"""본선 한 줄 실행 — 뇌 + 근육 + 자동 잠금해제.

    ros2 launch race_last race_day.launch.py                # 속도 2.0 m/s
    ros2 launch race_last race_day.launch.py speed:=1.0     # 속도 지정

이 한 줄로: 인식 전체(race.launch.py) + 모터(motor.launch.py armed) 가 뜨고,
race_starter 가 준비 확인 후 잠금을 풀어 "빨간불 대기 → 초록불 자동 출발"
상태로 만든다. 정지는 ~/race_halt.sh (또는 이 launch 를 Ctrl+C).

⚠ 이 launch 는 곧 "무장"이다 — 차를 코스에 놓은 뒤에만 실행한다.
   벤치 디버깅은 race.launch.py (인식만) 를 쓴다.
사전 1회: sudo systemctl stop kobot-sensors.service (부팅 자동 구 스택 내림)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    race_share = get_package_share_directory('race_last')
    kv_share = get_package_share_directory('kobot_vehicle')
    speed = LaunchConfiguration('speed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'speed', default_value='2.0',
            description='차선 주행 최고 속도 [m/s] (콘 구간은 별도 정책)'),

        # 뇌: 인식 전체 + 터미널 상황판 (dashboard 기본 꺼짐 그대로)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(race_share, 'launch', 'race.launch.py'))),

        # 근육: 모터 사슬 (motor_adapter 가속 4.0 기본값 포함)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(kv_share, 'launch', 'motor.launch.py')),
            launch_arguments={
                'armed': 'true',
                'acceleration_limit_mps2': '4.0',
            }.items()),

        # 잠금해제: 준비 확인 후 enabled+armed — 이후는 신호등이 지배
        Node(package='race_last', executable='race_starter',
             name='race_starter',
             parameters=[{'speed_mps': ParameterValue(speed,
                                                      value_type=float)}],
             output='screen'),
    ])
