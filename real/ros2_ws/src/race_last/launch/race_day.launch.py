"""본선 한 줄 실행 — 뇌 + 근육 + 자동 잠금해제.

    ros2 launch race_last race_day.launch.py                # 속도 2.05 m/s
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
            'speed', default_value='2.05',
            description='차선 주행 최고 속도 [m/s] (콘 구간은 별도 정책). '
                        '직진 구간은 straight_speed_mps(1.9), 최대 조향에서는 '
                        'curve_min_speed_mps(1.8) 로 따로 내려간다.'),

        # 뇌: 인식 전체 + 터미널 상황판 (dashboard 기본 꺼짐 그대로)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(race_share, 'launch', 'race.launch.py'))),

        # 근육: 모터 사슬 (motor_adapter 가속 4.0 기본값 포함)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(kv_share, 'launch', 'motor.launch.py')),
            launch_arguments={
                'armed': 'true',
                # 0824: 4.0 -> 8.0. 실측(motorlog 015354)에서 0 -> 1.9 m/s
                # 90% 도달에 0.49초가 걸렸는데 그게 정확히 1.9/4.0 이다.
                # 그때 duty 는 16% 뿐이라 모터·배터리는 여유가 있었다 —
                # 순수하게 이 램프가 잡고 있었다. 출발선에서만 0 부터
                # 올리므로(콘 직후는 1.05 에서 시작) 출발이 유독 굼떴다.
                'acceleration_limit_mps2': '8.0',
            }.items()),

        # 잠금해제: 준비 확인 후 enabled+armed — 이후는 신호등이 지배
        Node(package='race_last', executable='race_starter',
             name='race_starter',
             parameters=[{'speed_mps': ParameterValue(speed,
                                                      value_type=float)}],
             output='screen'),
    ])
