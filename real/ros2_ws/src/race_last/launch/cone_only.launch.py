"""콘 추종 시험 전용 — 라이다와 기록만 띄운다.

race.launch.py 는 카메라·차선·신호등·앞차·대시보드까지 전부 올린다. 콘 기하만
시험할 때는 그게 방해가 된다. 특히 dashboard_server 가 /xycar_motor 에 함께
발행해서 cone_follow 의 명령과 섞인다 (0823: 발행자 2개 확인).

그래서 여기서는 세 개만 띄운다.
  · 라이다      cone_follow 가 먹을 /scan
  · 콘 인식     원본 스캔을 jsonl 로 남긴다 — 나중에 프레임 단위로 재현할 근거
  · 모터 기록   실제 속도·전류·전압. '명령은 갔는데 안 나갔다' 를 가릴 유일한 증거

모터는 따로 띄운다:  ros2 launch kobot_vehicle motor.launch.py armed:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    kv_config = os.path.join(
        get_package_share_directory('kobot_vehicle'), 'config')
    return LaunchDescription([
        Node(package='xycar_lidar', executable='xycar_lidar_node',
             name='xycar_lidar_node',
             parameters=[os.path.join(kv_config, 'lidar.yaml')],
             output='log'),
        # 원본 스캔 기록 — force_enabled 로 신호등 없이도 항상 돈다
        Node(package='race_last', executable='cone_perception',
             name='cone_perception',
             parameters=[{'force_enabled': True}], output='log'),
        Node(package='race_last', executable='motor_logger',
             name='motor_logger', output='log'),
    ])
