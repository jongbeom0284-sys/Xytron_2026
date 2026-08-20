from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 정규화 색도 + 원형성 방식이라 YOLO 도 ultralytics 도 쓰지 않는다.
    # numpy 충돌을 피하려고 별도 venv 를 PYTHONPATH 에 넣지 않는다.
    return LaunchDescription([
        Node(
            package='kobot_vehicle',
            executable='traffic_perception',
            name='traffic_perception_node',
            output='screen',
            parameters=[{'debug_image': True, 'max_hz': 20.0}],
        ),
    ])
