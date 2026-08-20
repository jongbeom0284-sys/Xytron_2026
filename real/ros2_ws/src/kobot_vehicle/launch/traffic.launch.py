import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # ultralytics가 설치되어 있는 clrernet_eval venv 경로 추가
    clrernet_site = '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages'
    inherited_pythonpath = os.environ.get('PYTHONPATH', '')
    pythonpath = clrernet_site + (os.pathsep + inherited_pythonpath if inherited_pythonpath else '')

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', pythonpath),
        Node(
            package='kobot_vehicle',
            executable='traffic_perception',
            name='traffic_perception_node',
            output='screen',
        ),
    ])
