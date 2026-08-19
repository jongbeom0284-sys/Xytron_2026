from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import yaml


def camera_control_command(config_dir):
    """usb_cam 이 장치에 쓰지 못하는 v4l2 제어를 직접 건다.

    usb_cam 은 camera.yaml 의 brightness/autoexposure/exposure 를 ROS 파라미터로는
    읽지만 실제 장치에는 쓰지 않는다. 2026-08-19 재부팅 후 확인: 파라미터는
    autoexposure=False, exposure=40 인데 장치는 auto_exposure=3(자동),
    exposure_time_absolute=157 이었다. 그래서 노드가 뜬 뒤 같은 값을 한 번 더 건다.

    값의 출처는 camera.yaml 하나로 유지한다. 여기에 숫자를 새로 박지 않는다.
    """
    with open(os.path.join(config_dir, 'camera.yaml'), encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['/**']['ros__parameters']
    device = params.get('video_device', '/dev/videoCAM')
    controls = ','.join([
        'brightness=%d' % int(params.get('brightness', 0)),
        'auto_exposure=%s' % ('3' if params.get('autoexposure', True) else '1'),
        'exposure_time_absolute=%d' % int(params.get('exposure', 157)),
        'white_balance_automatic=%d' % (1 if params.get('auto_white_balance', True) else 0),
        'white_balance_temperature=%d' % int(params.get('white_balance', 4600)),
    ])
    return ['v4l2-ctl', '-d', device, '--set-ctrl', controls]


def generate_launch_description():
    share = get_package_share_directory('kobot_vehicle')
    dashboard = os.path.join(share, 'dashboard')
    config = os.path.join(share, 'config')
    camera = LaunchConfiguration('camera')
    ultrasonic = LaunchConfiguration('ultrasonic')
    imu = LaunchConfiguration('imu')
    lidar = LaunchConfiguration('lidar')
    torch_site = os.environ.get(
        'KOBOT_TORCH_SITE',
        '/home/xytron/.venvs/twinmixing/lib/python3.10/site-packages',
    )
    inherited_pythonpath = os.environ.get('PYTHONPATH', '')
    pythonpath = torch_site + (os.pathsep + inherited_pythonpath if inherited_pythonpath else '')
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', pythonpath),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('ultrasonic', default_value='true'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        Node(package='usb_cam', executable='usb_cam_node_exe', name='camera',
             parameters=[os.path.join(config, 'camera.yaml')], condition=IfCondition(camera)),
        Node(package='kobot_lane_drive', executable='twinmixing_center_drive',
             name='twinmixing_center_drive',
             parameters=[os.path.join(
                 get_package_share_directory('kobot_lane_drive'),
                 'config', 'twinmixing_center.yaml')],
             condition=IfCondition(camera), output='screen'),
        Node(package='kobot_vehicle', executable='ultrasonic_node',
             parameters=[os.path.join(config, 'ultrasonic.yaml')], condition=IfCondition(ultrasonic)),
        Node(package='kobot_vehicle', executable='imu_node',
             parameters=[os.path.join(config, 'imu.yaml')], condition=IfCondition(imu)),
        Node(package='xycar_lidar', executable='xycar_lidar_node', name='xycar_lidar_node',
             parameters=[os.path.join(config, 'lidar.yaml')], condition=IfCondition(lidar)),
        Node(package='kobot_vehicle', executable='status_node'),
        Node(package='web_video_server', executable='web_video_server',
             parameters=[{'port': 8080}]),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             parameters=[{'port': 9090}]),
        Node(package='kobot_vehicle', executable='dashboard_server', output='screen'),
        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(cmd=camera_control_command(config), output='screen')],
            condition=IfCondition(camera),
        ),
    ])
