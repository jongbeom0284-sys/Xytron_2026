from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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
    ])
