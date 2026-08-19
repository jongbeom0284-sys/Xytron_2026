from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    armed = LaunchConfiguration('armed')
    speed_limit_mps = LaunchConfiguration('speed_limit_mps')
    acceleration_limit_mps2 = LaunchConfiguration('acceleration_limit_mps2')
    config = os.path.join(get_package_share_directory('kobot_vehicle'), 'config', 'vesc.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'armed', default_value='false',
            description='Must be explicitly true. Open VESC and allow vehicle commands.'),
        DeclareLaunchArgument(
            'speed_limit_mps', default_value='2.16',
            description='Hard motor-adapter speed cap in metres per second.'),
        DeclareLaunchArgument(
            'acceleration_limit_mps2', default_value='2.5',
            description='Motor-adapter acceleration slew limit in metres per second squared.'),
        Node(package='vesc_driver', executable='vesc_driver_node', name='vesc_driver_node',
             parameters=[config], condition=IfCondition(armed), output='screen'),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
             name='ackermann_to_vesc_node', condition=IfCondition(armed), output='screen',
             parameters=[{
                 'speed_to_erpm_gain': 4614.0,
                 'speed_to_erpm_offset': 0.0,
                 'steering_angle_to_servo_gain': -1.2135,
                 'steering_angle_to_servo_offset': 0.5004,
             }]),
        Node(package='kobot_vehicle', executable='motor_adapter', name='kobot_motor_adapter',
             parameters=[{
                 'enabled': True,
                 'speed_limit_mps': speed_limit_mps,
                 'acceleration_limit_mps2': acceleration_limit_mps2,
             }], condition=IfCondition(armed), output='screen'),
    ])
