from glob import glob
from setuptools import find_packages, setup

package_name = 'kobot_vehicle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/dashboard', glob('dashboard/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@kobot.local',
    description='Kobot vehicle devices and dashboard',
    license='MIT',
    entry_points={'console_scripts': [
        'ultrasonic_node = kobot_vehicle.ultrasonic_node:main',
        'imu_node = kobot_vehicle.imu_node:main',
        'status_node = kobot_vehicle.status_node:main',
        'dashboard_server = kobot_vehicle.dashboard_server:main',
        'motor_adapter = kobot_vehicle.motor_adapter:main',
        'figure_eight = kobot_vehicle.figure_eight:main',
        'lane_preview = kobot_vehicle.lane_preview_node:main',
        'lidar_avoidance = kobot_vehicle.LidarAvoidance:main',
        'lane_follow = kobot_vehicle.Lane_follow:main',
    ]},
)
