from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'kobot_vehicle'


def install_dashboard_tree():
    groups = {}
    root = Path('dashboard')
    for path in root.rglob('*'):
        if not path.is_file():
            continue
        relative_parent = path.relative_to(root).parent
        target = str(Path('share') / package_name / 'dashboard' / relative_parent)
        groups.setdefault(target, []).append(str(path))
    return sorted(groups.items())

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    package_data={'kobot_vehicle': ['*.pt']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ] + install_dashboard_tree(),
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
        'lane_follow = kobot_vehicle.Lane_follow:main',
        'lidar_avoidance = kobot_vehicle.Lidar_Avoidance:main',
        'traffic_perception = kobot_vehicle.Traffic_Perception:main',
        'vehicle_perception = kobot_vehicle.Vehicle_Perception:main',
        'yolop_drive = kobot_vehicle.yolop_drive_node:main',
        'object_detection = kobot_vehicle.object_detection_node:main',
    ]},
)
