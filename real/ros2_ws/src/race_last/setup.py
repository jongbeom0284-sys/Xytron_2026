import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'race_last'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kobot',
    maintainer_email='hhm282828@gmail.com',
    description='2026 본선 통합 주행 스택 — 차선 + 신호등 + 차량 + 라바콘 + 콘솔.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolop_drive = race_last.yolop_drive_node:main',
            'traffic_perception = race_last.Traffic_Perception:main',
            'vehicle_perception = race_last.Vehicle_Perception:main',
            'cone_perception = race_last.Cone_Perception:main',
            'race_console = race_last.race_console:main',
            'race_starter = race_last.race_starter:main',
            'motor_logger = race_last.motor_logger:main',
            'drive_logger = race_last.drive_logger:main',
            'frame_recorder = race_last.frame_recorder:main',
            'integrated_cone_test = race_last.integrated_cone_test_node:main',
        ],
    },
)
