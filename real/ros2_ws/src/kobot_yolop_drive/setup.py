import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kobot_yolop_drive'

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
    description='YOLOPv2 lane segmentation drive.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolop_drive = kobot_yolop_drive.yolop_drive_node:main',
        ],
    },
)
