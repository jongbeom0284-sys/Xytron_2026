from glob import glob
import os

from setuptools import find_packages, setup


package_name = "lane_id_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jjong-haya",
    maintainer_email="jongha0315@gmail.com",
    description="Motor-free scanline lane ID perception for physical ROS2 vehicles.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lane_id_preview = lane_id_perception.perception_preview:main",
            "lane_id_offline_replay = lane_id_perception.lane_offline_replay:main",
        ],
    },
)
