from glob import glob

from setuptools import find_packages, setup

package_name = "kmu_race"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kmu-race team",
    maintainer_email="hhm282828@gmail.com",
    description="2026 국민대 자율주행 경진대회 본선 주행 스택",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "race = kmu_race.nodes.race_node:main",
            "preflight = kmu_race.tools.preflight:main",
            "replay = kmu_race.tools.replay:main",
            "lanesim = kmu_race.tools.lanesim:main",
            "lanebench = kmu_race.tools.lanebench:main",
            "exposure_scan = kmu_race.tools.exposure_scan:main",
            "teleop = kmu_race.tools.teleop:main",
        ],
    },
)
