from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "kobot_lane_drive"


def install_tree(source: str, destination: str) -> list[tuple[str, list[str]]]:
    root = Path(source)
    groups: dict[str, list[str]] = {}
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        relative_parent = path.relative_to(root).parent
        target = str(Path("share") / PACKAGE_NAME / destination / relative_parent)
        groups.setdefault(target, []).append(str(path))
    return sorted(groups.items())


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
    ("share/" + PACKAGE_NAME, ["package.xml", "README.md", "PROVENANCE.md"]),
    ("share/" + PACKAGE_NAME + "/launch", [
        "launch/fast_lane_perception.launch.py",
        "launch/lane_perception.launch.py",
        "launch/track_test.launch.py",
    ]),
    ("share/" + PACKAGE_NAME + "/config", [
        "config/fast_lane.yaml",
        "config/lane_drive.yaml",
    ]),
]
data_files += install_tree("third_party/CLRerNet", "third_party/CLRerNet")
data_files += install_tree("third_party/PythonRobotics", "third_party/PythonRobotics")
data_files += install_tree("patches", "patches")


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="xytron",
    maintainer_email="xytron@kobot.local",
    description="Open-source based CLRerNet left-lane metric path and gated drive stack",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fast_lane_benchmark = kobot_lane_drive.fast_lane_benchmark:main",
            "fast_lane_perception = kobot_lane_drive.fast_lane_node:main",
            "lane_perception = kobot_lane_drive.lane_perception_node:main",
            "offline_replay = kobot_lane_drive.offline_replay:main",
            "drive_bridge = kobot_lane_drive.drive_bridge_node:main",
        ],
    },
)
