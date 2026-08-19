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
        "launch/fast_lane_track_test.launch.py",
        "launch/lane_perception.launch.py",
        "launch/track_test.launch.py",
        "launch/twinmixing_center_record_drive.launch.py",
        "launch/white_left_record_drive.launch.py",
    ]),
    ("share/" + PACKAGE_NAME + "/config", [
        "config/fast_lane_control.yaml",
        "config/fast_lane.yaml",
        "config/lane_drive.yaml",
        "config/twinmixing_center.yaml",
        "config/white_left_drive.yaml",
    ]),
]
data_files += install_tree("third_party/CLRerNet", "third_party/CLRerNet")
data_files += install_tree("third_party/simple_pid", "third_party/simple_pid")
data_files += install_tree("third_party/TwinMixing", "third_party/TwinMixing")
data_files += install_tree("patches", "patches")


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="xytron",
    maintainer_email="xytron@kobot.local",
    description="TwinMixing CUDA center-line follower with gated drive bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fast_lane_benchmark = kobot_lane_drive.fast_lane_benchmark:main",
            "fast_lane_controller = kobot_lane_drive.fast_lane_controller_node:main",
            "fast_lane_drive_bridge = kobot_lane_drive.fast_lane_drive_bridge_node:main",
            "fast_lane_perception = kobot_lane_drive.fast_lane_node:main",
            "lane_perception = kobot_lane_drive.lane_perception_node:main",
            "offline_replay = kobot_lane_drive.offline_replay:main",
            "drive_bridge = kobot_lane_drive.drive_bridge_node:main",
            "twinmixing_center_drive = kobot_lane_drive.twinmixing_center_node:main",
            "twinmixing_center_drive_bridge = kobot_lane_drive.twinmixing_center_drive_bridge_node:main",
            "white_left_drive = kobot_lane_drive.white_left_drive_node:main",
            "white_left_drive_bridge = kobot_lane_drive.white_left_drive_bridge_node:main",
            "recording_lane_viewer = kobot_lane_drive.recording_lane_viewer:main",
        ],
    },
)
