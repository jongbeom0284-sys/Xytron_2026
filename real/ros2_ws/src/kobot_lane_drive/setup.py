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
    (
        "share/" + PACKAGE_NAME + "/launch",
        [
            "launch/twinmixing_center_record_drive.launch.py",
            "launch/cone_corridor_drive.launch.py",
        ],
    ),
    (
        "share/" + PACKAGE_NAME + "/config",
        [
            "config/twinmixing_center.yaml",
            "config/cone_corridor_robust.yaml",
        ],
    ),
]
data_files += install_tree("third_party/TwinMixing", "third_party/TwinMixing")


setup(
    name=PACKAGE_NAME,
    version="0.3.0",
    packages=find_packages(),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="xytron",
    maintainer_email="xytron@kobot.local",
    description="TwinMixing yellow-center lane driving with gated drive bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "twinmixing_center_drive = kobot_lane_drive.twinmixing_center_node:main",
            "twinmixing_center_drive_bridge = kobot_lane_drive.twinmixing_center_drive_bridge_node:main",
            "cone_corridor_drive = kobot_lane_drive.cone_corridor_drive_node:main",
            "recording_lane_viewer = kobot_lane_drive.recording_lane_viewer:main",
        ],
    },
)
