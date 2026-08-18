"""튜닝용 런치 — OpenCV 디버그 창 켜짐.

    ros2 launch kmu_race debug.launch.py

연습주행(07.01~08.21) 중 파라미터를 맞출 때만 쓴다.
대회 당일에는 race.launch.py 를 쓸 것 — 창을 띄우면 프레임이 밀린다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare("kmu_race"), "config", "params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=params_file),
            Node(
                package="kmu_race",
                executable="race",
                name="kmu_race",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"debug_gui": True, "control_hz": 30.0},
                ],
            ),
        ]
    )
