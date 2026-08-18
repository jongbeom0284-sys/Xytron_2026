"""대회 당일용 런치 — GUI 없음, 최대 성능.

규정 p.16: "실행 명령은 ros2 launch ... 가 되어야 함."

    ros2 launch kmu_race race.launch.py

심판이 노트북 화면을 봐야 하므로(p.16) OpenCV 창 대신 터미널 로그로
상태·랩·경과시간을 1초마다 찍는다. 창을 띄우면 프레임이 밀려 4분 예산(p.37)이
위태로워진다.
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
            DeclareLaunchArgument(
                "debug_gui",
                default_value="false",
                description="OpenCV 디버그 창. 대회 당일에는 반드시 false.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="튜닝 파라미터 YAML",
            ),
            # 스모크 테스트용: motor_topic:=/kmu_smoke_motor 로 주면 실모터와
            # 완전히 분리된 채 전체 스택을 돌려볼 수 있다 (차가 절대 안 움직임).
            # 대회 당일에는 이 인자를 주지 않는다 (기본 xycar_motor).
            DeclareLaunchArgument(
                "motor_topic",
                default_value="xycar_motor",
                description="모터 명령 토픽. 벤치 테스트 시 더미로 변경",
            ),
            Node(
                package="kmu_race",
                executable="race",
                name="kmu_race",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "debug_gui": LaunchConfiguration("debug_gui"),
                        "motor_topic": LaunchConfiguration("motor_topic"),
                    },
                ],
            ),
        ]
    )
