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
            # ── 시운전/본선 스위치 (2026-08-20 노출) ──────────────────
            # 기본값이 곧 본선 설정이다. 시운전에서만 바꾼다.
            # 이전에는 params.yaml 에도 없고 런치 인자도 없어서, 실차에서
            # 바꾸려면 `ros2 run ... -p` 로 런치를 우회해야 했다 — 규정 p.16
            # 이 요구하는 `ros2 launch` 실행과 어긋난다.
            DeclareLaunchArgument(
                "imu_enabled",
                default_value="true",
                description="IMU 사용. 하드웨어가 죽었으면 false "
                            "(요 적산 0 고정, 요 보충 랩·지름길 완료 판정 없음)",
            ),
            DeclareLaunchArgument(
                "maneuvers_enabled",
                default_value="true",
                description="전방 타겟 회피/추월. 본선 true (false 는 시운전 전용)",
            ),
            DeclareLaunchArgument(
                "shortcut_enabled",
                default_value="true",
                description="지름길 트리거. 본선 true",
            ),
            DeclareLaunchArgument(
                "manual_start",
                default_value="false",
                description="/kmu_race/go 수동 출발. 본선 금지(false)",
            ),
            DeclareLaunchArgument(
                "total_laps",
                default_value="3",
                description="완주 바퀴 수. 99 = 랩 무제한 시운전",
            ),
            DeclareLaunchArgument(
                "min_gantry_span_px",
                default_value="40",
                description="랩 마커 하한 [px] — 구조 검증된 하우징 가로폭 "
                            "(params.yaml 주석의 실측 근거 참조)",
            ),
            DeclareLaunchArgument(
                "cone_speed",
                default_value="18.0",
                description="라바콘 구간 목표속도 [자이카 단위]. 18 ≈ 1.14 m/s. "
                            "곡선감속·min_speed 바닥이 함께 걸리므로 실효는 "
                            "base×0.79 정도다 (speed_policy 주석의 환산표 참조)",
            ),
            DeclareLaunchArgument(
                "lane_speed",
                default_value="28.0",
                description="차선주행 목표속도 [자이카 단위]. 28 ≈ 2.24 m/s "
                            "(max_speed 와 같음 = 직선 상한)",
            ),
            DeclareLaunchArgument(
                "speed_scale",
                default_value="1.0",
                description="전 구간 속도 배율. 첫 확인 주행은 0.7 등으로 낮춘다",
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
                        "imu_enabled": LaunchConfiguration("imu_enabled"),
                        "maneuvers_enabled": LaunchConfiguration("maneuvers_enabled"),
                        "shortcut_enabled": LaunchConfiguration("shortcut_enabled"),
                        "manual_start": LaunchConfiguration("manual_start"),
                        "total_laps": LaunchConfiguration("total_laps"),
                        "min_gantry_span_px": LaunchConfiguration("min_gantry_span_px"),
                        "speed_scale": LaunchConfiguration("speed_scale"),
                        "cone_speed": LaunchConfiguration("cone_speed"),
                        "lane_speed": LaunchConfiguration("lane_speed"),
                    },
                ],
            ),
        ]
    )
