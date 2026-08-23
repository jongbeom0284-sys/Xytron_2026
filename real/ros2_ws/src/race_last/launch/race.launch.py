"""race_last 통합 런치 — 본선용 한 줄 실행.

    ros2 launch race_last race.launch.py                   # 인식 + 콘솔
    ros2 launch race_last race.launch.py dashboard:=true   # 웹 대시보드까지

이 명령을 친 터미널이 곧 상황판이다 (race_console 이 그린다).
모터 무장은 지금까지처럼 별도다: ~/arm_motor.sh, 주행 시작은 ~/yolop_arm.sh <속도>.

sensors.launch.py 에서 물려받은 것:
- 카메라 v4l2 보정 (usb_cam 이 장치에 안 쓰는 제어를 8초 뒤 직접 건다)
- PYTHONPATH (torch venv + ultralytics venv — 차선/차량 인식이 쓴다)
대시보드/영상 스트림/rosbridge 는 dashboard:=true 일 때만 뜬다.
주행 중에는 CPU 를 인식에 몰아주기 위해 기본 꺼짐이다.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def camera_control_command(config_dir):
    """usb_cam 이 장치에 쓰지 못하는 v4l2 제어를 직접 건다.

    sensors.launch.py 원본 그대로다. 값의 출처는 camera.yaml 하나로 유지한다.
    """
    with open(os.path.join(config_dir, 'camera.yaml'), encoding='utf-8') as handle:
        params = yaml.safe_load(handle)['/**']['ros__parameters']
    device = params.get('video_device', '/dev/videoCAM')

    ctrl_list = [
        'brightness=%d' % int(params.get('brightness', 0)),
        'auto_exposure=%s' % ('3' if params.get('autoexposure', False) else '1'),
        'exposure_time_absolute=%d' % int(params.get('exposure', 40)),
        'exposure_dynamic_framerate=0',
    ]
    auto_wb = params.get('auto_white_balance', True)
    if auto_wb:
        ctrl_list.append('white_balance_automatic=1')
    else:
        ctrl_list.append('white_balance_automatic=0')
        ctrl_list.append(
            'white_balance_temperature=%d' % int(params.get('white_balance', 2800)))

    return ['v4l2-ctl', '-d', device, '--set-ctrl', ','.join(ctrl_list)]


def generate_launch_description():
    share = get_package_share_directory('race_last')
    config = os.path.join(share, 'config')
    kv_config = os.path.join(
        get_package_share_directory('kobot_vehicle'), 'config')
    dashboard = LaunchConfiguration('dashboard')
    # 디버그 영상은 대시보드를 볼 때만 만든다. 안 보는 영상 인코딩은 낭비다.
    debug_images = ParameterValue(dashboard, value_type=bool)

    # ultralytics(차량 YOLO)는 clrernet venv 에만 있다. torch 경로와 함께 넣는다.
    ultra_site = '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages'
    torch_site = os.environ.get(
        'KOBOT_TORCH_SITE',
        '/home/xytron/.venvs/twinmixing/lib/python3.10/site-packages',
    )
    inherited = os.environ.get('PYTHONPATH', '')
    pythonpath = os.pathsep.join(
        [p for p in (torch_site, ultra_site, inherited) if p])

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', pythonpath),
        DeclareLaunchArgument(
            'dashboard', default_value='false',
            description='웹 대시보드 + 영상 스트림. 주행 중에는 끈다.'),

        # ---- 센서 ----
        Node(package='usb_cam', executable='usb_cam_node_exe', name='camera',
             parameters=[os.path.join(config, 'camera.yaml')], output='log'),
        Node(package='xycar_lidar', executable='xycar_lidar_node',
             name='xycar_lidar_node',
             parameters=[os.path.join(kv_config, 'lidar.yaml')], output='log'),
        Node(package='kobot_vehicle', executable='imu_node',
             parameters=[os.path.join(kv_config, 'imu.yaml')], output='log'),
        Node(package='kobot_vehicle', executable='status_node', output='log'),

        # ---- 인식 ----
        # 차선: YOLOPv2 TensorRT 전용 (폴백 없음 — 적재 실패는 기동 실패다)
        Node(package='race_last', executable='yolop_drive', name='yolop_drive',
             parameters=[os.path.join(config, 'yolop_drive.yaml')],
             output='log'),
        # 신호등: 정규화 색도 + 원형성. numpy + OpenCV 만 쓴다.
        Node(package='race_last', executable='traffic_perception',
             name='traffic_perception_node',
             # 0823 밤: 하단 0.34 -> 0.42 (마지막 바퀴에서 신호를 놓쳐 띠를 넓혔다).
             # 2026-08-22 연습장 실측: 램프가 원거리 y0.35 -> 근거리 y0.29/x0.54 로
             # 움직인다. 접근 전 구간을 덮는 띠 + 좌측 천장 주황등(x<0.35)·우측 창문
             # (x>0.64) 배제. 장소 바뀌면 param set 으로 즉석 조절 (대시보드 ROI 박스 참조).
             parameters=[{'debug_image': debug_images, 'max_hz': 20.0,
                          'roi_top': 0.28, 'roi_bottom': 0.42,
                          'roi_left': 0.36, 'roi_right': 0.64}],
             output='log'),
        # 차량: YOLOv8n(imgsz 640) + 라이다 퓨전. 좌/우와 거리만 낸다.
        Node(package='race_last', executable='vehicle_perception',
             name='vehicle_perception_node',
             parameters=[{'debug_image': debug_images, 'infer_interval': 3}],
             output='log'),
        # 라바콘: 차선 위 주황(첫 콘)이 잡히면 15초간 활성. 0823 합의.
        Node(package='race_last', executable='cone_perception',
             name='cone_perception',
             # 트리거는 기존값 그대로. 활성화 뒤 wall_aim만 실차에서 검증한
             # 차량 중심 기준(전방 LiDAR 보정 15cm)으로 계산한다.
             parameters=[{
                 'vehicle_length_m': 0.60,
                 'vehicle_width_m': 0.30,
                 'lidar_to_vehicle_center_m': 0.15,
                 'center_lookahead_m': 0.55,
                 'boundary_clearance_m': 0.02,
             }],
             output='log'),
        # 모터 텔레메트리 상시 기록 (~/motor_logs/*.csv) — 주행 진단용
        Node(package='race_last', executable='motor_logger',
             name='motor_logger', output='log'),
        # 주행 판단 전수 기록 (~/drive_logs/*.csv) — 사후 분석용
        Node(package='race_last', executable='drive_logger',
             name='drive_logger', output='log'),

        # ---- 콘솔 (이 터미널이 상황판이다) ----
        # 대시보드를 쓰면 콘솔은 중복이라 띄우지 않는다 (CPU 절약)
        Node(package='race_last', executable='race_console',
             name='race_console', output='screen', emulate_tty=True,
             condition=UnlessCondition(dashboard)),

        # ---- 대시보드 (dashboard:=true 일 때만) ----
        Node(package='web_video_server', executable='web_video_server',
             parameters=[{'port': 8080}], condition=IfCondition(dashboard),
             output='log'),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             parameters=[{'port': 9090}], condition=IfCondition(dashboard),
             output='log'),
        Node(package='kobot_vehicle', executable='dashboard_server',
             condition=IfCondition(dashboard), output='log'),

        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=camera_control_command(config), output='log')],
        ),
    ])
