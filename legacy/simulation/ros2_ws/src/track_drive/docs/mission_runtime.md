# Track Drive Mission Runtime

이 문서는 Ubuntu 22.04 + ROS2 Humble 기준 실행 구조를 정리한다.

## 현재 결론

- 현재 작업 브랜치: `feat/scanline-lane-id-tracking`
- 시뮬레이터 workspace: `C:\Users\yaja0\Downloads\2026_Xytron\simulation\ros2_ws`
- Unity용 ROS2 패키지는 `simulation/ros2_ws/src`에만 둔다.
- `newAuto`는 더 이상 실행 source 경로로 쓰지 않아도 된다.
- `newAuto`의 기존 주행 코드와 `origin/codex/track-drive-start-signal-gate`는 같은 기준 코드였고, 현재 브랜치는 그 위에 라바콘 통합을 얹은 버전이다.
- 현재 확인 시점에는 ROS graph에 `/scan`, `/usb_cam/image_raw/front`, `/imu`가 없었다. 즉 simulator/ROS-TCP endpoint live 연결은 아직 확인되지 않았다.
- 실제 주행 시작은 하지 않았다.

## 폴더 구조

```text
C:\Users\yaja0\Downloads\2026_Xytron
├─ simulation\
│  ├─ ros2_ws\src\
│  │  ├─ xycar_msgs\                    # Unity XycarMotor 메시지
│  │  ├─ ROS-TCP-Endpoint-main-ros2\    # Unity endpoint
│  │  ├─ kookmin9_viewer\
│  │  └─ track_drive\
│  │     ├─ launch\
│  │     │  └─ track_drive.launch.py
│  │     ├─ docs\
│  │     │  └─ mission_runtime.md
│  │     ├─ package.xml
│  │     ├─ setup.py
│  │     └─ track_drive\
│  ├─ assets\
│  └─ legacy\
└─ real\
   ├─ lane_id_perception\
   └─ drivers\
```

## 추종해야 하는 코드 기준

시뮬레이터 코드는 `simulation/ros2_ws/src`만 기준으로 삼는다.

`track_drive`는 현재 브랜치의 것을 따라야 한다. `newAuto`의 `track_drive`를 다시 복사하면 라바콘 통합이 사라질 수 있으므로 덮어쓰면 안 된다.

## 빌드

WSL Ubuntu-22.04 터미널 기준:

```bash
source /opt/ros/humble/setup.bash

cd /mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws
colcon build
```

## Source 순서

이제 실행 터미널마다 `2026_Xytron` workspace 하나만 source하면 된다.

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws/install/setup.bash
```

확인 명령:

```bash
ros2 pkg prefix xycar_msgs
ros2 pkg prefix ros_tcp_endpoint
ros2 pkg prefix track_drive
python3 -c "import track_drive.track_drive as t; print(t.__file__)"
```

기대값:

- `xycar_msgs`: `/mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws/install/xycar_msgs`
- `ros_tcp_endpoint`: `/mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws/install/ros_tcp_endpoint`
- `track_drive`: `/mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws/install/track_drive`

## Simulator 연결 확인

주행 노드를 켜기 전에 endpoint와 simulator topic부터 확인한다.

터미널 1:

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws/install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

Unity simulator를 실행하고 ROS-TCP endpoint에 연결한다.

터미널 2:

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/yaja0/Downloads/2026_Xytron/simulation/ros2_ws/install/setup.bash

ros2 topic list
ros2 topic hz /scan
ros2 topic hz /usb_cam/image_raw/front
```

필수 입력 topic:

- `/usb_cam/image_raw/front`: 신호등/차선/라바콘 종료 판단용 카메라
- `/scan`: 라바콘 주행 및 라이다 기반 거리 판단
- `/imu`: yaw/IMU 보조 정보

출력 topic:

- `xycar_motor`: 실제 조향/속도 명령
- `/track_drive/debug`: 상태 JSON
- `/track_drive/start_signal_debug_image`
- `/track_drive/cone_debug_image`
- `/track_drive/lane_debug_image`
- `/track_drive/lane_bev_image`
- `/track_drive/lane_mask_image`

## rqt

연결 확인용:

```bash
rqt_graph
rqt_image_view
```

추천 이미지 topic:

- `/track_drive/start_signal_debug_image`
- `/track_drive/cone_debug_image`
- `/track_drive/lane_debug_image`

라바콘 라이다 디버그는 모터 명령을 publish하지 않는다.

```bash
ros2 run track_drive cone_debug
```

## 실제 mission launch

아래 명령은 `xycar_motor`를 publish하므로 실제 주행 시작 명령이다.

```bash
ros2 launch track_drive track_drive.launch.py \
  enable_start_signal_gate:=true \
  enable_cone_mode:=true \
  cone_start_active:=true
```

기본 `track_drive.launch.py`는 안전을 위해 `enable_start_signal_gate=false`, `enable_cone_mode=false`가 기본값이다. 과제 흐름으로 달릴 때만 위처럼 명시적으로 켠다.

## 상태 흐름

```text
시작
└─ start signal gate
   ├─ 빨강/노랑/미검출: speed=0, angle=0
   └─ 초록: gate release
      └─ cone mode
         ├─ /scan 기반 라바콘 좌우/left-wall/path target 계산
         ├─ cone_follower가 angle/speed 생성
         └─ 종료 조건 만족
            └─ lane driving
               ├─ lane_detector
               └─ lane_controller
```

## 라바콘에서 차선 추종으로 넘어가는 트리거

`cone_follower.py`의 종료 조건은 다음을 모두 만족해야 한다.

1. 라바콘 추적 이력이 `cone_min_tracked_frames` 이상이다. 기본값은 `80`.
2. 라바콘이 사라진 상태가 `cone_hold_max_frames`보다 길다. 기본값은 `25`.
3. 카메라 하단 ROI에서 노란 차선 픽셀이 `cone_yellow_pixel_threshold` 이상 보인다. 기본값은 `2500`.
4. 위 상태가 `cone_clear_frames_required` 프레임 유지된다. 기본값은 `25`.

종료되면 `track_drive.py`에서 `cone_finished=True`, `cone_mode_active=False`로 바꾸고, `cone_finish_to_lane=True`이므로 바로 차선 추종 루프로 넘어간다.

## 시작 신호 인식

시작 신호는 `traffic_light_detector.py`를 그대로 사용한다. `enable_start_signal_gate=true`일 때만 활성화된다.

- 초록 감지 전: `xycar_motor`에 정지 명령
- 초록 감지 후: `start_signal_released=True`
- `start_signal_release_latched=True`라서 한 번 초록으로 열린 뒤에는 다시 신호 대기로 돌아가지 않는다.

## 현재 확인 상태

확인 완료:

- Ubuntu-22.04 / ROS2 Humble 환경에서 `ros2`, `rqt`, `colcon` 사용 가능
- 기존 통합 workspace에서 패키지 인식과 `colcon build`가 완료된 이력이 있음
- 새 구조에서는 Unity 패키지 4개를 `simulation/ros2_ws`로 이동함
- 새 경로에서의 전체 `colcon build`는 이 브랜치 검증 단계에서 다시 확인함

미확인:

- simulator가 현재 ROS graph에 붙어 있지 않아 `/scan`, `/usb_cam/image_raw/front`, `/imu` live topic은 확인되지 않음
- 실제 `xycar_motor` 주행 publish는 실행하지 않음
