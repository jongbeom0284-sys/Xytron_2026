# 출처와 변경 이력

## 기준 스냅샷

이 기준판은 2026-08-13 `kobot` 의 `/home/xytron/xycar_ws/src` 에서 실행·검토한
소스에서 출발했다. import 시 `.git`, `build`, `install`, `log`, Python/pytest cache,
`*.bak`, 임시 출력은 제외했다.

> **현재 기준 (2026-08-22 갱신)**: 위 서술은 2026-08-13 시점의 이력이다.
> 현재 실차에서 실행되는 기준 워크스페이스는 `/home/xytron/2026_Xytron/real/ros2_ws` 이며,
> `xycar_ws` 에서는 최종 코드를 개발하지 않는다. `xycar_ws/src/kmu_race` 에만 있던
> 2026-08-21 콘 주행 작업분은 `2026_Xytron` 으로 편입했다 (커밋 `da08996`).

| 모듈 | 출처 | 기준 |
|---|---|---|
| `kobot_vehicle` | Kobot 차량용 자체 작성 ROS 2 패키지 | 2026-08-13 Jetson 실행본 |
| `kobot_lane_drive` | Kobot 자체 + TwinMixing | 아래 표 |
| `vesc` | https://github.com/f1tenth/vesc | `153998df8545fe1781b975df88e411b4e71d4bfe` + 4파일 패치 |
| `xycar_lidar/YDLidar-SDK` | https://github.com/YDLIDAR/YDLidar-SDK | `16ca1f402453d1c094ada83e94200951ed2f711b` + checksum 로그 억제 1줄 |
| `xycar_lidar` ROS wrapper | Kobot 에서 사용 중인 ROS 2 wrapper | 2026-08-13 Jetson 실행본 |

## kobot_lane_drive

| 역할 | Upstream | 라이선스 | 포함 방식 |
|---|---|---|---|
| 주행영역·차선 세그멘테이션 | TwinMixing | — | `third_party/TwinMixing` (모델 정의 + `pretrained/large.pth`) |

프로젝트 자체 코드는 다음이다. 전부 `kobot_lane_drive/kobot_lane_drive/` 안에 있다.

- `yellow_center.py` — 노란 중앙선 검출과 점선 외삽.
  점선 외삽(GHOST/PREDICT)의 발상은 `legacy/vehicle-nodes/Lane_follow.py` 에서 가져왔다.
- `lane_policy.py` — 좌우 차선 폭 측정, 평상시 폭 기준 학습, 주행 라인 상태기계
- `obstacle_lidar.py` — LaserScan 을 차량 좌표계 코리도로 요약.
  각도 보정(+pi, 좌측 양수)은 `legacy/vehicle-nodes/LidarAvoidance.py` 에서 확인한 규약이다.
- `twinmixing_center.py` — 마스크에서 중심 경로 추출, 화면 오버레이
- `geometry.py` — 레이트 리밋·클램프·저역통과.
  `move_towards` 는 `legacy/lane-detectors/white-left/white_left_drive_node.py` 에서 옮겨 왔다.
- `twinmixing_center_node.py` — ROS 인터페이스와 안전 차단
- `twinmixing_center_drive_bridge_node.py` — 승인 잠금 모터 형식 adapter
- `recording_lane_viewer.py` — 녹화 재생 분석 뷰어

## kobot_vehicle

센서 launch 와 dashboard, 웹 WASD heartbeat, 시간 주행, motor API, 속도·가속도 제한과
watchdog 은 차량 프로젝트 자체 코드다. `dashboard/roslib.min.js` 만 외부 원본이다.

`dashboard/app.js` 의 최대속도 표시, input `max`, client validation 은 실제 상한
`4.87 m/s` 와 일치한다.

## vesc

upstream 기준에서 바뀐 파일은 네 개뿐이다.

- `vesc_driver/include/vesc_driver/vesc_packet.hpp`
- `vesc_driver/src/vesc_driver.cpp`
- `vesc_driver/src/vesc_interface.cpp`
- `vesc_driver/src/vesc_packet.cpp`

변경 목적은 Kobot VESC FW 6.5 의 packet/firmware 응답 호환이다.
나머지 파일은 vendored upstream 이다.

## xycar_lidar

YDLidar SDK 기준 commit 에서 의미 있는 내용 변경은
`YDLidar-SDK/src/YDlidarDriver.cpp` 의 반복 checksum 오류 로그 1줄 억제다.

## legacy 로 내려간 것

2026-08-19 저장소를 `real/` 과 `legacy/` 두 축으로 재편했다.
아래는 실차 실행 경로에서 빠졌지만 이력과 함께 보존된 것이다.

| 대상 | 출처 | 지금 위치 |
|---|---|---|
| CLRerNet | https://github.com/hirotomusiker/CLRerNet `dae038f6…` (Apache-2.0) | `legacy/lane-detectors/clrernet/third_party_CLRerNet` |
| simple-pid | https://github.com/m-lundberg/simple-pid v2.0.1 `6e6f4b00…` (MIT) | `legacy/lane-detectors/clrernet/` |
| IPM | https://github.com/ros-sports/ipm Humble `29439254…` | `legacy/ipm` |
| ROS-TCP Endpoint | Unity | `legacy/simulation/ros2_ws/src/ROS-TCP-Endpoint-main-ros2` |
| DonkeyCar / Waveshare | 목표 픽셀 PID·두 샘플 행 구조 참고 (코드 미복사) | `legacy/lane-detectors/clrernet` 설계 근거 |

CLRerNet 의 CUDA NMS 두 소스에는 Jetson PyTorch 2.8 API 호환 패치만 적용했다.
패치는 `legacy/lane-detectors/clrernet/patches/torch28_nms.patch` 에 보존한다.
가중치 `.pth` 는 `.gitignore` 대상이라 저장소에 없다. 원본은 `~/clrernet_eval` 에 있다.

## 장치 설정 자료

`real/config/vesc_tool/` XML 은 2026-08-13 VESC Tool readback 이다.

- motor: FW 6.5, speed-controller `Ki=0.0015`
- app: servo output enabled

XML 은 참고·비교용이고 자동 적용되지 않는다. 설정을 다시 변경하면 날짜,
장치 HW/FW, 변경 전후 readback, fault 상태를 이 문서에 추가한다.
