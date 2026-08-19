# 폴더별 소유물

저장소에서 무엇이 어디에 있는지 한 곳에 적는다. 새 코드를 어디에 둘지 헷갈리면
여기를 먼저 본다.

## 최상위

| 폴더 | 무엇이 들었나 | 빌드되나 |
|---|---|---|
| `real/` | 실차에서 실제로 도는 코드 전부 | ✅ |
| `legacy/` | 이전 시도 보관소 | ❌ |

새 주행 코드는 **항상** `real/ros2_ws/src` 안에 만든다. `legacy/` 는 읽기 전용
보관소이며 여기 있는 파일을 고쳐 쓸 일은 없다.

---

## real/ — 실차 코드

```text
real/
├── README.md            빠른 시작과 주행 절차 요약
├── AGENTS.md            작업 규칙 (모터 안전, 빌드 규칙)
├── docs/                아래 표 참고
├── ros2_ws/src/         ROS 2 패키지 4개
├── config/vesc_tool/    VESC Tool readback XML (참고용, 자동 적용 금지)
├── scripts/             Jetson 빌드·사전점검 스크립트
└── system/              udev · systemd · sudoers · env 설치 예제
```

### real/docs

| 파일 | 무엇을 답하나 |
|---|---|
| `FOLDERS.md` | 이 문서. 무엇이 어디에 있나 |
| `DRIVING_POLICY.md` | 차가 어떤 규칙으로 달리나 |
| `ARCHITECTURE.md` | 노드·토픽·데이터 흐름 |
| `OPERATIONS.md` | 빌드·검증·테스트 주행 절차 |
| `HARDWARE_AND_VESC.md` | 하드웨어 배선과 VESC 기준값 |
| `PROVENANCE.md` | 외부 원본과 우리가 고친 범위 |

### real/ros2_ws/src

| 패키지 | 소유 범위 | 출처 |
|---|---|---|
| `kobot_lane_drive` | **차선 인식과 주행 판단.** 새 주행 로직은 전부 여기 | 자체 + TwinMixing |
| `kobot_vehicle` | 센서 노드, 모터 어댑터, 웹 대시보드 | 자체 |
| `vesc` | VESC 드라이버 | f1tenth 원본 + FW 6.5 패치 4파일 |
| `xycar_lidar` | LiDAR 노드 + YDLidar SDK | 외부 원본 |

#### kobot_lane_drive 안쪽

```text
kobot_lane_drive/kobot_lane_drive/
├── geometry.py                         공용 계산 (레이트 리밋, 클램프, 저역통과)
├── twinmixing_center.py                마스크 → 중심 경로. 화면 오버레이
├── yellow_center.py                    노란 중앙선 검출 + 점선 외삽
├── lane_policy.py                      차선 폭 측정 + 주행 라인 상태기계
├── obstacle_lidar.py                   /scan → 옆 차선 코리도 거리
├── twinmixing_center_node.py           ROS 노드. 인식 + 판단 (모터 없음)
├── twinmixing_center_drive_bridge_node.py   승인 잠금 모터 브리지 (유일한 모터 출력)
└── recording_lane_viewer.py            :8001 녹화 재생 분석 뷰어
```

윗줄 다섯 파일은 ROS 에 의존하지 않는 순수 모듈이다. 차 없이 단위 시험으로
검증할 수 있고, 실제로 `test/` 에서 그렇게 검증한다.

```text
kobot_lane_drive/
├── config/twinmixing_center.yaml       모든 튜닝값. 조명이 바뀌면 여기만 고친다
├── launch/twinmixing_center_record_drive.launch.py
├── third_party/TwinMixing/             모델 정의 + 가중치 large.pth
└── test/                               단위 시험 37개
```

#### kobot_vehicle 안쪽

```text
kobot_vehicle/kobot_vehicle/
├── imu_node.py           Razor AHRS → /imu
├── ultrasonic_node.py    초음파 8채널 → /xycar_ultrasonic
├── status_node.py        장치 상태 집계 → /kobot/status
├── motor_adapter.py      /xycar_motor → Ackermann. 속도·가속도 상한과 watchdog
├── camera_control.py     v4l2 카메라 파라미터
└── dashboard_server.py   :8000 웹 서버. 장치 관리·모터 API·기록 주행 상태기계

kobot_vehicle/dashboard/    index.html · app.js · style.css · drive.css ·
                            camera_tuner.html · roslib.min.js
kobot_vehicle/launch/       sensors · motor · teleop
kobot_vehicle/config/       camera · imu · lidar · ultrasonic · vesc yaml
```

---

## legacy/ — 이전 코드 보관소

빌드 경로 밖에 있다. 되살리려면 해당 폴더를 `real/ros2_ws/src` 로 옮기고
`setup.py` 의 entry point 를 되살려야 한다. 그냥 두면 아무 영향이 없다.

| 폴더 | 무엇이 들었나 | 왜 내렸나 |
|---|---|---|
| `lane-detectors/white-left/` | 흰 좌측 경계 RANSAC 검출기 6파일 | TwinMixing 으로 대체. 폴백 후보로 보존 |
| `lane-detectors/fast-lane/` | 보정 없는 빠른 차선 8파일 | 픽셀 출력만 내고 제어 입력이 아니었다 |
| `lane-detectors/clrernet/` | CLRerNet 노드·가중치·설정 일체 | 기준 사진 미완성, 실차 검증 0회 |
| `vehicle-nodes/` | 원본 `Lane_follow.py`, `LidarAvoidance.py`, `figure_eight.py` | 로직은 새 모듈로 이관, 원본은 참고용 보존 |
| `simulation/` | Unity ROS-TCP 시뮬레이터 일체 | 실차와 메시지·토픽 규약이 다르다 |
| `evidence/lane-comparison/` | 차선 알고리즘 16종 비교 갤러리 | TwinMixing 을 왜 골랐는지에 대한 근거 |
| `ipm/` | ros-sports IPM 패키지 | README 가 미사용을 명시했다 |

### 새 코드가 물려받은 것

| 새 위치 | 물려받은 원본 |
|---|---|
| `yellow_center.py` 의 GHOST/PREDICT 외삽 | `legacy/vehicle-nodes/Lane_follow.py` |
| `obstacle_lidar.py` 의 각도 보정(+pi, 좌측 양수) | `legacy/vehicle-nodes/LidarAvoidance.py` |
| `geometry.py` 의 `move_towards` | `legacy/lane-detectors/white-left/white_left_drive_node.py` |
