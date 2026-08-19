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
| **`DRIVING_POLICY.md`** | **차가 어디를 보고 어떻게 달리나. 가장 먼저 읽을 문서** |
| `ARCHITECTURE.md` | 노드·토픽·데이터 흐름, 상태 JSON 필드 |
| `OPERATIONS.md` | 빌드·검증·테스트 주행 절차, 문제 해결 |
| `HARDWARE_AND_VESC.md` | 하드웨어 배선과 VESC 기준값 |
| `PROVENANCE.md` | 외부 원본과 우리가 고친 범위 |
| `RACE_STRATEGY.md` | 대회 전략과 성능 목표 |

### real/ros2_ws/src

| 패키지 | 소유 범위 | 출처 |
|---|---|---|
| **`kobot_lane_drive`** | **차선 인식과 주행 판단. 새 주행 로직은 전부 여기** | 자체 + TwinMixing |
| `kobot_vehicle` | 센서 노드, 모터 어댑터, 웹 대시보드 | 자체 |
| `vesc` | VESC 드라이버 | f1tenth 원본 + FW 6.5 패치 4파일 |
| `xycar_lidar` | LiDAR 노드 + YDLidar SDK | 외부 원본 |

---

## kobot_lane_drive 안쪽 — 파일별 역할

주행 로직의 전부다. 위 5개는 **ROS 에 의존하지 않는 순수 모듈**이라 차 없이
단위 시험으로 검증할 수 있고, 실제로 `test/` 에서 그렇게 한다.

| 파일 | 하는 일 |
|---|---|
| `geometry.py` | 레이트 리밋, 클램프, 저역통과. 여러 모듈이 공유하는 계산 헬퍼 |
| `twinmixing_center.py` | 마스크 → 중심 경로 (센터 갭 / 영역 중앙). 화면 오버레이 그리기 |
| `yellow_center.py` | 노란 중앙선 검출 (LAB top-hat), 사다리꼴 ROI, 점선 외삽 |
| `lane_policy.py` | 좌우 차선 폭 측정, 평상시 폭 학습, 주행 라인 상태기계 |
| `obstacle_lidar.py` | `/scan` → 차량 좌표계 코리도 거리 |
| `twinmixing_center_node.py` | ROS 노드. 위 모듈들을 엮어 목표선과 조향을 낸다. **모터 없음** |
| `twinmixing_center_drive_bridge_node.py` | **`/xycar_motor` 를 발행하는 유일한 파일.** `armed` 없이는 기동 안 함 |
| `drive_command.py` | 조향·속도 → `/xycar_motor` 형식 변환 |
| `recording_lane_viewer.py` | 포트 8001 녹화 재생 분석 뷰어 |

```text
kobot_lane_drive/
├── config/twinmixing_center.yaml    모든 튜닝값. 주석에 근거까지 적혀 있다
├── launch/twinmixing_center_record_drive.launch.py
├── third_party/TwinMixing/          모델 정의 + 가중치 large.pth
└── test/                            단위 시험 54개
```

### 시험이 지키는 계약

| 파일 | 무엇을 막나 |
|---|---|
| `test_motor_free.py` | 인식 계층에 모터 publisher 가 생기는 것 |
| `test_imports.py` | 모듈을 옮기다 import 가 끊기는 것 (실제로 한 번 났다) |
| `test_yellow_center.py` | 노란선 검출, 점선 외삽, 나무 거르기 |
| `test_lane_policy.py` | 차선 폭 측정, 상태 전이, 오검출 방지 |
| `test_obstacle_lidar.py` | 코리도 안팎 판정, 벽 배제, 낡은 스캔 거부 |
| `test_twinmixing_center.py` | 센터 갭·영역 중앙 폴백 |

---

## kobot_vehicle 안쪽

| 파일 | 하는 일 |
|---|---|
| `imu_node.py` | Razor AHRS → `/imu` |
| `ultrasonic_node.py` | 초음파 8채널 → `/xycar_ultrasonic` |
| `status_node.py` | 장치 상태 집계 → `/kobot/status` |
| `motor_adapter.py` | `/xycar_motor` → Ackermann. 속도·가속도 상한과 watchdog |
| `camera_control.py` | v4l2 카메라 파라미터 적용/조회 |
| `dashboard_server.py` | 포트 8000 웹 서버. 장치 관리, 모터 API, 기록 주행 상태기계 |

```text
kobot_vehicle/
├── dashboard/     index.html · app.js · style.css · drive.css
│                  camera_tuner.html · roslib.min.js
├── launch/        sensors.launch.py · motor.launch.py · teleop.launch.py
└── config/        camera · imu · lidar · ultrasonic · vesc yaml
```

`config/camera.yaml` 은 **카메라 설정의 유일한 출처**다. `sensors.launch.py` 가 이
파일을 읽어 `v4l2-ctl` 명령을 만든다 — `usb_cam` 이 이 값들을 장치에 쓰지 않기 때문이다.

---

## legacy/ — 이전 코드 보관소

빌드 경로 밖에 있다. 되살리려면 해당 폴더를 `real/ros2_ws/src` 로 옮기고
`setup.py` 의 entry point 를 되살려야 한다. 그냥 두면 아무 영향이 없다.

| 폴더 | 무엇이 들었나 | 왜 내렸나 |
|---|---|---|
| `lane-detectors/white-left/` | 흰 좌측 경계 RANSAC 검출기 | TwinMixing 으로 대체. 폴백 후보로 보존 |
| `lane-detectors/fast-lane/` | 보정 없는 빠른 차선 | 픽셀 출력만 내고 제어 입력이 아니었다 |
| `lane-detectors/clrernet/` | CLRerNet 노드·설정 일체 | 기준 사진 미완성, 실차 검증 0회 |
| `vehicle-nodes/` | 원본 `Lane_follow.py`, `LidarAvoidance.py`, `figure_eight.py` | 로직은 새 모듈로 이관, 원본은 참고용 |
| `simulation/` | Unity ROS-TCP 시뮬레이터 일체 | 실차와 메시지·토픽 규약이 다르다 |
| `evidence/lane-comparison/` | 차선 알고리즘 16종 비교 갤러리 | TwinMixing 선정 근거 |
| `ipm/` | ros-sports IPM 패키지 | README 가 미사용을 명시 |

### 새 코드가 물려받은 것

| 새 위치 | 물려받은 원본 |
|---|---|
| `yellow_center.py` 의 점선 외삽 | `legacy/vehicle-nodes/Lane_follow.py` |
| `obstacle_lidar.py` 의 각도 규약 | `legacy/vehicle-nodes/LidarAvoidance.py` |
| `geometry.py` 의 `move_towards` | `legacy/lane-detectors/white-left/` |
