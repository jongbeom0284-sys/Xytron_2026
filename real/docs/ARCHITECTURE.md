# 아키텍처와 공개 인터페이스

## 실행 경계

| launch | 무엇을 띄우나 | 모터 |
|---|---|---|
| `sensors.launch.py` | 카메라, 초음파, IMU, LiDAR, 상태 수집, **twinmixing_center_drive**, web_video_server, rosbridge, dashboard | ❌ 없음 |
| `motor.launch.py` | VESC 드라이버, Ackermann 변환기, motor_adapter | `armed:=true` 일 때만 |
| `teleop.launch.py` | 위 둘을 조합한 상위 주행 대기 launch | `armed:=true` 일 때만 |
| `twinmixing_center_record_drive.launch.py` | 기록 주행용. 인식 재사용 + 승인 잠금 bridge | `armed:=true` 일 때만 |

Kobot 은 부팅 시 `kobot-sensors.service` 가 `sensors.launch.py` 를 실행한다.
이 서비스에는 모터가 없다.

## 데이터 흐름

```text
/dev/videoCAM → usb_cam → /image_raw ─┬→ web_video_server → 웹 카메라 화면
                                      ├→ twinmixing_center_drive
                                      └→ dashboard_server → 녹화 API
/dev/ttyLIDAR → xycar_lidar → /scan ──┬→ twinmixing_center_drive (옆 차선 코리도)
                                      └→ dashboard LiDAR 화면
/dev/ttyIMU   → imu_node → /imu → dashboard IMU 화면
/dev/ttySonic → ultrasonic_node → /xycar_ultrasonic → dashboard 초음파 화면
status_node → /kobot/status → dashboard 장치 상태
```

## 차선 인식과 주행 판단

```text
/image_raw ─┐
            ├→ TwinMixing CUDA → driving_mask + lane_mask
/scan ──────┘                          │
                                       ├→ 1. 노란 중앙선 (색 확인, 끊기면 1초 유지)
                                       ├→ 2. 센터 갭
                                       ├→ 3. 영역 중앙
                                       │      ↓ 목표선
                                       ├→ 좌우 차선 폭 측정
                                       ├→ 옆 차선 라이다 코리도
                                       │      ↓
                                       └→ 주행 라인 상태기계
                                              ↓ 목표 x (레이트 리밋)
                                          조향 preview
   ├→ /kobot_lane_drive/twinmixing_center/debug_image   (웹 화면)
   ├→ /kobot_lane_drive/twinmixing_center/status        (상태 JSON)
   └→ /kobot_lane_drive/twinmixing_center/drive_preview
          → 승인 잠금 bridge (armed) → /xycar_motor
```

인식 노드는 모터 publisher 를 갖지 않는다. 규칙 자체는
[DRIVING_POLICY.md](DRIVING_POLICY.md) 에 있다.

## 모터 명령 흐름

```text
웹 WASD·시간 주행·기록 주행 bridge
  → /xycar_motor (std_msgs/Float32MultiArray: [steering, speed-command])
  → motor_adapter
  → /ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
  → ackermann_to_vesc_node
  ├→ /commands/motor/speed  (ERPM)
  └→ /commands/servo/position
  → vesc_driver_node → /dev/ttyMOTOR
```

`motor_adapter` 는 속도 상한 `4.87 m/s`, 명령 watchdog `0.5 s` 를 적용한다.
가속도 상한은 소스 기본값이 `4.0 m/s²` 이나 빌드 미반영이라 실행 중인 노드는 `2.5 m/s²` 다.
속도 상한은 설정값이며 실측치는 이에 미달한다 (`HARDWARE_AND_VESC.md` 참조). 웹 WASD 는 누른 키 상태를 20 Hz 로 계속 발행하고,
정지는 0 명령을 즉시 발행한다.

## 공개 인터페이스

| 구분 | 이름 | 타입·역할 |
|---|---|---|
| 센서 | `/image_raw` | `sensor_msgs/Image` 카메라 원본 |
| 센서 | `/scan` | `sensor_msgs/LaserScan` LiDAR |
| 센서 | `/imu` | `sensor_msgs/Imu` |
| 센서 | `/xycar_ultrasonic` | `std_msgs/Int32MultiArray` |
| 차선 화면 | `/kobot_lane_drive/twinmixing_center/debug_image` | 오버레이 영상 |
| 차선 상태 | `/kobot_lane_drive/twinmixing_center/status` | 아래 표의 JSON |
| 주행 미리보기 | `/kobot_lane_drive/twinmixing_center/drive_preview` | `[steering_rad, speed_mps, confidence]` |
| 기록 주행 enable | `/kobot_lane_drive/record_drive/enabled` | transient-local `Bool` |
| 장치 상태 | `/kobot/status` | JSON |
| 차량 입력 | `/xycar_motor` | `[steering, speed-command]` |
| 변환 | `/ackermann_cmd` | Ackermann 명령 |
| VESC 출력 | `/commands/motor/speed`, `/commands/servo/position` | ERPM, servo position |

### 차선 상태 JSON 필드

**무엇을 봤나**

| 필드 | 뜻 |
|---|---|
| `path_source` | `YELLOW_LINE` / `YELLOW_LINE_GHOST` / `CENTER_GAP` / `REGION_CENTER` / `NONE` |
| `state`, `mode` | `TRACKING` / `HOLD_LAST` / `WAITING` / `ERROR` |
| `confidence` | 목표선 신뢰도 0~1 |
| `center_offset_px` | 목표선의 화면 중앙 대비 오차 |
| `lookahead_x_px`, `lookahead_y_px` | 조향 기준점. 오차는 **이 점**에서 잰다 |
| `heading_deg` | 목표선의 화면상 방향 |
| `center_gap_width_px` | 두 주행영역 사이 틈의 평균 폭 |
| `lane_support_ratio` | 목표선이 `lane_mask` 위에 놓인 비율 |
| `path_points` | 피팅에 쓰인 행 수 |
| `ghost_frames` | 노란선을 유지한 연속 프레임 수. 0 이면 지금 실제로 보고 있다 |
| `ghost_seconds` | 마지막 실측 노란선 이후 흐른 시간. 1.0 을 넘기면 도로로 내려간다 |

**어떻게 판단했나**

| 필드 | 뜻 |
|---|---|
| `drive_line` | `CENTER` / `SHIFT_LEFT` / `SHIFT_RIGHT` / `RETURNING` |
| `blocked_side` | `left` / `right` / `null` |
| `target_x_px` | 레이트 리밋을 거친 최종 목표 x |
| `speed_scale` | 상태별 속도 배율 (`SHIFT`·`RETURNING` 은 0.75) |
| `left_lane_width_px`, `right_lane_width_px` | 좌우 차선 평균 폭 |
| `narrow_side` | 카메라가 좁다고 본 쪽 |
| `lidar_front_m`, `lidar_left_m`, `lidar_right_m` | 코리도 안 최근접 거리 |
| `maneuvers_enabled` | 회피 허용 여부 |

**얼마나 꺾었나** — 조향이 왜 그 값이 됐는지 되짚을 때 쓰는 필드다.
텔레메트리 CSV 에도 같은 이름으로 기록된다.

| 필드 | 뜻 |
|---|---|
| `cross_error_px` | 제어에 실제로 들어간 가로 오차 (P 항의 입력) |
| `error_rate` | 그 오차의 시간 변화율 (D 항의 입력) |
| `steering_deadband_px` | 이 값 안쪽 오차는 0 으로 죽인다 |
| `lane_tangent_deg` | 화면상 차선 기울기. **표시·진단 전용, 제어에 안 쓴다** |
| `steering_rad` | 최종 조향 |
| `speed_mps` | 최종 속도 |

**얼마나 빨랐나**

| 필드 | 뜻 |
|---|---|
| `inference_ms`, `inference_fps` | 모델 추론 시간과 실효 주기 |
| `device`, `runtime` | `cuda` / `pytorch_cuda_fp16` |
| `motor_output` | 항상 `false` (이 노드는 모터를 발행하지 않는다) |
| `reason` | 사람이 읽는 한 줄 사유 |

## HTTP API

dashboard server(:8000)가 정적 화면과 다음 API 를 제공한다.

- `GET /api/devices` — 장치 상태, 장치별 start/stop
- `GET|POST /api/recording[/start|/stop]` — 카메라 프레임 저장
- `GET /api/motor`, `POST /api/motor/{arm,restart,test,stop}`
- `GET /api/record-drive`, `POST /api/record-drive/{start,pause,resume,finish,heartbeat}`
  - `start` 본문: `{speed_mps, maneuvers_enabled}`
- `GET /api/recovery`, `POST /api/recovery/component/<name>`, `POST /api/recovery/group/{media,all}`

WASD 무장·시간 주행·기록 주행 시작 전에 ROS 그래프의 `/xycar_motor` publisher 를
확인하고 `dashboard_server` 이외의 publisher 가 있으면 시작을 거부한다.

인증·권한 검사가 없다. 신뢰된 로컬망이나 Tailscale 안에서만 쓴다.

## 별도 도구

| 도구 | 포트 | 역할 |
|---|---|---|
| `recording_lane_viewer` | 8001 | 녹화 세션을 현재 인식 파이프라인으로 다시 돌려 원본과 오버레이를 나란히 재생. `--prepare-only` 로 배치 분석만 할 수도 있다 |
| `camera_tuner.html` | 8000 | 카메라 노출·화이트밸런스 웹 튜너 |
