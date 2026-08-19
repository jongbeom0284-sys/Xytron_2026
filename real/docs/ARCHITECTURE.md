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
                                       ├→ 1. 노란 중앙선 (색 확인 + 점선 외삽)
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

`motor_adapter` 는 속도 상한 `2.16 m/s`, 가속도 상한 `2.5 m/s²`,
명령 watchdog `0.5 s` 를 적용한다. 웹 WASD 는 누른 키 상태를 20 Hz 로 계속 발행하고,
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

### 차선 상태 JSON 주요 필드

| 필드 | 뜻 |
|---|---|
| `path_source` | `YELLOW_LINE` / `YELLOW_LINE_GHOST` / `CENTER_GAP` / `REGION_CENTER` / `NONE` |
| `mode` | `TRACKING` / `HOLD_LAST` / `WAITING` / `ERROR` |
| `drive_line` | `CENTER` / `SHIFT_LEFT` / `SHIFT_RIGHT` / `RETURNING` |
| `blocked_side` | `left` / `right` / `null` |
| `target_x_px` | 레이트 리밋을 거친 최종 목표 x |
| `center_offset_px` | 중앙선 자체의 화면 중앙 대비 오차 |
| `left_lane_width_px`, `right_lane_width_px` | 좌우 차선 평균 폭 |
| `narrow_side` | 카메라가 좁다고 본 쪽 |
| `lidar_left_m`, `lidar_front_m`, `lidar_right_m` | 코리도 안 최근접 전방 거리 |
| `ghost_frames` | 노란선 외삽 연속 프레임 수 |
| `maneuvers_enabled` | 회피 허용 여부 |
| `motor_output` | 항상 `false` (이 노드는 모터를 발행하지 않는다) |

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
