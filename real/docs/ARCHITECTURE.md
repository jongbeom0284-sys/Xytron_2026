# 아키텍처와 공개 인터페이스

## 실행 경계

`sensors.launch.py`는 카메라, 초음파, IMU, LiDAR, 상태 수집, 무모터 `white_left_drive`, web video server, rosbridge, dashboard server를 실행한다. VESC, 모터 변환기, `white_left_drive_bridge`는 실행하지 않는다.

`motor.launch.py`는 `armed:=true`일 때만 VESC 드라이버, Ackermann 변환기, `motor_adapter`를 실행한다. 기본값은 `false`다.

`teleop.launch.py`는 기존 두 launch를 조합하는 상위 주행 대기 launch다. Kobot의
기본 운영에서는 `kobot-sensors.service`가 실행한 센서를 재사용하고 모터 체인만
launch 수명주기에 추가한다. 센서 서비스를 사용하지 않을 때만
`start_sensors:=true`로 센서도 함께 실행한다. 모터 잠금 기본값은 `false`다.

## 센서와 화면 흐름

```text
/dev/videoCAM → usb_cam → /image_raw ─┬→ web_video_server → 전방 카메라 화면
                                      ├→ white_left_drive → white_left debug·status·preview
                                      └→ dashboard_server → 녹화 API
/dev/ttyLIDAR → xycar_lidar → /scan → dashboard LiDAR 화면
/dev/ttyIMU → imu_node → /imu → dashboard IMU 화면
/dev/ttySonic → ultrasonic_node → /xycar_ultrasonic → dashboard 초음파 화면
status_node → /kobot/status → dashboard 장치 상태
white_left_drive → /kobot_lane_drive/white_left/status → dashboard 차선 상태
```

기존 HSV/Canny/Hough `lane_preview` 노드와 토픽은 삭제했다. 차선 카드는 항상
`/kobot_lane_drive/white_left/debug_image`와 `/kobot_lane_drive/white_left/status`만
사용한다. 센서 launch가 실행하는 `white_left_drive`는 조향 미리보기까지만 계산하고
모터 publisher를 갖지 않는다.

`white_left_drive`는 픽셀 오차에 `steering_kp=0.42`를 적용한 뒤 별도
`steering_output_gain=2.0`으로 조향 요구량을 증폭한다. 최종 preview는 정지 시험에서
확인한 legacy angle `-30~+30`에 해당하는 `-0.204~+0.204 rad`로 제한된다.
같은 detector를 좌우 반전해 우측 ROI에도 적용하며, debug 영상에는 양쪽 현재 프레임
경계와 차량 중심으로부터 같은 `d=244px` 기준을 표시한다. 과거 경계 형상은 누적하지 않는다.

## 웹 기록 주행 흐름

```text
/image_raw
  → 매 프레임 흰색·저채도·국부 명암 ridge score
  → 차량 왼쪽의 가장 가까운 연속 흰색 경계 추적
  → 1번 사진 기준 d=244 px와 현재 경계 비교
  → 제한된 조향 preview, 웹에서 선택한 0.20~0.50 m/s
  ├→ /kobot_lane_drive/white_left/debug_image
  ├→ /kobot_lane_drive/white_left/status
  └→ /kobot_lane_drive/white_left/drive_preview
      → 승인 잠금 bridge → /xycar_motor
```

차선 형상은 누적하지 않는다. 새 프레임에서 차선이 한 번 끊기면 마지막 명령을
유지하지만, 카메라 또는 상태가 0.30초 이상 갱신되지 않으면 bridge는 0을 보낸다.
웹은 기록 주행 중 0.5초마다 heartbeat를 보내며 1.2초 동안 끊기면 서버가 일시정지한다.
`fast_lane`은 이 기록 주행의 제어 입력이 아니다. 기록 주행 launch는 센서 서비스의
상시 `white_left_drive`를 재사용하고 승인 잠금 bridge만 추가한다.
시작할 때마다 새 기록 폴더를 만들고, 일시정지·재개는 그 폴더를 유지한다. 종료는
CSV와 manifest를 닫아 세션을 확정하므로 다음 기록 주행의 데이터와 섞이지 않는다.

## 보정 없는 빠른 차선 흐름

```text
/image_raw
  -> LAB 밝기 정규화 + 흰색 경계/노란색 보조 mask
  -> 좌우 외곽선 sliding-window fitting
  -> 픽셀 lookahead 중심과 confidence
  |-> /kobot_lane_drive/fast_lane/status
  |-> /kobot_lane_drive/fast_lane/pixel_path
  `-> /kobot_lane_drive/fast_lane/debug_image (기본 5 Hz)
```

이 흐름은 카메라 K/TF 없이 실행할 수 있지만 픽셀 출력만 발행한다. 미터 단위
경로와 조향 제어의 대체물이 아니며 actuator publisher를 갖지 않는다. ROS 영상
구독 queue는 깊이 1이고, 화면 출력은 인식 주기보다 낮게 제한한다.

## CLRerNet 좌측 차선 흐름

```text
/image_raw → 공식 CLRerNet → 차량 중심 바로 왼쪽 경계 영상점
  → 고정 near/far 행에서 x 비율 추출
  → 기준 사진의 near/far x 비율과 비교
  → vendored simple-pid 두 루프 → 조향 미리보기(rad)
  ├→ /kobot_lane_drive/debug_image
  ├→ /kobot_lane_drive/status
  └→ /kobot_lane_drive/steering_preview
```

카메라와 트랙이 고정되어 있으므로 미터 거리, K 행렬, BEV/IPM, TF를 사용하지 않는다. 원하는 위치에 차량을 세운 사진에서 기준 픽셀 비율을 만든다. `reference_ready=false`, 좌측 차선 소실, near/far 행 샘플 실패 중 하나라도 발생하면 `BLOCKED`이며 새 조향값을 발행하지 않는다. 카메라 위치·각도·해상도·crop이 바뀌면 기준 사진을 다시 만든다.

기본 perception launch는 모터 토픽을 발행하지 않고 `sensors.launch.py`에도 자동 포함되지 않는다. 승인 잠금 `drive_bridge`만 `armed:=true`일 때 `/xycar_motor`를 발행한다.

## 모터 명령 흐름

```text
웹 WASD·시간 주행 또는 ROS publisher
  → /xycar_motor (std_msgs/Float32MultiArray: [steering, speed-command])
  → motor_adapter
  → /ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
  → ackermann_to_vesc_node
  ├→ /commands/motor/speed (std_msgs/Float64, ERPM)
  └→ /commands/servo/position (std_msgs/Float64)
  → vesc_driver_node → /dev/ttyMOTOR
```

`motor_adapter`는 속도 상한 `2.16 m/s`, 가속도 상한 `2.5 m/s²`, 명령 watchdog `0.5 s`를 적용한다. 웹 WASD는 누른 키 상태를 50 ms 간격(20 Hz)으로 계속 발행한다. 정지는 0 명령을 즉시 발행하고, watchdog은 명령이 끊기면 목표 속도를 0으로 만든다.

## 유지하는 공개 인터페이스

| 구분 | 이름 | 타입·역할 |
|---|---|---|
| 센서 입력 | `/image_raw` | `sensor_msgs/Image`, 카메라 원본 |
| 기록 주행 상태 | `/kobot_lane_drive/white_left/status` | 현재 프레임 경계·d 오차·조향 JSON |
| 기록 주행 미리보기 | `/kobot_lane_drive/white_left/drive_preview` | `[steering_rad, speed_mps, confidence]` |
| 기록 주행 화면 | `/kobot_lane_drive/white_left/debug_image` | 승인된 좌측 경계 초록색 overlay |
| 기록 주행 enable | `/kobot_lane_drive/record_drive/enabled` | 웹 일시정지/재개 transient-local Bool |
| 빠른 차선 상태 | `/kobot_lane_drive/fast_lane/status` | 픽셀 검출 상태·confidence·처리시간 JSON |
| 빠른 픽셀 경로 | `/kobot_lane_drive/fast_lane/pixel_path` | `[y_px, center_x_px, offset_px, confidence]` |
| 빠른 차선 화면 | `/kobot_lane_drive/fast_lane/debug_image` | 저주기 mask·외곽선·lookahead overlay |
| 센서 입력 | `/scan` | `sensor_msgs/LaserScan`, LiDAR |
| 센서 입력 | `/imu` | `sensor_msgs/Imu` |
| 센서 입력 | `/xycar_ultrasonic` | `std_msgs/Int32MultiArray` |
| 차량 입력 | `/xycar_motor` | `[steering, speed-command]` |
| 변환 | `/ackermann_cmd` | Ackermann 명령 |
| VESC 출력 | `/commands/motor/speed` | ERPM 명령 |
| VESC 출력 | `/commands/servo/position` | servo position 명령 |
| 차선 인식 입력 | `/image_raw` | CLRerNet 입력 영상 |
| 좌측 차선 화면 | `/kobot_lane_drive/debug_image` | 좌측 경계·near/far 점·픽셀 비율 오차 overlay |
| 좌측 차선 상태 | `/kobot_lane_drive/status` | 기준값·측정값·픽셀 비율 오차·조향 상태 JSON |
| 조향 미리보기 | `/kobot_lane_drive/steering_preview` | simple-pid 조향각 rad |

## HTTP API

dashboard server는 정적 화면과 다음 API를 제공한다.

- `GET /api/devices`, 장치 상태와 장치별 start/stop API
- `GET /api/recording`, `POST /api/recording/start`, `POST /api/recording/stop`
- `GET /api/motor`, `POST /api/motor/arm`, `POST /api/motor/restart`
- `POST /api/motor/test`, `POST /api/motor/stop`
- `GET /api/record-drive`, `POST /api/record-drive/start`
- `POST /api/record-drive/pause`, `POST /api/record-drive/resume`
- `POST /api/record-drive/finish`, `POST /api/record-drive/heartbeat`
- `GET /api/recovery`, 현재 복구 작업과 구성요소 실행 상태
- `POST /api/recovery/component/<name>`, 선택한 센서·차선·영상 서버만 재시작
- `POST /api/recovery/group/media`, 카메라·차선·영상 서버 묶음 재시작
- `POST /api/recovery/group/all`, 전체 센서 묶음 재시작

브라우저의 `영상 새로고침`은 API를 호출하지 않고 기존 MJPEG 연결을 닫은 뒤
cache-busting URL로 다시 연결한다. 서버의 실제 재시작 API는 녹화·기록 주행·시간
주행 중에는 거부하며, 모터 스택이 있으면 0 명령과 스택 종료를 선행한다.
WASD 무장·시간 주행·기록 주행 시작 전에는 ROS 그래프의 `/xycar_motor` publisher를
확인하고 `dashboard_server` 이외의 publisher가 있으면 시작을 거부한다.

인증·권한 검사가 없으므로 접근 가능한 사용자는 모터 스택을 시작하거나 명령할 수 있다. 신뢰된 로컬망/Tailscale에서만 사용한다.
