# 아키텍처와 공개 인터페이스

## 실행 경계

`sensors.launch.py`는 카메라, 초음파, IMU, LiDAR, 상태 수집, `lane_preview`, web video server, rosbridge, dashboard server를 실행한다. VESC와 모터 변환기는 실행하지 않는다.

`motor.launch.py`는 `armed:=true`일 때만 VESC 드라이버, Ackermann 변환기, `motor_adapter`를 실행한다. 기본값은 `false`다.

## 센서와 화면 흐름

```text
/dev/videoCAM → usb_cam → /image_raw ─┬→ web_video_server → 전방 카메라 화면
                                      ├→ lane_preview → /lane_preview/image
                                      └→ dashboard_server → 녹화 API
/dev/ttyLIDAR → xycar_lidar → /scan → dashboard LiDAR 화면
/dev/ttyIMU → imu_node → /imu → dashboard IMU 화면
/dev/ttySonic → ultrasonic_node → /xycar_ultrasonic → dashboard 초음파 화면
status_node → /kobot/status → dashboard 장치 상태
lane_preview → /lane_preview/status → dashboard 차선 상태
```

`lane_preview`는 OpenCV HSV 색상 마스크, Canny edge, Hough 선분을 이용해 카메라 영상 위에 좌우 차선 후보를 그리는 표시 전용 노드다. CLRerNet 모델이 아니며 경로 계획이나 차량 제어를 하지 않는다. 출력 정확도는 실제 트랙 대표 영상으로 별도 검증해야 한다.

## CLRerNet 좌측 차선 흐름

```text
/image_raw → 공식 CLRerNet → 가장 가까운 좌측 경계 영상점
/camera_info + base_link↔default_cam TF
  → ros-sports/ipm → 좌측 경계 지면점(m)
  → Shapely offset_curve(d) → 목표 Path(m)
  → PythonRobotics Stanley → 조향 미리보기(rad)
  ├→ /kobot_lane_drive/debug_image
  ├→ /kobot_lane_drive/status
  ├→ /kobot_lane_drive/target_path
  └→ /kobot_lane_drive/steering_preview
```

`d`는 미터 단위다. 카메라 K 행렬이나 TF가 유효하지 않으면 `CALIBRATION_INVALID`로 차단하고 목표 경로와 조향값을 발행하지 않는다.

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
| 센서 입력 | `/scan` | `sensor_msgs/LaserScan`, LiDAR |
| 센서 입력 | `/imu` | `sensor_msgs/Imu` |
| 센서 입력 | `/xycar_ultrasonic` | `std_msgs/Int32MultiArray` |
| 차량 입력 | `/xycar_motor` | `[steering, speed-command]` |
| 변환 | `/ackermann_cmd` | Ackermann 명령 |
| VESC 출력 | `/commands/motor/speed` | ERPM 명령 |
| VESC 출력 | `/commands/servo/position` | servo position 명령 |
| 화면 | `/lane_preview/image` | 오버레이 영상 |
| 화면 | `/lane_preview/status` | 검출 상태 JSON 문자열 |
| 차선 인식 입력 | `/image_raw` | CLRerNet 입력 영상 |
| 좌측 차선 화면 | `/kobot_lane_drive/debug_image` | 좌측 경계·목표 경로·오차 overlay |
| 좌측 차선 상태 | `/kobot_lane_drive/status` | 보정·IPM·거리·조향 상태 JSON |
| 목표 경로 | `/kobot_lane_drive/target_path` | `nav_msgs/Path`, base_link 미터 좌표 |
| 조향 미리보기 | `/kobot_lane_drive/steering_preview` | Stanley 조향각 rad |

## HTTP API

dashboard server는 정적 화면과 다음 API를 제공한다.

- `GET /api/devices`, 장치 상태와 장치별 start/stop API
- `GET /api/recording`, `POST /api/recording/start`, `POST /api/recording/stop`
- `GET /api/motor`, `POST /api/motor/arm`, `POST /api/motor/restart`
- `POST /api/motor/test`, `POST /api/motor/stop`

인증·권한 검사가 없으므로 접근 가능한 사용자는 모터 스택을 시작하거나 명령할 수 있다. 신뢰된 로컬망/Tailscale에서만 사용한다.
