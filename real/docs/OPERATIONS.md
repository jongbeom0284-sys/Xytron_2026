# 빌드·운영·검증 절차

## 1. 대상 확인

```bash
ssh -i ~/.ssh/jetson_j4012_auto xytron@100.95.40.26
hostname
```

결과가 `kobot`이 아니면 중단한다. 최종 저장소는 `/home/xytron/2026_Xytron`, 실제 ROS 2 workspace는 `/home/xytron/2026_Xytron/real/ros2_ws`다. `xycar_ws`, `real_candidate`, `clrernet_eval`에서는 최종 코드를 개발하지 않는다.

저장소 작업 전 다음 순서를 지킨다.

```bash
cd /home/xytron/2026_Xytron
test "$(git branch --show-current)" = main
git pull --ff-only origin main
test "$(git rev-parse HEAD)" = "$(git ls-remote origin refs/heads/main | cut -f1)"
```

## 2. 통합 workspace 빌드

```bash
source /opt/ros/humble/setup.bash
source /home/xytron/2026_Xytron/real/.venv/bin/activate
cd /home/xytron/2026_Xytron/real/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon test --packages-select kobot_vehicle kobot_lane_drive
colcon test-result --verbose
```

`build/`, `install/`, `log/`는 생성물이며 소스처럼 편집하거나 커밋하지 않는다.

## 3. 환경

`system/env/kobot_env.sh.example`은 저장소 설치 경로 예제다. 자동 설치되지 않는다.

```bash
source /opt/ros/humble/setup.bash
source real/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=xycar
export ROS_LOCALHOST_ONLY=1
```

서로 다른 ROS workspace의 `setup.bash`를 한 터미널에 겹쳐 source하지 않는다.

## 4. 센서 읽기 전용 확인

1. 장치: `ls -l /dev/videoCAM /dev/ttyIMU /dev/ttyLIDAR /dev/ttySonic`
2. 노드: `ros2 node list`
3. 발행자: `ros2 topic info -v /image_raw` 등에서 publisher 수와 노드를 확인
4. 데이터: `ros2 topic hz /image_raw`, `/scan`, `/imu`, `/xycar_ultrasonic` 또는 `ros2 topic echo --once`

토픽 이름만 존재하면 통과가 아니다. publisher와 실제 메시지/주기가 모두 확인돼야 한다.

센서 launch에는 모터가 없다.

```bash
ros2 launch kobot_vehicle sensors.launch.py
```

대시보드의 차선 화면은 `/lane_preview/image`와 `/lane_preview/status`를 표시할 뿐 자율주행을 수행하지 않는다.

`kobot_lane_drive`도 기본 센서 launch에 포함되지 않는다. 패키지 빌드, 모델 준비, launch는 각각 승인 후 수행한다. `/camera_info` K 행렬과 카메라 TF를 먼저 검증하고, 무모터 launch 후 `/kobot_lane_drive/status`, 목표 Path, 조향 미리보기와 debug 영상을 확인한다. `BLOCKED` 상태에서 모터 단계로 넘어가지 않는다.

```bash
ros2 launch kobot_lane_drive lane_perception.launch.py
```

실제 출력은 별도 승인과 충돌 점검 뒤에만 실행한다.

```bash
ros2 launch kobot_lane_drive track_test.launch.py \
  calibration_ready:=true camera_x_m:=<실측> camera_y_m:=<실측> \
  camera_z_m:=<실측> camera_roll_rad:=<실측> \
  camera_pitch_rad:=<실측> camera_yaw_rad:=<실측> armed:=true
```

## 5. 웹 확인

- 속도 안내 최대값: `2.16 m/s`
- speed input의 DOM `max`: `2.16`
- WASD keydown 동안 `/xycar_motor` 20 Hz 지속 발행
- keyup, window blur, WASD 종료, 즉시 정지에서 0 명령 발행
- 서버와 `motor_adapter`도 `2.16 m/s` 상한을 독립적으로 검사

모터 API에는 인증이 없다. dashboard 포트와 rosbridge를 인터넷에 직접 노출하지 않는다.

## 6. 실제 모터 시험

이 절차는 문서만으로 승인되지 않는다. 실행 전 명시적 승인을 다시 받는다.

1. 바퀴를 띄우고 비상 정지 수단을 준비한다.
2. `/dev/ttyMOTOR` 소유 프로세스와 기존 모터 노드를 확인해 중복 접근을 제거한다.
3. `ros2 launch kobot_vehicle motor.launch.py armed:=true`를 실행한다.
4. `0.20 m/s`로 짧게 시험한다.
5. WASD 지속 발행, keyup/즉시 정지, 조향 중앙 복귀를 확인한다.
6. VESC 상태의 `fault_code=0`을 확인한다.
7. 종료 시 0 명령을 보낸 뒤 모터 launch를 종료한다.

`motor.launch.py`의 `armed:=false` 기본값을 바꾸지 않는다.

## 7. systemd와 udev

`system/systemd/kobot-sensors.service.example`과 `system/env/kobot_env.sh.example`은 설치 예제다. 저장소 checkout만으로 복사·enable·start되지 않는다. 실제 경로와 사용자·그룹을 검토한 뒤 승인받아 설치한다. 서비스는 sensor launch만 실행하고 모터를 실행하지 않는다.

## 롤백

- 코드: Git의 이전 커밋으로 `real/` 내용을 확인하거나 별도 checkout에서 복원한다.
- 시스템 예제: 설치 전에 원본 파일을 백업하고 변경 diff를 기록한다.
- VESC: XML을 자동 적용하지 말고 VESC Tool에서 현재 readback과 비교한 뒤 수동 판단한다.
