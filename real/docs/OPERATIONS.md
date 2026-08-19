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

현재 고정 카메라 차선 패키지만 확인할 때는 더 이상 사용하지 않는 `ipm`을 빌드
경로에 포함하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /home/xytron/2026_Xytron/real/.venv/bin/activate
cd /home/xytron/2026_Xytron/real/ros2_ws
rosdep install --from-paths src/kobot_lane_drive --ignore-src -r -y
colcon build --symlink-install --packages-select kobot_lane_drive
colcon test --packages-select kobot_lane_drive
colcon test-result --verbose
```

차량 전체 workspace를 함께 확인할 때만 다음 통합 명령을 사용한다.

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

Kobot의 센서 서비스가 정상 실행 중이고, 승인 후 VESC와 모터 어댑터까지 한 번에
주행 대기 상태로 만들 때는 다음을 사용한다.

```bash
systemctl is-active kobot-sensors.service
ros2 launch kobot_vehicle teleop.launch.py armed:=true
```

첫 명령의 결과가 `active`가 아니면 모터 단계로 넘어가지 않는다. 센서 서비스를
사용하지 않는 독립 실행에서는 서비스를 먼저 정지한 뒤 다음처럼 센서와 모터를
같은 launch 수명주기로 실행할 수 있다.

```bash
ros2 launch kobot_vehicle teleop.launch.py start_sensors:=true armed:=true
```

활성 센서 서비스와 `start_sensors:=true`를 동시에 사용하지 않는다. 종료 시에는
0 명령을 보낸 뒤 `Ctrl+C`로 teleop launch를 종료하고 모터 구독자가 0인지 확인한다.

대시보드의 차선 화면은 항상 `/kobot_lane_drive/white_left/debug_image`와
`/kobot_lane_drive/white_left/status`를 사용한다. 기존 `lane_preview` 실행 파일과
토픽은 없다. 센서 launch의 `white_left_drive`는 무모터이며 화면 확인만으로 차량을
움직이지 않는다.

debug 화면은 동일한 y 범위의 좌·우 대칭 ROI, 화면 중앙, 좌우 `d=244px` 기준점,
현재 프레임의 좌측 초록 경계와 우측 자홍 경계를 함께 표시한다. 이 단계의 실제 조향은
기존 좌측 경계 기준을 유지하며, 우측 결과는 ID·단독 추종 적용 전 시각 검증용이다.

웹 `기록 주행`은 별도 `white_left_record_drive.launch.py`를
`start_perception:=false`로 실행해 상시 인식 노드를 재사용하고 승인 잠금 bridge만
추가한다. Space는 주행과 이미지 저장을 함께 일시정지·재개하며 종료 버튼은
일시정지 상태에서 마우스로만 활성화된다.

무모터 확인:

```bash
ros2 launch kobot_lane_drive white_left_record_drive.launch.py armed:=false
```

실제 기록 주행은 웹 버튼이 `armed:=true` bridge를 만들므로, 버튼을 누르기 직전에
기존 모터 프로세스, `/xycar_motor` publisher, `/dev/ttyMOTOR` 점유와 조향 부호를
다시 확인하고 별도 승인을 받는다. 웹에서 기록 주행 시작 전에 `0.20`, `0.30`,
`0.40`, `0.50 m/s` 중 하나를 선택하며, 선택값은 해당 세션의 detector preview와
bridge 상한에 함께 적용된다. 각 시작은 밀리초 단위의 새 저장 폴더를 만들고,
일시정지·재개는 같은 폴더를 유지하며, 종료 후 다음 시작은 별도 폴더에 저장한다.

좌측 차선 제어의 `steering_kp=0.42`는 유지한다. 2026-08-17 정지 조향 시험에서
확인한 `/xycar_motor` `-30~+30` 범위에 맞춰 `steering_output_gain=2.0`을 적용하고,
최종 조향은 `max_steering_rad=0.204`로 제한한다. VESC 서보 gain/offset은 변경하지 않는다.

`kobot_lane_drive`의 무모터 `white_left_drive`는 기본 센서 launch에 포함된다. CLRerNet
비교 launch와 승인 잠금 motor bridge는 자동 실행되지 않는다.

```bash
ros2 launch kobot_lane_drive lane_perception.launch.py
```

실제 출력은 별도 승인과 충돌 점검 뒤에만 실행한다.

```bash
ros2 launch kobot_lane_drive track_test.launch.py \
  armed:=true
```

위 명령은 실행 예시일 뿐 승인으로 간주하지 않는다. 실행 직전에 기존 모터
프로세스, `/xycar_motor` publisher와 `/dev/ttyMOTOR` 점유를 다시 확인하고 별도
승인을 받는다.

## 5. 웹 확인

- 속도 안내 최대값: `2.16 m/s`
- speed input의 DOM `max`: `2.16`
- WASD keydown 동안 `/xycar_motor` 20 Hz 지속 발행
- keyup, window blur, WASD 종료, 즉시 정지에서 0 명령 발행
- 서버와 `motor_adapter`도 `2.16 m/s` 상한을 독립적으로 검사
- 기록 주행은 시작 전 `0.20~0.50 m/s` 선택, Space 일시정지/재개, 일시정지 후 마우스 종료
- 기록 주행 heartbeat 단절 1.2초 시 서버 일시정지
- `화면·센서 복구 > 영상 새로고침`은 브라우저의 기존 MJPEG 연결만 닫고 다시 연다. ROS 노드와 모터에는 손대지 않는다.
- 카메라·차선 인식·영상 서버·LiDAR·초음파·IMU는 각각 재시작하거나, 영상 묶음/전체 센서 묶음으로 재시작할 수 있다.
- 실제 구성요소 재시작은 녹화·기록 주행·시간 주행 중에는 거부된다. 모터 스택이 남아 있으면 0 명령 후 종료한 다음 센서를 재시작한다.
- WASD 무장·시간 주행·기록 주행 시작은 dashboard 외 `/xycar_motor` publisher가 하나라도 있으면 거부한다. 이전 teleop 노드를 종료한 뒤 다시 시도한다.

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

센서 대시보드 서비스 재가동 때 비밀번호를 저장하거나 명령줄에 넣지 않는다. 최초 한
번만 관리자가 `real/system/sudoers/kobot-sensors`를
`/etc/sudoers.d/kobot-sensors`에 root 소유·`0440` 권한으로 설치하고 `visudo -cf`로
검사한다. 이 규칙은 `kobot-sensors.service`의 start/stop/restart만 허용하며 다른 sudo
명령이나 모터 서비스 권한은 열지 않는다.

`system/systemd/kobot-sensors.service.example`과 `system/env/kobot_env.sh.example`은 설치 예제다. 저장소 checkout만으로 복사·enable·start되지 않는다. 실제 경로와 사용자·그룹을 검토한 뒤 승인받아 설치한다. 서비스는 sensor launch만 실행하고 모터를 실행하지 않는다.

## 롤백

- 코드: Git의 이전 커밋으로 `real/` 내용을 확인하거나 별도 checkout에서 복원한다.
- 시스템 예제: 설치 전에 원본 파일을 백업하고 변경 diff를 기록한다.
- VESC: XML을 자동 적용하지 말고 VESC Tool에서 현재 readback과 비교한 뒤 수동 판단한다.
