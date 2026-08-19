# 빌드·운영·검증 절차

## 1. 대상 확인

```bash
ssh -i ~/.ssh/kmu_car xytron@100.95.40.26
hostname
```

결과가 `kobot` 이 아니면 중단한다. 저장소는 `/home/xytron/2026_Xytron`,
ROS 2 workspace 는 `/home/xytron/2026_Xytron/real/ros2_ws` 다.
`xycar_ws`, `clrernet_eval` 에서는 최종 코드를 개발하지 않는다.

```bash
cd /home/xytron/2026_Xytron
test "$(git branch --show-current)" = main
git pull --ff-only origin main
```

## 2. 빌드와 시험

```bash
real/scripts/build_jetson.sh
```

스크립트는 호스트가 `kobot` 인지, TwinMixing venv 가 있는지 확인한 뒤
`colcon build --symlink-install` 을 돌리고 런타임 import 까지 검사한다.
특정 패키지만 빌드하려면 인자를 그대로 넘긴다.

```bash
real/scripts/build_jetson.sh --packages-select kobot_lane_drive
```

단위 시험은 패키지 루트에서 돌린다.

```bash
source /opt/ros/humble/setup.bash
cd real/ros2_ws/src/kobot_lane_drive && python3 -m pytest test -q   # 54개
cd ../kobot_vehicle             && python3 -m pytest test -q        # 21개
```

`build/`, `install/`, `log/` 는 생성물이다. 편집하거나 커밋하지 않는다.

## 3. 환경

```bash
source ~/kobot_env.sh   # ROS + install/setup.bash + DOMAIN 7 + LOCALHOST_ONLY
```

서로 다른 ROS workspace 의 `setup.bash` 를 한 터미널에 겹쳐 source 하지 않는다.
TwinMixing 추론에 필요한 Torch 는 `sensors.launch.py` 가 `PYTHONPATH` 로 주입한다.
수동 실행할 때는 직접 넣는다.

```bash
export PYTHONPATH=/home/xytron/.venvs/twinmixing/lib/python3.10/site-packages:$PYTHONPATH
```

## 4. 센서 읽기 전용 확인

판정은 반드시 이 순서로 한다. 토픽 이름이 목록에 있다는 사실만으로 정상이라 하지 않는다.

1. 장치: `ls -l /dev/videoCAM /dev/ttyIMU /dev/ttyLIDAR /dev/ttySonic`
2. 노드: `ros2 node list`
3. 발행자: `ros2 topic info -v /image_raw`
4. 데이터: `ros2 topic hz /image_raw /scan /imu /xycar_ultrasonic`

`/dev/ttyMOTOR` 가 없으면 모터 전원 스위치가 꺼진 것이다. 고장이 아니다.

```bash
systemctl is-active kobot-sensors.service   # active 여야 한다
ros2 launch kobot_vehicle sensors.launch.py # 서비스를 쓰지 않을 때만
```

센서 서비스가 실행 중일 때 같은 launch 를 또 띄우면 카메라와 직렬 장치를
중복 점유한다. 금지한다.

## 5. 카메라 설정 — 인식의 전제

이 차는 **색으로** 중앙선을 고른다. 카메라 설정이 틀리면 그 아래 인식·판단·조향이
전부 같이 틀어진다. 주행 전에 반드시 확인한다.

```bash
v4l2-ctl -d /dev/videoCAM --get-ctrl=white_balance_temperature,exposure_time_absolute,brightness,auto_exposure,white_balance_automatic
```

기대값:

| 항목 | 값 | 확인 |
|---|---|---|
| `white_balance_temperature` | **2800** | 이보다 높으면 화면이 초록으로 뜬다 |
| `white_balance_automatic` | 0 | 켜져 있으면 프레임마다 색이 흔들린다 |
| `exposure_time_absolute` | 40 | |
| `auto_exposure` | 1 (수동) | 3 이면 자동. 흰 바닥에 물려 화면이 날아간다 |
| `brightness` | 0 | 50 이면 화면이 완전히 날아간다 |

**값의 출처는 `kobot_vehicle/config/camera.yaml` 하나다.** `usb_cam` 은 이 값들을
ROS 파라미터로 읽기만 하고 장치에는 쓰지 않기 때문에, `sensors.launch.py` 가
노드 기동 8초 뒤 같은 값을 `v4l2-ctl` 로 한 번 더 건다. 값을 바꾸려면 yaml 을 고치고
센서 서비스를 재시작한다 — 코드를 고칠 필요 없다.

```bash
sudo systemctl restart kobot-sensors.service   # 차선 화면이 20~30초 끊긴다
```

**대회장에서 조명이 다르면 다시 맞춘다.** `camera_tuner.html`(:8000) 로 손으로
바꿔 보거나, 전 조합을 찍어 눈으로 고른다. 수치가 아니라 **노란 마스크를 원본 위에
겹쳐** 보고, 마스크가 중앙선 위에만 얹혔는지로 판단한다. 마스크가 차체나 노면으로
번지면 그 조합은 탈락이다.

## 6. 무모터 인식 확인

대시보드(`http://100.95.40.26:8000`)의 차선 카드에서 확인한다.

- **인식 상태** — `노란 중앙선` 이 기본이어야 한다. `센터 갭`/`영역 중앙` 이
  계속 나오면 **먼저 카메라 설정(§5)** 을 확인하고, 그 다음 `yellow_lab_margin` 을 본다.
- **주행 라인** — `중앙선 · 2차선 중앙` + 라이다 세 방향 거리
- 오버레이 색: 노랑 = 중앙선, 주황 = 왼쪽 차선, 보라 = 오른쪽 차선,
  하늘색 = 예정 경로, 빨간 십자 = 목표점

터미널에서 볼 때:

```bash
ros2 topic echo --once /kobot_lane_drive/twinmixing_center/status
```

## 7. 차 없이 검증하기 — 녹화 재생

실차를 세워 두고 지난 주행 영상에 현재 인식 코드를 그대로 돌릴 수 있다.

```bash
source ~/kobot_env.sh
export PYTHONPATH=/home/xytron/.venvs/twinmixing/lib/python3.10/site-packages:$PYTHONPATH
ros2 run kobot_lane_drive recording_lane_viewer --prepare-session <세션이름> --prepare-only
```

결과는 `~/recordings/camera/<세션>/_twinmixing_replay/analysis.json` 과
`overlays/` 에 남는다. 웹으로 나란히 재생하려면 `--prepare-only` 를 빼고 실행한 뒤
`http://100.95.40.26:8001` 을 연다.

`path_source` 분포로 노란선 검출률을, `narrow_side` 로 오검출 경향을 본다.
녹화에는 라이다가 없으므로 회피 판정은 이 방법으로 검증되지 않는다.

## 8. 테스트 주행

웹 `기록 주행` 패널에서 시작한다. 서버가
`twinmixing_center_record_drive.launch.py armed:=true start_perception:=false` 를
띄워 상시 인식 노드를 재사용하고 승인 잠금 bridge 만 추가한다.

시작 전 선택:

| 항목 | 값 |
|---|---|
| 속도 | `0.20` / `0.30` / `0.40` / `0.50 m/s` |
| 장애물 회피 허용 | 체크박스. **기본 꺼짐** |

권장 순서:

1. **회피 끄고** 낮은 속도로 — 노란 중앙선 추종만 확인한다.
2. 중앙선 추종이 안정되면 속도를 올린다.
3. 빈 트랙에 장애물 1대를 놓고 **회피를 켜서** `SHIFT` → `RETURNING` → `CENTER`
   전이를 웹 주행 라인 표시로 확인한다.

운전 중 조작:

- `Space` — 주행과 이미지 저장을 함께 일시정지·재개
- 종료 버튼 — 일시정지 상태에서만 활성화
- heartbeat 가 1.2초 끊기면 서버가 자동으로 일시정지한다

각 시작은 밀리초 단위의 새 저장 폴더를 만든다. 일시정지·재개는 같은 폴더를 유지하고,
종료 후 다음 시작은 별도 폴더에 저장한다.

무모터로 launch 만 확인하려면:

```bash
ros2 launch kobot_lane_drive twinmixing_center_record_drive.launch.py armed:=false
```

**웹 버튼은 `armed:=true` bridge 를 만든다.** 누르기 직전에 기존 모터 프로세스,
`/xycar_motor` publisher, `/dev/ttyMOTOR` 점유, 조향 부호를 다시 확인하고
별도 승인을 받는다.

## 9. 웹 확인 항목

- 속도 안내 최대값과 input `max` 가 `2.16 m/s` 로 일치
- WASD keydown 동안 `/xycar_motor` 20 Hz 지속 발행
- keyup, window blur, 즉시 정지에서 0 명령 발행
- 서버와 `motor_adapter` 가 각각 독립적으로 상한 검사
- `화면·센서 복구 > 영상 새로고침` 은 브라우저의 MJPEG 연결만 다시 연다.
  ROS 노드와 모터에는 손대지 않는다
- 구성요소 재시작은 녹화·기록 주행·시간 주행 중에는 거부된다
- 모터 API 에 인증이 없다. 인터넷에 직접 노출하지 않는다

## 10. 실제 모터 시험

문서만으로 승인되지 않는다. 실행 전 명시적 승인을 다시 받는다.

1. 바퀴를 띄우고 비상 정지 수단을 준비한다.
2. `/dev/ttyMOTOR` 소유 프로세스와 기존 모터 노드를 확인해 중복 접근을 제거한다.
3. `ros2 launch kobot_vehicle motor.launch.py armed:=true`
4. `0.20 m/s` 로 짧게 시험한다.
5. WASD 지속 발행, keyup/즉시 정지, 조향 중앙 복귀를 확인한다.
6. VESC 상태의 `fault_code=0` 을 확인한다.
7. 종료 시 0 명령을 보낸 뒤 모터 launch 를 종료한다.

`motor.launch.py` 의 `armed:=false` 기본값을 바꾸지 않는다.

## 11. systemd 와 udev

`real/system/` 은 설치 예제다. checkout 만으로 복사·enable·start 되지 않는다.
실제 경로와 사용자·그룹을 검토한 뒤 승인받아 설치한다.

`real/system/sudoers/kobot-sensors` 는 `/etc/sudoers.d/kobot-sensors` 에
root 소유·`0440` 으로 설치하고 `visudo -cf` 로 검사한다. 이 규칙은
`kobot-sensors.service` 의 start/stop/restart 만 허용한다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `/dev/ttyMOTOR` 없음 | 모터 전원 스위치 꺼짐 | 고장이 아니다. 스위치를 켜고 재무장 |
| 차선 화면이 20~30초 검게 | 센서 서비스 재시작 후 CUDA 모델 로딩 | 기다린다. 재시작할 거면 **미리 알린다** |
| 상태가 계속 `센터 갭`/`영역 중앙` | 노란선 미검출 | 먼저 §5 카메라 값부터 확인. 그 다음 `yellow_lab_margin` |
| 화면 전체가 초록빛 | 화이트밸런스가 2800 이 아님 | §5 |
| 목표선이 도로 밖 나무 기둥 위 | 도로 거리 필터가 느슨 | `yellow_maximum_road_distance_ratio` 를 줄인다 |
| 인식 노드가 살아 있는데 `status` 발행 0, CPU ~4% | **GPU 무거운 재생·스윕 스크립트를 돌린 직후 3회 재현.** 두 번째 CUDA 모델 적재와 충돌하는 것으로 보인다 (원인 미확정) | `sudo systemctl restart kobot-sensors.service`. 주행 중에는 GPU 스크립트를 돌리지 않는다 |
| 라이다 거리가 전부 `null` | 각도 보정 틀어짐 | `lidar_angle_offset_rad` 는 실측값 `-2.1217`. 라이다를 다시 장착했다면 재실측 |

## 롤백

- 코드: Git 의 이전 커밋으로 되돌린다. 단계별로 커밋이 나뉘어 있다.
- 이전 검출기: `legacy/lane-detectors/` 에서 해당 폴더를 `real/ros2_ws/src` 로
  옮기고 `setup.py` entry point 를 되살린다.
- 시스템 예제: 설치 전에 원본을 백업하고 변경 diff 를 기록한다.
- VESC: XML 을 자동 적용하지 말고 VESC Tool 에서 현재 readback 과 비교한 뒤 수동 판단한다.
