# NVIDIA Kobot 실제 차량 기준판

이 디렉터리는 NVIDIA Jetson 차량 `kobot`에서 함께 개발할 ROS 2 Humble 기준판이다. 센서 대시보드, 표시 전용 `lane_preview`, VESC/LiDAR 드라이버와 공개 구현 기반 고정 카메라 좌측 경계 추종 패키지를 포함한다.

현재 웹 기록 주행은 90번 사진에서 승인한 흰색 좌측 경계를 매 프레임 새로 추적하고, 1번 사진에서 얻은 영상상 거리 `d=244 px`를 유지한다. `fast_lane`과 `lane_preview`는 제어 입력이 아니다. CLRerNet·simple-pid 구현은 비교·후속 검증용으로 함께 보존한다. 기본 launch는 무모터이며 실제 출력 bridge는 명시적인 `armed:=true`와 사용자 승인 때만 실행된다. `track_drive`, `kmu_race`, 과거 평가 폴더는 최종 개발 위치가 아니다.

## 구조

```text
real/
├─ AGENTS.md
├─ README.md
├─ docs/
├─ ros2_ws/src/
│  ├─ kobot_vehicle/
│  ├─ kobot_lane_drive/
│  ├─ ipm/
│  ├─ vesc/
│  └─ xycar_lidar/
├─ config/vesc_tool/
└─ system/
```

- `kobot_vehicle`: 센서 launch, 대시보드, 웹 주행, 모터 어댑터, `lane_preview`
- `kobot_lane_drive`: 좌측 차선 검출, 고정 픽셀 기준, PID 조향 미리보기, 승인 잠금 drive bridge
- `ipm`: 이전 미터 경로 검토 때 가져온 공개 패키지이며 현재 차선 주행 runtime에서는 사용하지 않음
- `vesc`: F1TENTH ROS 2 VESC 드라이버와 Kobot FW 6.5 호환 패치
- `xycar_lidar`: ROS 2 LiDAR 노드와 vendored YDLidar SDK
- `config/vesc_tool`: 장치 자동 기록용이 아닌 VESC Tool 백업·비교 XML
- `system`: udev 규칙과 설치하지 않은 환경·systemd 예제

## 빠른 시작

```bash
cd ~/2026_Xytron/real/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7 ROS_NAMESPACE=xycar ROS_LOCALHOST_ONLY=1
ros2 launch kobot_vehicle sensors.launch.py
```

`sensors.launch.py`는 모터를 실행하지 않는다. 모터 launch는 기본적으로 잠겨 있다.

```bash
ros2 launch kobot_vehicle motor.launch.py          # armed=false, 모터 노드 미실행
ros2 launch kobot_vehicle motor.launch.py armed:=true  # 승인과 안전조치 후에만
```

Kobot은 부팅 시 `kobot-sensors.service`가 센서와 dashboard를 실행한다. 센서를
중복 실행하지 않고 VESC와 모터 어댑터까지 한 번에 주행 대기 상태로 만들려면 다음
상위 launch를 사용한다.

```bash
ros2 launch kobot_vehicle teleop.launch.py armed:=true
```

센서 서비스가 정지된 독립 개발 환경에서만 센서도 같은 프로세스에서 실행한다.

```bash
ros2 launch kobot_vehicle teleop.launch.py start_sensors:=true armed:=true
```

`kobot-sensors.service`가 실행 중일 때 `start_sensors:=true`를 사용하면 카메라와
직렬 센서 장치를 중복 점유하므로 금지한다. `teleop.launch.py`도 `armed:=false`가
기본이며, 실제 모터 연결은 사용자 승인과 안전조치 후 명시적으로 허용한다.

운영 절차와 검증 순서는 [OPERATIONS.md](docs/OPERATIONS.md), 데이터 흐름은 [ARCHITECTURE.md](docs/ARCHITECTURE.md), 3바퀴 시간경기·미션·실시간 성능 기준은 [RACE_STRATEGY.md](docs/RACE_STRATEGY.md), 하드웨어·VESC 기준은 [HARDWARE_AND_VESC.md](docs/HARDWARE_AND_VESC.md), 원본과 수정 이력은 [PROVENANCE.md](docs/PROVENANCE.md), 파일별 역할은 [FILE_INDEX.md](docs/FILE_INDEX.md)를 따른다.

> 경고: dashboard의 모터 HTTP API에는 인증이 없다. 신뢰된 로컬망 또는 Tailscale 환경에서만 사용하고 인터넷에 직접 공개하지 않는다.
