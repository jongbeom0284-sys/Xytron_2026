# NVIDIA Kobot 실제 차량 기준판

이 디렉터리는 NVIDIA Jetson 차량 `kobot`에서 함께 개발할 ROS 2 Humble 기준판이다. 현재 차량에서 확인한 센서 대시보드, 표시 전용 `lane_preview`, 웹 WASD·시간 주행, 모터 변환기, VESC 드라이버, LiDAR 드라이버까지만 포함한다.

자율주행 차선 제어, CLRerNet, `track_drive`, `kmu_race`, 과거 `real/drivers`, 과거 `lane_id_perception`은 포함하지 않는다.

## 구조

```text
real/
├─ AGENTS.md
├─ README.md
├─ docs/
├─ ros2_ws/src/
│  ├─ kobot_vehicle/
│  ├─ vesc/
│  └─ xycar_lidar/
├─ config/vesc_tool/
└─ system/
```

- `kobot_vehicle`: 센서 launch, 대시보드, 웹 주행, 모터 어댑터, `lane_preview`
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

운영 절차와 검증 순서는 [OPERATIONS.md](docs/OPERATIONS.md), 데이터 흐름은 [ARCHITECTURE.md](docs/ARCHITECTURE.md), 하드웨어·VESC 기준은 [HARDWARE_AND_VESC.md](docs/HARDWARE_AND_VESC.md), 원본과 수정 이력은 [PROVENANCE.md](docs/PROVENANCE.md), 파일별 역할은 [FILE_INDEX.md](docs/FILE_INDEX.md)를 따른다.

> 경고: dashboard의 모터 HTTP API에는 인증이 없다. 신뢰된 로컬망 또는 Tailscale 환경에서만 사용하고 인터넷에 직접 공개하지 않는다.
