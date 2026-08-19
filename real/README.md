# NVIDIA Kobot 실차 기준판

Jetson Orin NX 차량 `kobot` 에서 실제로 도는 코드다. ROS 2 Humble.

차선 인식은 **TwinMixing** 하나만 상시 실행한다. 주행 목표선은 노란 중앙선을
1순위로 고르고, 한쪽 차선이 막히면 반대편 차선으로 비켜섰다가 중앙으로 돌아온다.
규칙 전문은 [docs/DRIVING_POLICY.md](docs/DRIVING_POLICY.md).

이전 검출기(흰선·fast_lane·CLRerNet)와 시뮬레이터는 저장소 최상위 `legacy/` 에 있다.

## 구조

```text
real/
├── AGENTS.md            작업 규칙
├── docs/                FOLDERS · DRIVING_POLICY · ARCHITECTURE · OPERATIONS
│                        · HARDWARE_AND_VESC · PROVENANCE
├── ros2_ws/src/
│  ├── kobot_lane_drive/ 차선 인식과 주행 판단 (새 주행 로직은 전부 여기)
│  ├── kobot_vehicle/    센서 노드 · 모터 어댑터 · 웹 대시보드
│  ├── vesc/             f1tenth VESC 드라이버 + FW 6.5 패치
│  └── xycar_lidar/      YDLidar SDK + ROS 2 wrapper
├── config/vesc_tool/    VESC 설정 백업 XML (참고용, 자동 적용 금지)
├── scripts/             빌드 · 사전점검
└── system/              udev · systemd · sudoers · env 설치 예제
```

폴더별 소유물은 [docs/FOLDERS.md](docs/FOLDERS.md) 한 곳에 있다.

## 빠른 시작

```bash
cd ~/2026_Xytron
real/scripts/build_jetson.sh
source ~/kobot_env.sh
ros2 launch kobot_vehicle sensors.launch.py
```

`sensors.launch.py` 는 카메라, 라이다, IMU, 초음파, TwinMixing 인식,
web_video_server(8080), rosbridge(9090), 대시보드(8000)를 띄운다. **모터는 없다.**

Kobot 은 부팅 시 `kobot-sensors.service` 가 같은 launch 를 실행한다. 서비스가
돌고 있으면 위 명령을 또 실행하지 않는다 — 카메라와 직렬 장치를 중복 점유한다.

## 모터

```bash
ros2 launch kobot_vehicle motor.launch.py            # armed=false, 모터 노드 미실행
ros2 launch kobot_vehicle motor.launch.py armed:=true  # 승인과 안전조치 후에만
ros2 launch kobot_vehicle teleop.launch.py armed:=true # 센서 재사용 + 모터 체인
```

`/xycar_motor` 를 발행하는 파일은 승인 잠금 bridge 하나뿐이고, 단위 시험이 이를
강제한다. 인식 노드에 모터 publisher 를 추가하지 않는다.

## 테스트 주행

웹 대시보드(`:8000`)의 `기록 주행` 패널에서 속도와 **장애물 회피 허용** 여부를
고르고 시작한다. 회피는 기본 꺼짐이다. 중앙선 추종부터 확인한 뒤 켠다.
절차는 [docs/OPERATIONS.md](docs/OPERATIONS.md) 8절.

## 카메라 설정이 먼저다

중앙선을 **색으로** 고르기 때문에 카메라 설정이 틀리면 인식이 통째로 틀어진다.
주행 전에 확인한다.

```bash
v4l2-ctl -d /dev/videoCAM --get-ctrl=white_balance_temperature,exposure_time_absolute,brightness
```

화이트밸런스 **2800**, 노출 40, 밝기 0 이어야 한다. 값의 출처는
`kobot_vehicle/config/camera.yaml` 하나이고, `sensors.launch.py` 가 기동 8초 뒤
`v4l2-ctl` 로 장치에 직접 쓴다 (`usb_cam` 은 파라미터를 읽기만 하고 장치에 쓰지 않는다).
자세한 내용은 [docs/OPERATIONS.md](docs/OPERATIONS.md) 5절.

## 차 없이 검증하기

지난 주행 녹화에 현재 인식 코드를 그대로 돌릴 수 있다.

```bash
ros2 run kobot_lane_drive recording_lane_viewer --prepare-session <세션> --prepare-only
```

## 문서

| 문서 | 무엇을 답하나 |
|---|---|
| [FOLDERS.md](docs/FOLDERS.md) | 무엇이 어디에 있나 |
| [DRIVING_POLICY.md](docs/DRIVING_POLICY.md) | 차가 어떤 규칙으로 달리나 |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | 노드·토픽·데이터 흐름 |
| [OPERATIONS.md](docs/OPERATIONS.md) | 빌드·검증·테스트 주행 절차 |
| [HARDWARE_AND_VESC.md](docs/HARDWARE_AND_VESC.md) | 하드웨어와 VESC 기준값 |
| [PROVENANCE.md](docs/PROVENANCE.md) | 외부 원본과 우리가 고친 범위 |
| [RACE_STRATEGY.md](docs/RACE_STRATEGY.md) | 대회 전략과 성능 목표 |

> 경고: 대시보드의 모터 HTTP API 에는 인증이 없다. 신뢰된 로컬망이나 Tailscale
> 안에서만 쓰고 인터넷에 직접 공개하지 않는다.
