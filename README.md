# 2026 Xytron — 국민대 자율주행 경진대회

NVIDIA Jetson 차량 `kobot` 의 주행 코드다. 저장소는 두 축으로만 나뉜다.

```text
2026_Xytron/
├── real/     실차에서 실제로 도는 코드. 이것만 빌드된다.
└── legacy/   이전 시도 보관소. 빌드되지 않는다.
```

- **[real](real/README.md)** — TwinMixing 차선 인식, 노란 중앙선 우선 주행, 웹 대시보드,
  VESC·LiDAR 드라이버, 센서 노드
- **[legacy](legacy/README.md)** — 흰선·fast_lane·CLRerNet 세 검출기, Unity 시뮬레이터,
  차선 알고리즘 비교 근거, 원본 노란선 노드

## 주행 규칙 한 줄 요약

노란 중앙선을 중심으로 2차선 전체 폭을 쓰며 달리고, 한쪽 차선이 막히면 반대편
차선으로 비켜섰다가 추월 후 중앙으로 돌아온다. 자세한 내용은
[DRIVING_POLICY.md](real/docs/DRIVING_POLICY.md).

## 어디에 무엇이 있는지

폴더별 소유물은 [FOLDERS.md](real/docs/FOLDERS.md) 한 곳에 정리돼 있다.

## 시작하기

```bash
cd ~/2026_Xytron/real/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7 ROS_NAMESPACE=xycar ROS_LOCALHOST_ONLY=1
ros2 launch kobot_vehicle sensors.launch.py
```

`sensors.launch.py` 는 모터를 실행하지 않는다. 모터는 명시적 승인과
`armed:=true` 없이는 절대 돌지 않는다.

> 경고: 대시보드의 모터 HTTP API 에는 인증이 없다. 신뢰된 로컬망이나 Tailscale
> 안에서만 쓰고 인터넷에 직접 공개하지 않는다.
