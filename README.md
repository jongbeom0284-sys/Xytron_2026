# 2026 Xytron

가상환경과 실제 차량 코드를 저장소 최상위에서 분리한다.

```text
2026_Xytron/
├── simulation/  # Unity 시뮬레이터 전용 코드와 자산
└── real/        # 실제 차량 및 NVIDIA 컴퓨터용 코드
```

- [simulation](simulation/README.md): Unity ROS-TCP 기반 시뮬레이터와 기존 미션 코드
- [real](real/README.md): NVIDIA Kobot용 센서 대시보드, 표시용 차선 미리보기, 웹 주행, VESC·LiDAR 드라이버

두 환경의 ROS workspace를 한 터미널에서 동시에 `source`하지 않는다. 동일한 기능을 서로 다른 메시지와 토픽으로 구현하므로 환경별 터미널과 빌드 디렉터리를 분리한다.
