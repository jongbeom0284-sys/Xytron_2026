# Real vehicle

실제 차량과 NVIDIA 컴퓨터에서 사용하는 코드다.

```text
real/
├── lane_id_perception/  # 독립형 차선 ID 인식 ROS2 패키지
└── drivers/             # AMD 차량에 있던 카메라·IMU·LiDAR·모터 보조 패키지
```

NVIDIA 컴퓨터에는 우선 `lane_id_perception/`만 ROS2 workspace의 `src/` 아래로 가져간다. `drivers/`는 NVIDIA의 실제 장치와 토픽을 확인한 뒤 필요한 패키지만 선택한다.

차선 인식 패키지는 모터 명령을 publish하지 않는다. 기본 카메라 토픽은 `/image_raw`이며 launch 인자로 바꿀 수 있다.
