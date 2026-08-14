# Kobot left-lane drive

## Calibration-free fast lane baseline

`fast_lane_perception.launch.py`는 체크보드, 카메라 K 행렬, TF 없이 현재
`640x480` 영상에서 흰색 외곽선과 노란 중앙 점선을 픽셀 좌표로 검출한다.
LAB 밝기 정규화, 밝은 중성색 경계, 노란색 보조 관측, sliding-window 강건
다항식 fitting을 사용한다. 결과는 미터 경로나 모터 명령이 아니다.

```bash
ros2 launch kobot_lane_drive fast_lane_perception.launch.py
```

- 상태: `/kobot_lane_drive/fast_lane/status`
- 픽셀 lookahead: `/kobot_lane_drive/fast_lane/pixel_path`
  (`[y_px, center_x_px, offset_px, confidence]`)
- 저주기 화면: `/kobot_lane_drive/fast_lane/debug_image`

저장 프레임 benchmark는 다음처럼 실행한다.

```bash
ros2 run kobot_lane_drive fast_lane_benchmark /path/to/image_directory
```

이 경로는 빠른 인식·조명 실험용이다. 보정된 BEV와 거리 기반 제어가 준비되기
전에는 `/xycar_motor` 또는 `/ackermann_cmd`로 연결하지 않는다.

좌측 차선으로부터 `target_distance_m`를 유지하는 ROS2 Humble 패키지다. 알고리즘을
새로 만들지 않고 다음 공개 구현을 연결한다.

- 차선 검출: 공식 CLRerNet
- 영상점 → 지면점(m): `ros-sports/ipm`
- 좌측 차선 평행 경로: Shapely `offset_curve`
- 조향값: PythonRobotics Stanley controller

`lane_perception.launch.py`는 인식과 조향 미리보기만 실행한다. 모터 토픽을
발행하지 않는다. `track_test.launch.py armed:=true`일 때만 별도
`drive_bridge`가 `/xycar_motor`를 발행한다. 실제 실행에는 매번 사용자 승인과
기존 모터 프로세스 충돌 점검이 필요하다.

## 입출력

- 입력: `/image_raw`, `/camera_info`, `base_link → default_cam` TF
- 출력: `/kobot_lane_drive/debug_image`
- 상태: `/kobot_lane_drive/status` (`READY` 또는 `BLOCKED`)
- 목표 경로: `/kobot_lane_drive/target_path` (`nav_msgs/Path`, m)
- 조향 미리보기: `/kobot_lane_drive/steering_preview` (`Float32`, rad)

`/camera_info`의 K 행렬이 0이거나 카메라 TF가 없거나 좌측 차선이 사라지면
미터 거리와 조향값을 발행하지 않는다. 현재 Kobot 실기에서 확인한 K 행렬은
전부 0이고 TF도 없으므로 카메라 보정 전 상태는 의도적으로 `BLOCKED`다.

## 실행 단계

1. 저장 영상: `offline_replay`로 CLRerNet 좌측 차선 선택을 검토한다.
2. 실시간 무모터: 카메라 내부 보정과 장착 TF를 준비한 뒤
   `lane_perception.launch.py`를 실행한다.
3. 바닥 실측: 0.5/1.0/1.5m 표식에서 IPM 거리 오차를 기록한다.
4. 바퀴를 띄운 시험: 별도 승인 후 `track_test.launch.py armed:=true`를 사용한다.
5. 트랙 저속: 0.30m/s부터 시작하고 차선 소실 시 정지를 확인한다.

가상환경·가중치·NMS 빌드 결과는 커밋하지 않는다. 최종 가상환경 위치는
`/home/xytron/2026_Xytron/real/.venv`다. `clrernet_eval/venv`를 복제해 먼저
검증하고, 이전 폴더는 검증 완료 전 삭제하지 않는다. Jetson에서는 이 가상환경을
활성화한 상태로 workspace를 빌드해 ROS entry point도 같은 Python 3.10을 쓴다.
