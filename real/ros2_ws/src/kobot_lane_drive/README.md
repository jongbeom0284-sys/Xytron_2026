# Kobot fixed-camera left-boundary follower

## 현재 기록 주행: 90번 승인 흰색 좌측 경계

웹의 `기록 주행 시작`은 `fast_lane`이나 `lane_preview`가 아니라, 90번 사진에서
사용자가 승인한 흰색 좌측 경계 추적기를 실행한다. 현재 프레임마다 흰색·저채도·
국부 명암 ridge 점수를 새로 만들고 차량 왼쪽의 가장 가까운 연속 경로를 아래에서
위로 추적한다. 차선 형상은 프레임 사이에 누적하지 않는다.

- 기준: 1번 사진 `y=336`, 좌측 경계 `x=76`, 영상 중심 `x=320`
- 원하는 영상상 거리 `d`: `244 px` (`244/640 = 0.38125`)
- 속도 상한: `0.20 m/s`
- 차선 한 프레임 미검출: 마지막 조향·속도 유지
- 카메라/상태가 `0.30초` 이상 끊김: bridge가 0 명령
- 브라우저 heartbeat가 `1.2초` 이상 끊김: 일시정지와 0 명령
- Space: 주행/저장을 함께 일시정지·재개
- 종료: 일시정지 뒤 마우스로만 가능

무모터 확인은 다음처럼 실행한다.

```bash
ros2 launch kobot_lane_drive white_left_record_drive.launch.py armed:=false
```

출력은 `/kobot_lane_drive/white_left/status`,
`/kobot_lane_drive/white_left/drive_preview`,
`/kobot_lane_drive/white_left/debug_image`다. 실제 `/xycar_motor` publisher는
`armed:=true`일 때만 별도 bridge로 생성된다. `fast_lane`은 비교·보존용이며 웹
기록 주행의 제어 입력이 아니다.

## Calibration-free fast lane baseline

`fast_lane_perception.launch.py`는 체크보드, 카메라 K 행렬, TF 없이 현재
`640x480` 영상에서 흰색 외곽선과 노란 중앙 점선을 픽셀 좌표로 검출한다.
HSV 흰색·노란색 마스크, CLAHE 밝기 정규화, Canny 경계, 노란색 보조 관측,
sliding-window 강건 다항식 fitting을 사용한다. 색상 후보가 ROI를 과도하게
점유하면 `mask_rejected=true`로 차단한다. 결과는 미터 경로나 모터 명령이 아니다.

```bash
ros2 launch kobot_lane_drive fast_lane_perception.launch.py
```

- 상태: `/kobot_lane_drive/fast_lane/status`
- 픽셀 lookahead: `/kobot_lane_drive/fast_lane/pixel_path`
  (`[y_px, center_x_px, offset_px, confidence]`)
- 저주기 화면: `/kobot_lane_drive/fast_lane/debug_image`

HSV와 과다 점유 차단 기준은 `config/fast_lane.yaml`에서 조정한다. 흰색은
`white_s_max`, `white_v_min`, `white_min_contrast`, 노란색은 `yellow_h_*`,
`yellow_s_min`, `yellow_v_min`을 사용한다. `max_white_support_ratio` 또는
`max_yellow_support_ratio`를 넘는 프레임은 경로 후보를 발행하지 않는다.

저장 프레임 benchmark는 다음처럼 실행한다.

```bash
ros2 run kobot_lane_drive fast_lane_benchmark /path/to/image_directory
```

인식 노드는 빠른 인식·조명 실험용이며 직접 모터 토픽을 발행하지 않는다. 실차
시험은 아래의 시간 필터·속도/조향 제한·명시적 잠금을 갖춘 전용 bridge만 사용한다.

## Fast-lane 주행 미리보기와 잠금 bridge

HSV 픽셀 경로를 조향·속도 미리보기로 변환하려면 다음 무모터 launch를 사용한다.

```bash
ros2 launch kobot_lane_drive fast_lane_track_test.launch.py armed:=false
```

- 미리보기: `/kobot_lane_drive/fast_lane/drive_preview`
  (`[steering_rad, speed_mps, confidence]`)
- 제어 상태: `/kobot_lane_drive/fast_lane/control_status`
- `BOTH`는 최대 `0.70 m/s`, 한쪽 차선은 최대 `0.48 m/s`로 계산한다.
- 순간 `LOST`와 `mask_rejected`는 0.25초 유지 후 0.50초까지 감속 정지한다.
- 조향은 `±0.34 rad`, 변화율은 `1.20 rad/s`로 제한한다. 짧은
  `BOTH ↔ 단일 차선` 전환은 마지막 양쪽 차선 중심을 0.30초 유지하고 이후
  복원 중심으로 서서히 이동해 곡선에서 조향이 튀는 것을 줄인다.

`armed:=false`에서는 controller와 perception에 모터 publisher가 없다. 실제 bridge는
`armed:=true`일 때만 생성되며, 실행 전 바퀴를 띄우고 기존 `/xycar_motor`
publisher 충돌, 모터 속도·가속도 상한과 조향 방향을 다시 확인해야 한다.

2026-08-14 Kobot 1차 실측:

- 기존 트랙 녹화 314장: `BOTH 32`, `LEFT_ONLY 252`, `RIGHT_ONLY 14`, `LOST 16`
- 픽셀 후보 생성률 `94.904%`; 수동 ground truth 정확도와 같은 의미는 아님
- Jetson 오프라인 처리시간: P50 `15.625 ms`, P95 `18.092 ms`, P99 `19.853 ms`
- 실제 `/image_raw` 상태 발행 약 `29.55 Hz`, debug 화면 약 `1.96 Hz`
- 기존 개발용 `lane_preview`가 함께 동작한 상태의 측정이며 motor subscriber는 0

현재 결과는 속도 계약을 통과했지만 단일 외곽선 복원 비율이 높고 시간 필터와
실제 조향 오차 ground truth가 없다. 따라서 `출력 생성됨 - 정확도 미검증` 단계다.

## Fixed-camera left-boundary follower

고정된 카메라와 같은 트랙을 전제로, 차량 중심 바로 왼쪽의 차선을 따라가는
ROS2 Humble 패키지다. 미터 단위 거리 `d`, 카메라 K, BEV/IPM은 사용하지 않는다.
차량을 원하는 위치에 세운 기준 사진에서 좌측 차선의 픽셀 위치를 저장하고 같은
위치를 유지한다.

공개 구현은 다음처럼 조합한다.

- 차선 검출: 공식 CLRerNet 고정 커밋
- 목표 픽셀 + PID 구조: DonkeyCar `line_follower.py` 참고
- 가까운 행/먼 행 두 지점 구조: Waveshare `ugv_jetson` 참고
- PID 계산: MIT `simple-pid` 2.0.1 vendored source

프로젝트 코드는 CLRerNet 결과에서 차량 중심 바로 왼쪽 후보를 선택하고, 두 고정
행에서 x를 선형 보간해 공개 PID에 전달하는 연결부만 담당한다.

## 입출력과 안전 상태

- 입력: `/image_raw`
- 화면: `/kobot_lane_drive/debug_image`
- 상태: `/kobot_lane_drive/status` (`READY` 또는 `BLOCKED`)
- 조향 미리보기: `/kobot_lane_drive/steering_preview` (`Float32`, rad)

`lane_perception.launch.py`는 모터 토픽을 발행하지 않는다. 설정 기본값은
`reference_ready: false`이며, 기준값을 넣기 전에는 `REFERENCE_NOT_READY`로
차단하고 조향 미리보기를 발행하지 않는다. 좌측 차선이 사라지거나 두 샘플 행 중
하나를 차선이 덮지 못해도 `BLOCKED`다.

`track_test.launch.py armed:=true`일 때만 별도 `drive_bridge`가
`/xycar_motor`를 발행한다. 실제 실행에는 매번 사용자 승인과 기존 모터 프로세스,
토픽 publisher, `/dev/ttyMOTOR` 점유 충돌 점검이 필요하다.

## 기준 사진 만들기

1. 카메라를 최종 위치에 고정한다.
2. 트랙의 직선 구간에서 차량 진행방향을 차선과 평행하게 맞춘다.
3. 차량의 좌측면이 좌측 차선에서 원하는 간격에 오도록 차량을 세운다.
4. 그 프레임을 `reference.jpg`로 저장한다.
5. 무모터 offline replay에서 기준값을 생성한다.

```bash
source /home/xytron/2026_Xytron/real/.venv/bin/activate
cd /home/xytron/2026_Xytron/real/ros2_ws
source install/setup.bash

ros2 run kobot_lane_drive offline_replay \
  --vendor-root src/kobot_lane_drive/third_party/CLRerNet \
  --config src/kobot_lane_drive/third_party/CLRerNet/configs/clrernet/culane/clrernet_culane_dla34_ema.py \
  --checkpoint /home/xytron/2026_Xytron/real/models/clrernet_culane_dla34_ema.pth \
  --reference-image /path/to/reference.jpg \
  --input /path/to/recorded/images \
  --output /tmp/kobot_lane_replay
```

`/tmp/kobot_lane_replay/pixel_reference.json`의 `near_x_ratio`와
`far_x_ratio`를 `config/lane_drive.yaml`의 `reference_near_x_ratio`,
`reference_far_x_ratio`에 복사한 후에만 `reference_ready: true`로 바꾼다.
도구가 YAML을 자동 수정하지 않는 이유는 잘못 고른 기준 사진이 즉시 주행 기준으로
반영되는 것을 막기 위해서다.

카메라 위치, 각도, 해상도, crop이 하나라도 바뀌면 기존 기준값은 폐기하고 다시
만든다. 카메라가 고정된 현재 차량에서는 체커보드 보정이나 미터 환산이 필요 없다.

## 검증 순서

1. 여러 저장 프레임을 offline replay하고 원본과 overlay를 함께 확인한다.
2. 직선과 모든 좌회전에서 선택한 선이 현재 차로의 좌측 경계인지 확인한다.
3. `near_error_ratio`, `far_error_ratio`, 조향 부호와 최대값을 확인한다.
4. 실시간 `lane_perception.launch.py`를 무모터로 실행하고 누락·지연을 기록한다.
5. 모터 단계는 별도 승인과 충돌 점검 후 바퀴를 띄운 상태에서 시작한다.

현재 Windows에는 실제 촬영본, CLRerNet 가중치와 CUDA runtime이 없으므로 단위
테스트만 수행할 수 있다. 실제 영상 출력은 검증 전까지 반드시
`출력 생성됨 - 정확성 미검증`으로 취급한다.
