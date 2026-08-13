# 좌측 차선 거리 유지 개발 단계

## 목표

CLRerNet이 검출한 좌측 차선을 지면 좌표로 변환하고, 좌측 차선에서 `d m` 떨어진
경로를 Stanley 방식으로 추종한다. 최종 코드는
`real/ros2_ws/src/kobot_lane_drive`에서만 관리한다.

## Stage 1 - 공개 구현 통합

상태: `implemented`, Jetson CLRerNet runtime-verified, ROS 빌드 의존성 대기

- 공식 CLRerNet 고정 커밋
- ros-sports/ipm Humble 고정 커밋
- Shapely 2.1.2 `offset_curve`
- PythonRobotics Stanley 고정 커밋
- 직접 만든 픽셀 거리/BEV/조향 수식 제거

로컬 검증 근거:

- Kobot 통합 단위시험 `12 passed`
- upstream `ipm_library` 시험 `16 tests, 0 failures`
- WSL ROS2 Humble `ipm_interfaces`, `ipm_library`, `kobot_lane_drive` 통합 빌드 통과
- 두 launch 파일의 인자 해석 통과
- `armed:=false`, `calibration_ready:=false` smoke에서 `drive_bridge` 시작 0건

2026-08-13 Jetson 배치 근거:

- 최종 경로 `real/.venv`로 기존 CLRerNet 환경을 복제하고 원본은 보존
- Torch 2.8.0, TorchVision 0.23.0, Shapely 2.1.2, CUDA NMS import 통과
- 모델 SHA-256 `cce8b552bc8301e31186da192fe4213043e9351b35651ef5d40014ce3040c787` 일치
- Jetson 최종 venv에서 Kobot 통합 단위시험 `12 passed`
- 저장된 실제 카메라 프레임 `001.jpg` GPU 추론에서 차선 3개 반환, 161.16ms
- ROS 선택 빌드는 `vision_msgs` 미설치로 `ipm_interfaces`에서 차단됨
- `ros-humble-vision-msgs`, `ros-humble-camera-calibration`은 sudo 입력 후 설치 필요

## Stage 2 - 무모터 실시간 검증

상태: `blocked`

차량 읽기 결과 `/camera_info`의 K/R/P가 모두 0이고 `/tf_static`이 없다. 실제
카메라 내부 보정과 `base_link → default_cam` 장착 TF가 준비되기 전에는 IPM을
실행할 수 없다. 노드는 이 상태를 `CALIBRATION_INVALID`로 차단한다.

통과 기준:

1. `/camera_info`의 fx/fy가 양수이고 K 행렬이 비특이 행렬이다.
2. 이미지 timestamp에서 `base_link ↔ default_cam` TF가 조회된다.
3. 0.5/1.0/1.5m 바닥 표식의 IPM 오차가 각각 기록된다.
4. 실제 트랙 직선·좌곡선·우곡선 원본과 overlay를 함께 확인한다.
5. 평균/최악 추론 지연과 좌측 차선 누락률을 기록한다.

2026-08-13 읽기 전용/저장 프레임 추론 근거:

- hostname `kobot`, branch `main`, 배치 시점마다 HEAD와 GitHub main 일치 확인
- 실제 `/camera_info`: 640x480, K/R/P 전부 0, `/tf_static` 없음
- 로컬 `CLRerNetEngine` 파일을 원격 파일 생성 없이 Jetson GPU에서 직접 실행
- 저장 이미지 5장 추론: 4장 좌측 후보 선택, 1장 `LOST`
- warmup 후 지연 39.55~167.62ms, 5장 평균 약 85.59ms
- 체크포인트는 seg decoder 관련 missing key 경고가 있어 모델/가중치 조합을 실제 트랙 영상으로 추가 검증해야 함

판정: `실행 스모크 테스트만 통과`, 정확성 미검증

## Stage 3 - 조향 미리보기

상태: `implemented`, hardware 미검증

IPM 경로가 유효할 때만 Shapely 목표 경로와 PythonRobotics Stanley 조향값을
발행한다. 차선·보정·TF 중 하나가 실패하면 상태는 `BLOCKED`이고 새 조향값이
나오지 않는다. `/xycar_motor`와 `/ackermann_cmd`는 발행하지 않는다.

## Stage 4 - 바퀴를 띄운 시험

상태: `planned`, 사용자 승인 필요

현재 읽기 전용 점검에서 기존 `motor.launch.py armed:=true`, VESC driver,
Ackermann 변환기, motor_adapter, `kmu_race replay`가 실행 중이었다. 임의로
종료하지 않았다. 시험 직전에 PID·토픽 publisher·`/dev/ttyMOTOR` 점유를 다시
확인하고 충돌 프로세스 정리 승인을 받아야 한다.

## Stage 5 - 실제 트랙 저속 시험

상태: `planned`, 사용자 승인 필요

- 시작 속도 0.30m/s
- 물리 비상 정지 담당자 배치
- 최초에는 직선 짧은 구간
- 좌측 차선 소실/상태 timeout 시 0 명령 확인
- 원본, overlay, 상태 JSON, 지연, 거리 오차, 조향값을 기록

출력 생성만으로 정확성을 통과 처리하지 않는다. 실차 시험 전 상태는
`출력 생성됨 - 정확성 미검증` 또는 `실행 스모크 테스트만 통과`로 기록한다.
