# 고정 카메라 좌측 경계 추종 개발 단계

## 목표와 고정 조건

CLRerNet이 검출한 현재 차로의 좌측 경계를 기준 사진과 같은 화면 위치에 유지한다.
최종 코드는 `real/ros2_ws/src/kobot_lane_drive`에서만 관리한다.

- 카메라는 차량에 고정되고 위치·각도·해상도·crop이 바뀌지 않는다.
- 같은 트랙이며 좌회전만 존재한다.
- 차량 중심 바로 왼쪽의 가장 가까운 차선 후보를 사용한다.
- 차량 좌측면을 좌측 경계에 원하는 만큼 붙여 우측 경계 침범을 방지한다.
- 미터 거리, K, BEV/IPM은 사용하지 않는다.

## Stage 1 - 공개 구현 통합

상태: `Windows unit-tested`, Jetson 미배치

- 공식 CLRerNet 고정 커밋
- DonkeyCar 목표 픽셀 PID 구조 참고
- Waveshare 가까운/먼 샘플 구조 참고
- MIT simple-pid 2.0.1 고정 소스
- 기준 사진에서 두 x 비율을 생성하는 offline replay
- `reference_ready=false`, 차선/샘플 소실 시 `BLOCKED`

Windows 검증은 순수 Python 단위시험과 정적 검사만 의미한다. Windows 작업본에는
실제 촬영본, CLRerNet 가중치, Torch/CUDA와 ROS2가 없으므로 모델 정확도, ROS
빌드와 실제 프레임 지연은 검증하지 못한다.

기존 Jetson 기록의 저장 프레임 GPU 추론 161.16ms는 CLRerNet 실행 스모크 근거일
뿐 현재 픽셀 제어 정확도 근거가 아니다.

## Stage 2 - 기준 사진과 저장 영상 검증

상태: `blocked - Jetson/촬영본 접근 대기`

1. 직선에서 차선과 평행하고 원하는 차량 위치인 `reference.jpg`의 원본과 CLRerNet overlay 확인
2. `pixel_reference.json` near/far x 비율 생성
3. 여러 직선·좌회전 프레임을 replay해 왼쪽 후보 ID, 오차, 조향 부호 확인
4. 차선 누락, 잘못된 물체 검출, 루프/교차선 오검출 프레임을 별도 기록
5. 평균·최악 추론 지연과 누락률 기록

통과 전 판정은 `출력 생성됨 - 정확성 미검증`이다.

## Stage 3 - 무모터 실시간 검증

상태: `planned`, Jetson launch 승인 필요

`lane_perception.launch.py`만 실행한다. `/kobot_lane_drive/debug_image`, 상태 JSON,
near/far 오차와 조향 미리보기를 기록한다. `/xycar_motor`, `/ackermann_cmd` 또는
VESC 하위 토픽을 발행하지 않는다.

통과 기준:

1. 직선과 모든 좌회전에서 현재 차로의 좌측 경계를 계속 선택한다.
2. 원하는 위치에서 오차가 0 부근이며 차량을 좌우로 옮긴 프레임의 부호가 맞다.
3. 검출 소실 후 상태가 즉시 `BLOCKED`로 바뀌고 새 조향값이 나오지 않는다.
4. 161ms 수준 지연에서도 저속 시험에 필요한 연속성이 있는지 기록한다.

## Stage 4 - 바퀴를 띄운 시험

상태: `planned`, 사용자 승인 필요

시험 직전에 기존 motor launch, VESC driver, Ackermann 변환기, motor adapter,
`/xycar_motor` publisher와 `/dev/ttyMOTOR` 점유를 다시 확인한다. 충돌 프로세스를
임의로 종료하지 않고 정리 계획과 롤백을 설명한 뒤 승인을 받는다.

## Stage 5 - 실제 트랙 저속 시험

상태: `planned`, 사용자 승인 필요

- 시작 속도 0.30m/s
- 물리 비상 정지 담당자 배치
- 최초에는 짧은 직선, 다음에 한 개 좌회전
- 차선 소실/상태 timeout 시 0 명령 확인
- 원본, overlay, 상태 JSON, 추론 지연, 픽셀 오차와 조향값 기록
- PID 값은 기록된 오차와 조향 진동을 보고 한 항목씩 조정

출력 생성만으로 정확성을 통과 처리하지 않는다.
