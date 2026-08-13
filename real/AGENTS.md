# NVIDIA Kobot 작업 규칙

이 디렉터리는 실제 NVIDIA Jetson 차량 `kobot`의 공동개발 기준판이다. 작업 전에 이 문서와 `README.md`, `docs/OPERATIONS.md`를 읽는다.

## 대상과 환경

- 대상 호스트: `kobot` (`xytron@100.95.40.26`), Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble
- SSH 키: `%USERPROFILE%\.ssh\jetson_j4012_auto`; 비밀번호를 파일·코드·명령줄에 남기지 않는다.
- 접속 직후 `hostname`을 실행하고 결과가 정확히 `kobot`인지 확인한다.
- `100.90.133.58`과 `10.42.0.1`은 기존 AMD 차량 주소이므로 사용하지 않는다.
- 저장소 기준 작업공간: `real/ros2_ws`; `ROS_DOMAIN_ID=7`, `ROS_NAMESPACE=xycar`, `ROS_LOCALHOST_ONLY=1`

## 빌드와 파일 규칙

- `real/ros2_ws`에서 `colcon build --symlink-install`을 실행한다.
- `build/`, `install/`, `log/` 안의 파일을 직접 수정하거나 커밋하지 않는다.
- 소스는 `ros2_ws/src/`에서만 수정한다. 생성 캐시, 백업 파일, 로그, 임시 출력은 커밋하지 않는다.
- `vesc/`와 `xycar_lidar/YDLidar-SDK/`는 외부 원본을 포함한다. 수정 전 `docs/PROVENANCE.md`의 기준 커밋과 로컬 패치 범위를 확인한다.
- VESC Tool XML은 비교·복구 참고 자료다. 이를 자동으로 장치에 기록하는 스크립트를 추가하지 않는다.

## 진단 순서

정상 판정은 반드시 `USB 장치 있음 → 노드 실행 → 토픽 publisher 있음 → 실제 메시지 또는 주기 수신` 순서로 한다. 토픽 이름이 목록에 있다는 사실만으로 정상이라 판단하지 않는다. 현재 IP, PID, 토픽 발행자와 주기는 매번 다시 확인한다.

## 모터 안전

- 승인 없이 ROS launch 실행·종료, 서비스 재시작, Jetson 파일 변경, 장치 write, VESC 설정 변경, `/xycar_motor`·`/ackermann_cmd` 및 하위 VESC 명령 발행을 하지 않는다.
- 모터 시험 전 정확한 명령, 위험, 중지 방법, 롤백을 설명하고 명시적 승인을 받는다.
- 시험은 바퀴를 띄우고 주변을 비운 뒤 낮은 속도부터 한다. `motor.launch.py`의 기본값 `armed:=false`를 유지한다.
- `sensors.launch.py`에 모터 노드를 추가하지 않는다.
- dashboard 모터 API는 인증이 없다. 신뢰된 로컬망 또는 Tailscale에서만 노출한다.

## 기능 경계

`lane_preview`는 HSV/Canny/Hough 기반 표시 전용 노드다. CLRerNet이나 자율주행 제어기가 아니며 모터 명령을 내보내면 안 된다. `track_drive`, `kmu_race`, CLRerNet 통합은 이 기준판 범위 밖이다.
