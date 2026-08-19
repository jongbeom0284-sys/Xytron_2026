# NVIDIA Kobot 작업 규칙

이 디렉터리는 실차 `kobot` 의 공동개발 기준판이다. 작업 전에 이 문서와
`README.md`, `docs/FOLDERS.md`, `docs/OPERATIONS.md` 를 읽는다.

## 대상과 환경

- 대상 호스트: `kobot` (`xytron@100.95.40.26`), Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble
- 접속 직후 `hostname` 을 실행하고 결과가 정확히 `kobot` 인지 확인한다.
- `100.90.133.58` 과 `10.42.0.1` 은 기존 AMD 차량 주소다. 사용하지 않는다.
- 비밀번호를 파일·코드·명령줄에 남기지 않는다.
- 작업공간: `real/ros2_ws`. `ROS_DOMAIN_ID=7`, `ROS_NAMESPACE=xycar`, `ROS_LOCALHOST_ONLY=1`

## 어디에 코드를 두나

새 주행 로직은 **항상** `real/ros2_ws/src/kobot_lane_drive` 안에 만든다.
`legacy/` 는 읽기 전용 보관소다. 거기 있는 파일을 고쳐 쓰지 않는다.

`xycar_ws`, `clrernet_eval`, `xycar_ws_*_test` 는 최종 개발 위치가 아니다.

## 빌드와 파일 규칙

- `real/scripts/build_jetson.sh` 또는 `real/ros2_ws` 에서 `colcon build --symlink-install`
- `build/`, `install/`, `log/` 안의 파일을 직접 수정하거나 커밋하지 않는다.
- 소스는 `ros2_ws/src/` 에서만 수정한다. 생성 캐시, 백업 파일, 임시 출력은 커밋하지 않는다.
- `vesc/` 와 `xycar_lidar/YDLidar-SDK/` 는 외부 원본이다. 수정 전
  `docs/PROVENANCE.md` 의 기준 커밋과 로컬 패치 범위를 확인한다.
- VESC Tool XML 은 비교·복구 참고 자료다. 자동으로 장치에 기록하는 스크립트를 추가하지 않는다.

## 진단 순서

정상 판정은 반드시 `USB 장치 있음 → 노드 실행 → 토픽 publisher 있음 → 실제 메시지 또는 주기 수신`
순서로 한다. 토픽 이름이 목록에 있다는 사실만으로 정상이라 판단하지 않는다.
현재 IP, PID, 토픽 발행자와 주기는 매번 다시 확인한다.

`/dev/ttyMOTOR` 가 없으면 모터 전원 스위치가 꺼진 것이다. 고장으로 오진하지 않는다.

## 모터 안전

- 승인 없이 ROS launch 실행·종료, 서비스 재시작, Jetson 파일 변경, 장치 write,
  VESC 설정 변경, `/xycar_motor`·`/ackermann_cmd` 및 하위 VESC 명령 발행을 하지 않는다.
- 모터 시험 전 정확한 명령, 위험, 중지 방법, 롤백을 설명하고 명시적 승인을 받는다.
- 시험은 바퀴를 띄우고 주변을 비운 뒤 낮은 속도부터 한다.
  `motor.launch.py` 의 기본값 `armed:=false` 를 유지한다.
- `sensors.launch.py` 에 모터 노드를 추가하지 않는다.
- dashboard 모터 API 는 인증이 없다. 신뢰된 로컬망이나 Tailscale 에서만 노출한다.

## 기능 경계

`sensors.launch.py` 가 상시 실행하는 `kobot_lane_drive/twinmixing_center_drive` 는
차선 화면과 조향 미리보기만 발행한다. **여기에 모터 publisher 를 추가하면 안 된다.**

실제 모터 출력은 `twinmixing_center_drive_bridge_node.py` 한 곳뿐이고,
`test_motor_free.py` 가 이를 강제한다. 인식·판단 모듈에 `/xycar_motor` 문자열이
들어가면 시험이 깨진다.

장애물 회피(`maneuvers_enabled`)의 기본값은 거짓이다. 이 기본값을 바꾸지 않는다.
켜는 것은 웹 스위치나 launch 인자로 그때그때 한다.

## 튜닝 값의 위치

주행 관련 상수는 전부 `kobot_lane_drive/config/twinmixing_center.yaml` 에 있다.
소스에 숫자를 새로 박지 않는다. 현장에서 재빌드 없이 고칠 수 있어야 한다.

## 검증 습관

- 순수 모듈(`yellow_center`, `lane_policy`, `obstacle_lidar`, `geometry`)은 ROS 없이
  단위 시험으로 검증한다. 새 판단 로직을 넣으면 시험도 같이 넣는다.
- 실차를 세워 두고 검증할 때는 `recording_lane_viewer --prepare-only` 로 지난 녹화에
  현재 코드를 돌린다. 추측 대신 `path_source` 분포를 근거로 말한다.
