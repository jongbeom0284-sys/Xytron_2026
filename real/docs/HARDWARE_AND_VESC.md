# 하드웨어와 VESC 기준

## 대상

- 컴퓨터: NVIDIA Jetson Orin NX, 호스트 `kobot`, Ubuntu 22.04, ROS 2 Humble
- VESC: HW 410, FW 6.5
- 장치 별칭: `/dev/videoCAM`, `/dev/ttyIMU`, `/dev/ttyLIDAR`, `/dev/ttyMOTOR`, `/dev/ttySonic`

`system/udev/10-kobot.rules`는 USB VID/PID를 위 별칭에 연결한다. 카메라는 한 USB 장치가 여러 video node를 만들 수 있으므로 `ATTR{index}=="0"`인 주 영상 노드만 `/dev/videoCAM`에 연결한다. 설치 전 실제 `udevadm info` 결과와 일치하는지 확인한다. 규칙을 복사하거나 reload하는 작업은 시스템 변경이므로 승인 후 수행한다.

## ROS 측 제한값

`kobot_vehicle/config/vesc.yaml`의 현재 기준은 다음과 같다.

- 포트 `/dev/ttyMOTOR`
- motor speed `-22500..22500 ERPM`
- servo position `0.15..0.85`
- Ackermann 변환: `speed_to_erpm_gain=4614.0`, offset `0.0`
- 조향 변환: gain `-1.2135`, center offset `0.5004`
- 차량 속도 상한 `4.87 m/s` (설정값. 22500 ERPM ÷ gain 4614), watchdog `0.5 s`
  - **실측치는 이 값에 미달한다.** 배터리(Traxxas 7셀 NiMH, 공칭 8.4 V)와
    제어 모드(BLDC 6-step 센서리스, Motor Detection 미실행) 제약 때문이며
    설정만 올려서는 도달하지 않는다.
- 가속도 상한: 소스 `motor.launch.py` 기본값은 `4.0 m/s²` 이나 `colcon build` 미반영으로
  **실행 중인 노드는 `2.5 m/s²`** 로 동작한다 (2026-08-22 확인). 값 변경은 보류 상태다.

웹 표시와 입력 상한도 `4.87 m/s`로 맞춘다. `8.00`은 과거 UI 표시 오류이며 실제 허용 속도가 아니다.

## VESC Tool XML

- `config/vesc_tool/motor_fw6_5_ki_0p0015.xml`: motor 설정 readback, FW 6.5, speed-controller `Ki=0.0015` 기준
- `config/vesc_tool/app_fw6_5_servo_enabled.xml`: app 설정 readback, servo output enabled 기준

두 XML은 백업·비교·수동 복구 판단용이다. 저장소가 자동으로 VESC에 기록하지 않으며 자동 기록 스크립트도 제공하지 않는다. 적용 전 VESC Tool의 연결 장치, HW/FW, 변경 diff를 다시 확인한다.

## 드라이버 호환 패치

VESC 모듈은 F1TENTH `153998df8545fe1781b975df88e411b4e71d4bfe`를 기준으로 FW 6.5 packet 호환을 위해 네 파일을 수정했다.

- `vesc_driver/include/vesc_driver/vesc_packet.hpp`
- `vesc_driver/src/vesc_driver.cpp`
- `vesc_driver/src/vesc_interface.cpp`
- `vesc_driver/src/vesc_packet.cpp`

다른 firmware에서 command enum·payload가 같다고 가정하지 않는다. firmware 변경 후에는 장치 write 전에 packet 정의를 대조하고, 설정을 다시 readback한다.

## 시험 전 필수 안전 조건

바퀴를 지면에서 띄우고 주변을 비운 뒤, 기존 VESC 프로세스와 serial 포트 소유자를 확인한다. 실제 시험은 별도 승인 후 `0.20 m/s`부터 짧게 수행하고 지속 명령, 즉시 정지, 조향 중앙 복귀, VESC `fault_code=0`을 확인한다.
