# 파일 색인

이 표는 `real/`에 커밋되는 파일의 출처, 역할, 주요 입력·출력, 편집 정책을 기록한다. 생성물(`build/`, `install/`, `log/`), 캐시, 백업은 포함하지 않는다. 큰 vendored subtree는 기준 commit과 패치 경계를 한 항목으로 묶어 기록한다.

총 추적 파일 수: **452**

| 경로 | 출처 | 역할 | 입력·출력 | 수정 정책 |
|---|---|---|---|---|
| `real/.gitattributes` | Kobot 프로젝트 자체 작성 | vendored upstream 공백 보존 정책 | Git diff 검사 → upstream 원문 보존 | 경로 변경 시 함께 갱신 |
| `real/AGENTS.md` | Kobot 프로젝트 자체 작성 | AI·공동개발 작업 규칙 | 작업자 입력 → 안전·빌드 규칙 | 검토·검증 후 수정 가능 |
| `real/README.md` | Kobot 프로젝트 자체 작성 | 기준판 시작 안내 | 개발자 입력 → 구조·빠른 시작 | 검토·검증 후 수정 가능 |
| `real/config/vesc_tool/app_fw6_5_servo_enabled.xml` | Kobot VESC Tool readback (2026-08-13) | VESC app readback | VESC Tool 비교용 XML | 참고 전용; 자동 적용 금지 |
| `real/config/vesc_tool/motor_fw6_5_ki_0p0015.xml` | Kobot VESC Tool readback (2026-08-13) | VESC motor readback | VESC Tool 비교용 XML | 참고 전용; 자동 적용 금지 |
| `real/docs/ARCHITECTURE.md` | Kobot 프로젝트 자체 작성 | 노드·토픽·HTTP 구조 | 소스 인터페이스 → 공동 설계 기준 | 검토·검증 후 수정 가능 |
| `real/docs/FILE_INDEX.md` | Kobot 프로젝트 자체 작성 | 모든 커밋 파일의 색인 | 저장소 파일 → 출처·역할·편집 정책 | 검토·검증 후 수정 가능 |
| `real/docs/HARDWARE_AND_VESC.md` | Kobot 프로젝트 자체 작성 | 하드웨어·VESC 안전 기준 | 실장 설정 → 제한값·시험 조건 | 검토·검증 후 수정 가능 |
| `real/docs/LANE_DRIVE_STAGE_SPEC.md` | Kobot 프로젝트 자체 작성 | 좌측 차선 거리 유지 단계·검증 계약 | 구현·실측 근거 → 단계 승인 | 단계 결과와 함께 갱신 |
| `real/docs/OPERATIONS.md` | Kobot 프로젝트 자체 작성 | 빌드·운영·검증 절차 | 운영자 입력 → 검증 순서 | 검토·검증 후 수정 가능 |
| `real/docs/PROVENANCE.md` | Kobot 프로젝트 자체 작성 | 원본 commit과 패치 기록 | upstream/차량 스냅샷 → 추적 정보 | 검토·검증 후 수정 가능 |
| `real/docs/RACE_STRATEGY.md` | Kobot 프로젝트 자체 작성 | 3바퀴 시간경기·미션·실시간 성능 전략 | 규정·실측 → 구현 순서와 승급 기준 | 규정·실측 변경 시 함께 갱신 |
| `real/models/README.md` | Kobot 프로젝트 자체 작성 | CLRerNet 가중치 출처·checksum | 모델 준비 → 무결성 검증 | 모델 파일 자체는 Git 제외 |
| `real/calibration/README.md` | Kobot 프로젝트 자체 작성 | 카메라 내부·외부 보정 gate | 실측 보정 → 유효 CameraInfo·TF | 추정값 금지 |
| `real/requirements/clrernet-jetson.freeze.txt` | Kobot 평가 venv read-only snapshot | Jetson Python 환경 재현 기록 | 기존 venv → 고정 버전 목록 | 실제 호환 검증 후 갱신 |
| `real/scripts/` | Kobot 프로젝트 자체 작성 | runtime 복제·venv 빌드·읽기 전용 preflight | 승인된 운영 입력 → 검증 결과 | 실행 전 승인 경계 확인 |
| `real/ros2_ws/src/kobot_lane_drive/` | Kobot 통합 + 공개 알고리즘 | 90번 승인 흰색 좌측 경계 기록 주행, fast-lane 비교판, CLRerNet 고정 픽셀 추종과 승인 잠금 bridge | 이미지/기준 사진 → 픽셀 경로·debug·상태·preview·승인 시 motor | 공개 구현 provenance와 안전 gate 유지 |
| `real/ros2_ws/src/kobot_lane_drive/kobot_lane_drive/white_left_boundary.py` | Kobot 프로젝트 자체 작성 | 현재 프레임 흰색 ridge와 방향 연속성으로 좌측 경계 추적 | BGR 영상 → 경계 path·d 픽셀 오차·overlay | 1번/90번 승인 기준과 함께 변경 |
| `real/ros2_ws/src/kobot_lane_drive/kobot_lane_drive/white_left_drive_node.py` | Kobot 프로젝트 자체 작성 | 무모터 경계 검출·제한 조향 preview | /image_raw → white_left status·preview·debug | 모터 publisher 추가 금지 |
| `real/ros2_ws/src/kobot_lane_drive/kobot_lane_drive/white_left_drive_bridge_node.py` | Kobot 프로젝트 자체 작성 | 명시적 armed·웹 enable·freshness gate motor bridge | preview·status·enable → /xycar_motor | 승인 잠금과 0.30초 watchdog 유지 |
| `real/ros2_ws/src/kobot_lane_drive/launch/white_left_record_drive.launch.py` | Kobot 프로젝트 자체 작성 | 웹 기록 주행 전용 launch | armed·속도 상한 → detector·조건부 bridge | armed 기본값 false 유지 |
| `real/ros2_ws/src/kobot_lane_drive/third_party/CLRerNet/` | CLRerNet upstream `dae038f` + NMS patch | CLRerNet 추론 runtime source/config | 이미지 tensor → 차선 후보 | upstream 원문 유지; 패치는 provenance와 함께 관리 |
| `real/ros2_ws/src/kobot_lane_drive/kobot_lane_drive/_vendor/simple_pid.py` | simple-pid 2.0.1 `6e6f4b0` | near/far 픽셀 PID 계산 | 기준/측정 x 비율 → 제한된 조향값 | 알고리즘 변경 금지; upstream 갱신으로 관리 |
| `real/ros2_ws/src/ipm/` | ros-sports/ipm Humble `2943925` | 이전 미터 경로 검토용 지면 좌표 투영 | CameraInfo·TF·영상점 → 지면점(m) | 현재 lane runtime 미사용; upstream 원문 유지 |
| `real/ros2_ws/src/kobot_vehicle/config/camera.yaml` | Kobot 차량 실행본 (2026-08-13) | 카메라 파라미터 | launch → usb_cam | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/config/imu.yaml` | Kobot 차량 실행본 (2026-08-13) | IMU 파라미터 | launch → imu_node | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/config/lidar.yaml` | Kobot 차량 실행본 (2026-08-13) | LiDAR 파라미터 | launch → xycar_lidar | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/config/ultrasonic.yaml` | Kobot 차량 실행본 (2026-08-13) | 초음파 파라미터 | launch → ultrasonic_node | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/config/vesc.yaml` | Kobot 차량 실행본 (2026-08-13) | VESC 포트·명령 제한 | launch → vesc_driver | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/dashboard/app.js` | Kobot 차량 실행본 + 기록 주행·복구 UI | 대시보드·WASD 20 Hz·기록 주행·MJPEG 재연결·센서 복구 | HTTP/ROSBridge 입력 → DOM·/xycar_motor·record-drive/recovery API | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/dashboard/drive.css` | Kobot 차량 실행본 (2026-08-13) | 주행 카드 스타일 | 주행 DOM → 화면 스타일 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/dashboard/index.html` | Kobot 차량 실행본 (2026-08-13) | 대시보드 HTML 진입점 | 브라우저 요청 → 화면 골격 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/dashboard/roslib.min.js` | roslibjs vendored minified asset | vendored roslib 브라우저 client | WebSocket → ROS topic API | 직접 수정 금지; upstream 교체 |
| `real/ros2_ws/src/kobot_vehicle/dashboard/style.css` | Kobot 차량 실행본 (2026-08-13) | 대시보드 공통 스타일 | HTML class → 화면 스타일 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/__init__.py` | Kobot 차량 실행본 (2026-08-13) | Python package marker | Python import 입력 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/dashboard_server.py` | Kobot 차량 실행본 + 기록 주행·복구 제어 | 정적 웹·장치·녹화·motor·record-drive·센서 복구 상태기계 | HTTP·/image_raw·white_left status → API·세션 기록·/xycar_motor·프로세스 재시작 | 승인·heartbeat·일시정지·모터 선종료 gate 유지 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/figure_eight.py` | Kobot 차량 실행본 (2026-08-13) | 수동 승인용 8자 시험 publisher | 시험 파라미터 → /xycar_motor | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/imu_node.py` | Kobot 차량 실행본 (2026-08-13) | IMU serial parser/publisher | /dev/ttyIMU → /imu | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/motor_adapter.py` | Kobot 차량 실행본 (2026-08-13) | 차량 명령 안전 변환기 | /xycar_motor → /ackermann_cmd | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/status_node.py` | Kobot 차량 실행본 (2026-08-13) | 장치·시스템 상태 publisher | OS 상태 → /kobot/status | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/kobot_vehicle/ultrasonic_node.py` | Kobot 차량 실행본 (2026-08-13) | 초음파 serial parser/publisher | /dev/ttySonic → /xycar_ultrasonic | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/launch/motor.launch.py` | Kobot 차량 실행본 (2026-08-13) | 잠금 기본 모터 launch | armed 인자 → VESC·변환 노드 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/launch/sensors.launch.py` | Kobot 차량 실행본 (2026-08-13) | 센서·대시보드 launch, 모터 제외 | launch 인자 → 센서 노드 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/launch/teleop.launch.py` | Kobot 프로젝트 자체 작성 | 센서 서비스 재사용 또는 센서·모터 통합 주행 대기 launch | start_sensors·armed 인자 → 기존 launch 조합 | 모터 잠금 기본값 유지 |
| `real/ros2_ws/src/kobot_vehicle/package.xml` | Kobot 차량 실행본 (2026-08-13) | ROS package manifest | 의존성 선언 → rosdep/ament | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/resource/kobot_vehicle` | Kobot 차량 실행본 (2026-08-13) | ament resource marker | ament index 입력 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/setup.cfg` | Kobot 차량 실행본 (2026-08-13) | ament Python 설치 설정 | build 설정 → 설치 경로 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/setup.py` | Kobot 차량 실행본 (2026-08-13) | Python package·entry point 정의 | ament build → 실행 파일 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/test/test_sensor_launch.py` | Kobot 프로젝트 자체 작성 | 센서 launch 차선 노드·무모터 계약 회귀 테스트 | launch·setup source → white_left 상시 실행·legacy 제거 검증 | launch 변경 시 함께 갱신 |
| `real/ros2_ws/src/kobot_vehicle/test/test_parsers.py` | Kobot 차량 실행본 (2026-08-13) | serial parser 단위 테스트 | 예제 frame → parser 결과 | 검토·테스트 후 수정 가능 |
| `real/ros2_ws/src/kobot_vehicle/test/test_teleop_launch.py` | Kobot 프로젝트 자체 작성 | 상위 launch 조합·잠금 기본값 회귀 테스트 | launch source → 안전 계약 검증 | launch 변경 시 함께 갱신 |
| `real/ros2_ws/src/vesc/.github/workflows/ros2-ci.yaml` | F1TENTH vesc upstream 153998df | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/.gitignore` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/LICENSE` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/README.md` | F1TENTH vesc upstream 153998df | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc/CHANGELOG.rst` | F1TENTH vesc upstream 153998df | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc/CMakeLists.txt` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc/package.xml` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/CHANGELOG.rst` | F1TENTH vesc upstream 153998df | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/CMakeLists.txt` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/include/vesc_ackermann/ackermann_to_vesc.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/include/vesc_ackermann/vesc_to_odom.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/launch/ackermann_to_vesc_node.launch.xml` | F1TENTH vesc upstream 153998df | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/launch/vesc_to_odom_node.launch.xml` | F1TENTH vesc upstream 153998df | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/package.xml` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp` | F1TENTH vesc upstream 153998df | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_ackermann/src/vesc_to_odom.cpp` | F1TENTH vesc upstream 153998df | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/CHANGELOG.rst` | F1TENTH vesc upstream 153998df | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/CMakeLists.txt` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/crc.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/datatypes.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/vesc_device_uuid_lookup.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/vesc_driver.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/vesc_interface.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/vesc_packet.hpp` | F1TENTH vesc 153998df + Kobot FW 6.5 patch | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 패치 diff·실장 검증과 함께 수정 |
| `real/ros2_ws/src/vesc/vesc_driver/include/vesc_driver/vesc_packet_factory.hpp` | F1TENTH vesc upstream 153998df | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/launch/vesc_driver_node.launch.py` | F1TENTH vesc upstream 153998df | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/package.xml` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/params/vesc_config.yaml` | F1TENTH vesc upstream 153998df | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/scripts/99-vesc6.rules` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/scripts/vesc_device_lookup` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/src/vesc_device_namer.cpp` | F1TENTH vesc upstream 153998df | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/src/vesc_device_uuid_lookup.cpp` | F1TENTH vesc upstream 153998df | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_driver/src/vesc_driver.cpp` | F1TENTH vesc 153998df + Kobot FW 6.5 patch | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 패치 diff·실장 검증과 함께 수정 |
| `real/ros2_ws/src/vesc/vesc_driver/src/vesc_interface.cpp` | F1TENTH vesc 153998df + Kobot FW 6.5 patch | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 패치 diff·실장 검증과 함께 수정 |
| `real/ros2_ws/src/vesc/vesc_driver/src/vesc_packet.cpp` | F1TENTH vesc 153998df + Kobot FW 6.5 patch | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 패치 diff·실장 검증과 함께 수정 |
| `real/ros2_ws/src/vesc/vesc_driver/src/vesc_packet_factory.cpp` | F1TENTH vesc upstream 153998df | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/CHANGELOG.rst` | F1TENTH vesc upstream 153998df | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/CMakeLists.txt` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/msg/VescImu.msg` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/msg/VescImuStamped.msg` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/msg/VescState.msg` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/msg/VescStateStamped.msg` | F1TENTH vesc upstream 153998df | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/vesc/vesc_msgs/package.xml` | F1TENTH vesc upstream 153998df | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/CMakeLists.txt` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/config/.gitkeep` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | CMake 설치 대상인 빈 config 디렉터리 보존 | Git checkout → config 디렉터리 | 디렉터리가 필요하므로 유지 |
| `real/ros2_ws/src/xycar_lidar/LICENSE.txt` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | upstream 문서 | 개발자 입력 → 사용·API 설명 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/README.md` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | upstream 문서 | 개발자 입력 → 사용·API 설명 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/.gitignore` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/Doxyfile` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/LICENSE.txt` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/README.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/README.pdf` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/FindPackage.cmake.in` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/PackageConfig.cmake.in` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/PackageConfigVersion.cmake.in` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/PkgConfig.pc.in` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/cmake_uninstall.cmake.in` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/common/ydlidar_base.cmake` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/common/ydlidar_parse.cmake` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/install_package.cmake` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/cmake/script_show_final_summary.cmake` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/datatype.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/locker.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/thread.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/timer.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/timer.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/typedef.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/utils.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/v8stdint.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/base/ydlidar.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/ChannelDevice.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/DriverInterface.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/ydlidar_datatype.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/ydlidar_def.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/ydlidar_def.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/ydlidar_help.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/common/ydlidar_protocol.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/json/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/json/cJSON.c` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/json/cJSON.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/math/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/math/angles.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/ActiveSocket.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/ActiveSocket.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/PassiveSocket.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/PassiveSocket.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/SimpleSocket.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/SimpleSocket.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/network/StatTimer.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/common.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/list_ports_linux.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/lock.c` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/lock.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/unix.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/unix_serial.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/unix/unix_serial.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/windows/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/windows/list_ports_win.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/windows/win.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/windows/win_serial.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/impl/windows/win_serial.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/serial.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/core/serial/serial.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/csharp/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/csharp/examples/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/csharp/examples/Program.cs` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/csharp/ydlidar_sdk.i` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/Dataset.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/Diagram.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/General_FAQs.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/General_FAQs_cn.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/Hardware_FAQs.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/Hardware_FAQs_cn.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/README.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/Software_FAQs.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/FAQs/Software_FAQs_cn.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/README.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/SDK.pdf` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/Tutorials.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/YDLIDAR_SDK_API_for_Developers.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/YDLidar-SDK-Communication-Protocol.html` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/YDLidar-SDK-Communication-Protocol.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/YDLidar-SDK-Communication-Protocol.pdf` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/YDLidar_SDK_Communication_Protocol.pdf` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/README.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_build_and_debug_using_vscode.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_build_and_install.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_create_a_csharp_project.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_create_a_pull.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_create_a_udev_rules.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_gerenrate_vs_project_by_cmake.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/how_to_solve_slow_pull_from_cn.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/cpack.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/csharp_build.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/csharp_exception.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/csharp_library.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/csharp_project.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/csharp_running.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/sdk_init.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/sdk_scanning.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/sequence.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/ttyUSB0.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/ttyUSB1.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/vscode/run_tasks.jpg` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/vscode/tasks.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/howto/images/ydlidar_lidar_monitor.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/EAI.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/FlowChart.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/YDLidar.jpg` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/YDLidar.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/angle_q2.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/communication_mechanism.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/frame.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/frame_intensity.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/sdk_architecture.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/sdk_init.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/sdk_scanning.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/sequence.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/images/system_workflow.png` | YDLidar-SDK upstream 16ca1f40 | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/quickstart/README.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/quickstart/ydlidar_sdk_software_installation_guide.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/tutorials/examine_the_simple_lidar_tutorial.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/tutorials/writing_lidar_network_adapter_tutorial_c++.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/tutorials/writing_lidar_tutorial_c++.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/tutorials/writing_lidar_tutorial_c.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/doc/tutorials/writing_lidar_tutorial_python.md` | YDLidar-SDK upstream 16ca1f40 | upstream 문서 | 개발자 입력 → 사용·API 설명 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/data/lidar0.csv` | YDLidar-SDK upstream 16ca1f40 | upstream 예제 데이터 | 예제 입력 → 테스트·시연 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/data/lidar1.csv` | YDLidar-SDK upstream 16ca1f40 | upstream 예제 데이터 | 예제 입력 → 테스트·시연 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/data/lidar2.csv` | YDLidar-SDK upstream 16ca1f40 | upstream 예제 데이터 | 예제 입력 → 테스트·시연 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/et_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/gs_ota.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/gs_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/gs_test2.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/lidar_c_api_test.c` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/scl_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/sdm18_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/sdm_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tea_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tia_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tmini_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tof_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tri_and_gs_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tri_restart.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/examples/tri_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/etlidar_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/gs_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/plot_tof_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/plot_ydlidar_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/tea_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/tminiplus_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/tof_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/examples/tri_test.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/numpy.i` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/test/pytest.py` | YDLidar-SDK upstream 16ca1f40 | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/python/ydlidar_sdk.i` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/setup.py` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/CYdLidar.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/CYdLidar.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/DTSLidarDriver.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/DTSLidarDriver.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/ETLidarDriver.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/ETLidarDriver.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/GSLidarDriver.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/GSLidarDriver.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/SDMLidarDriver.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/SDMLidarDriver.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/TiaLidarDriver.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/TiaLidarDriver.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/YDlidarDriver.cpp` | YDLidar-SDK 16ca1f40 + checksum log patch | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 패치 diff·LiDAR 검증과 함께 수정 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/YDlidarDriver.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/filters/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/filters/FilterInterface.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/filters/NoiseFilter.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/filters/NoiseFilter.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/filters/StrongLightFilter.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/filters/StrongLightFilter.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/ydlidar_sdk.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/src/ydlidar_sdk.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/startup/initenv.sh` | YDLidar-SDK upstream 16ca1f40 | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/test/CMakeLists.txt` | YDLidar-SDK upstream 16ca1f40 | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/test/lidar_test.cpp` | YDLidar-SDK upstream 16ca1f40 | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/test/lidar_test.h` | YDLidar-SDK upstream 16ca1f40 | C/C++ 공개·내부 선언 | compiler 입력 → API·자료구조 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/YDLidar-SDK/ydlidar_config.h.in` | YDLidar-SDK upstream 16ca1f40 | 빌드 보조 설정 | CMake 입력 → 생성·설치 설정 | 직접 수정 지양; upstream 갱신으로 관리 |
| `real/ros2_ws/src/xycar_lidar/details.md` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | upstream 문서 | 개발자 입력 → 사용·API 설명 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/images/EAI.png` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/images/YDLidar.jpg` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/images/cmake_error.png` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/images/finished.png` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/images/rviz.png` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/images/view.png` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 문서·UI 이미지 자산 | 문서/UI 입력 → 시각 자료 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/launch/xycar_lidar.launch.py` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/launch/xycar_lidar_viewer.launch.py` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/launch/ydlidar.py` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | Python node·도구·예제 | Python/ROS 입력 → runtime·검증 출력 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/package.xml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 빌드·패키지 메타데이터 | 빌드 도구 입력 → library/node/package | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/G1.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/G2.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/G6.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/GS2.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/TEA.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/TG.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/TminiPro.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/X2.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/X4-Pro.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/X4.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/ydlidar.original.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/params/ydlidar.yaml` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 실행·launch 설정 | node/launch 입력 → runtime 파라미터 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/rviz/ydlidar.rviz` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | ROS launch·시각화 설정 | ROS 도구 입력 → 노드·화면 구성 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/src/xycar_lidar_client.cpp` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/src/xycar_lidar_node.cpp` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/src/ydlidar_ros2_driver_client.cpp` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/src/ydlidar_ros2_driver_node.cpp` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | C/C++ 구현 소스 | 장치·ROS·library 입력 → runtime 동작 | 실장·빌드 검증 후 수정 가능 |
| `real/ros2_ws/src/xycar_lidar/startup/initenv.sh` | Kobot LiDAR ROS 2 wrapper (2026-08-13) | 모듈 지원 파일 | 빌드·문서·runtime 지원 | 실장·빌드 검증 후 수정 가능 |
| `real/system/env/kobot_env.sh.example` | Kobot 프로젝트 자체 작성 | ROS 2 환경 예제 | shell source → ROS 환경변수 | 검토·검증 후 수정 가능 |
| `real/system/systemd/kobot-sensors.service.example` | Kobot 프로젝트 자체 작성 | 센서 dashboard 서비스 예제 | systemd → sensors.launch.py | 검토·검증 후 수정 가능 |
| `real/system/sudoers/kobot-sensors` | Kobot 프로젝트 자체 작성 | 센서 dashboard 서비스 제한 NOPASSWD 규칙 | xytron 운영 명령 → kobot-sensors.service start/stop/restart | root 0440·visudo 검사 후 설치; 명령 범위 확대 금지 |
| `real/system/udev/10-kobot.rules` | Kobot 프로젝트 자체 작성 | USB 장치 고정 별칭 규칙 | USB VID/PID → /dev 별칭 | 검토·검증 후 수정 가능 |
