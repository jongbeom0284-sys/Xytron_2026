# 출처와 변경 이력

## 기준 스냅샷

이 기준판은 2026-08-13 `kobot`의 `/home/xytron/xycar_ws/src`에서 실행·검토한 소스를 가져왔다. import 시 `.git`, `build`, `install`, `log`, Python/pytest cache, `*.bak`, `*.before_*`, 임시 출력은 제외했다.

| 모듈 | 출처 | 기준 |
|---|---|---|
| `kobot_vehicle` | Kobot 차량용 자체 작성 ROS 2 패키지 | 2026-08-13 Jetson 실행본 |
| `kobot_lane_drive` | Kobot 통합 + 공식 공개 구현 | CLRerNet `dae038f` + IPM `2943925` + PythonRobotics `1fe4fb9` + Shapely 2.1.2 |
| `ipm` | https://github.com/ros-sports/ipm | Humble `29439254c6c1eaabfaf1e8a42345594ffd55f3b5` |
| `vesc` | https://github.com/f1tenth/vesc | `153998df8545fe1781b975df88e411b4e71d4bfe` + 아래 4파일 패치 |
| `xycar_lidar/YDLidar-SDK` | https://github.com/YDLIDAR/YDLidar-SDK | `16ca1f402453d1c094ada83e94200951ed2f711b` + checksum 로그 억제 1줄 |
| `xycar_lidar` ROS wrapper | Kobot에서 사용 중인 ROS 2 wrapper | 2026-08-13 Jetson 실행본 |

## Kobot/Codex 수정 범위

### `kobot_vehicle`

센서 launch와 dashboard, 웹 WASD heartbeat, 시간 주행, motor API, 속도·가속도 제한, watchdog, `lane_preview`와 테스트는 차량 프로젝트 자체 코드다. 이번 저장소 반영에서 `dashboard/app.js`의 잘못된 최대속도 표시, input `max`, client validation을 `8.00`에서 실제 상한 `2.16 m/s`로 맞췄다. `dashboard_server.py`의 firmware 설명도 실제 FW 6.5로 정정했다.

### `vesc`

upstream 기준에서 바뀐 파일은 다음 네 개뿐이다.

- `vesc_driver/include/vesc_driver/vesc_packet.hpp`
- `vesc_driver/src/vesc_driver.cpp`
- `vesc_driver/src/vesc_interface.cpp`
- `vesc_driver/src/vesc_packet.cpp`

변경 목적은 Kobot VESC FW 6.5의 packet/firmware 응답 호환이다. 나머지 파일은 vendored upstream이다.

### `xycar_lidar`

YDLidar SDK 기준 commit에서 의미 있는 내용 변경은 `YDLidar-SDK/src/YDlidarDriver.cpp`의 반복 checksum 오류 로그 1줄 억제다. Jetson 작업트리에서 보였던 실행권한(mode-only) 차이와 `examples/data/lidar*.csv` mode 차이는 가져오지 않았다.

### `kobot_lane_drive`

`/home/xytron/clrernet_eval`에서 확인한 CLRerNet CULane DLA34 EMA 평가 환경을 최종 저장소의 새 ROS 2 패키지로 이식했다. upstream `libs/`, `configs/`, 라이선스와 문서만 포함하고 가상환경, 데이터셋, 빌드 결과와 가중치는 제외했다.

Jetson PyTorch 2.8 호환을 위해 CUDA NMS 두 소스만 패치했다. 거리/BEV/평행 경로/조향은 직접 구현하지 않고 각각 ros-sports/ipm, Shapely `offset_curve`, PythonRobotics Stanley를 호출한다. 프로젝트 코드는 ROS 인터페이스와 안전 차단, 승인 잠금 모터 형식 adapter만 담당한다.

## 장치 설정 자료

`config/vesc_tool/` XML은 2026-08-13 VESC Tool readback이다.

- motor: FW 6.5, speed-controller `Ki=0.0015`
- app: servo output enabled

XML은 참고·비교용이고 자동 적용되지 않는다. 설정을 다시 변경하면 날짜, 장치 HW/FW, 변경 전후 readback, fault 상태를 이 문서에 추가한다.

## 제외 범위

과거 `track_drive`, `kmu_race`, `real/drivers`, `lane_id_perception`, LiDAR 예제 CSV와 임시 로그·백업은 포함하지 않는다. `lane_preview`와 `kobot_lane_drive` 출력은 실제 차선 정확도나 미터 단위 거리 검증을 통과했다는 뜻이 아니다.
