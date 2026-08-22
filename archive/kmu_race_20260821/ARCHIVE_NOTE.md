# kmu_race 아카이브 (2026-08-21 작업분)

## 출처
`/home/xytron/xycar_ws/src/kmu_race` 를 그대로 복사한 것이다.
복사 시각: 2026-08-22. 원본은 그대로 두었다.

## 왜 여기에 넣었나
`xycar_ws` 는 **git 저장소가 아니다.** 금요일(2026-08-21) 새벽 00:00~08:00 사이에
이 패키지의 소스 약 40개가 수정됐는데 버전관리 밖에 있어 유실 위험이 있었다.
팀원 4명이 각자 컴퓨터에서 Jetson을 직접 편집하는 환경이라 더 위험했다.

## 무엇이 들어 있나
- `kmu_race/perception/` — cone_detector, lidar_cones, signal_watcher, signal_detector,
  lane_detector, lidar_tracker, lamp_pitch
- `kmu_race/core/` — corridor, race_fsm, policy, speed_policy, control, heading,
  safety, target_passing, race_context, types
- `kmu_race/io/`, `kmu_race/nodes/`, `kmu_race/tools/`
- `launch/race.launch.py`, `launch/debug.launch.py`, `config/params.yaml`, `test/`

## 주의
이 경로는 `real/ros2_ws/src` **밖**이므로 colcon 이 빌드하지 않는다.
실행 중인 차량 동작에 영향을 주지 않도록 의도한 배치다.
실제로 쓰려면 `real/ros2_ws/src` 로 옮기고 빌드해야 한다.

원본과 동일함을 확인한 수동 백업본: `~/kmu_src_backup_0821_0634`, `~/kmu_src_backup_0821_0747`
