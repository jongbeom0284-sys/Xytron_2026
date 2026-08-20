# Cone racing snapshot — 2026-08-21

실차 라바콘 통로를 안정적으로 완주한 시점의 백업이다. 부모 버전은
`v1.0-stable-safe` (`c7ee720`)이며, 현재 라바콘 레이싱 코드와 모터 가속도
설정을 그 위에 보존한다.

## Active values

- Straight speed: `1.50 m/s`
- Corner speed: `1.00 m/s`
- Single-boundary fallback speed: `1.00 m/s`
- Lost-path continuation speed: `1.00 m/s`
- Motor acceleration limit: `4.0 m/s^2`
- Left-turn outward path bias: `0.20 m`
- Right-turn outward path bias: `0.15 m`
- LiDAR-to-rear-axle offset: `0.42 m`
- Stop on cone-path loss: disabled after the first trusted path
- Sharp lost hold time: `0.00 s`

ROS/DDS command freshness, enable state, and motor bridge watchdog stops remain
enabled. A vehicle that has never acquired a valid cone path does not blind-start.

## Rollback values

- Single-boundary fallback speed: `0.50 m/s`
- Lost-path continuation speeds: `0.50 m/s`
- Motor acceleration limit: `2.5 m/s^2`
- Timed stop on path loss: set `cone_path_stop_on_lost: true`
- Previous sharp lost hold time: `0.12 s`

## Verification

- Jetson `kobot_lane_drive`: `171 passed`
- Local backup overlay: `168 passed, 3 skipped` (ROS imports unavailable locally)
- Jetson `kobot_vehicle`: build passed; motor launch default verified as
  `acceleration_limit_mps2=4.0`
