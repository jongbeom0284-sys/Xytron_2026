# 라바콘 주행 오프라인 개발본 인수인계

## 현재 상태

- 작업 위치: `tmp/cone-natural-fix`
- 실차 Jetson, 센서, 모터, 대시보드 프로세스는 이번 개발 중 실행하거나 종료하지 않았다.
- 이 디렉터리는 독립 Git 저장소가 아니라, 실차 코드를 안전하게 검토·수정하기 위한 선택 파일 묶음이다.
- 최종 로컬 검증: `155 passed, 3 skipped`, 핵심 Python 파일 compile 통과, `scripts/cone_active_run.sh`의 `bash -n` 통과.
- 실차에는 아직 배포하지 않았다. 팀원이 수정한 현재 Jetson 소스와 먼저 비교한 뒤 선택 파일만 반영해야 한다.

## 이번 개발의 주행 로직

1. 라바콘 클러스터 임계는 원거리 탐지를 위해 최소 반사점 2개, 전체 반사점 3개를 유지한다.
2. 좌우 경계는 현재 차량 앞의 입구 단면을 먼저 고정한 뒤 전방으로 연결한다.
3. 같은 쪽 가구·벽 모서리 열이 더 길다는 이유만으로 실제 경계를 이기지 못하도록 root 위치, 폭, 진행 방향을 함께 평가한다.
4. staggered cone에서는 greedy 한 갈래 대신 제한된 beam search로 경계 연결 후보를 비교한다.
5. 목표점은 LOCAL 경로를 기본 안전 기준으로 사용한다. POLY2가 local 목표의 위치·부호·heading과 일치할 때만 곡선 선행 목표로 채택한다.
6. 정상 S자처럼 매 프레임 일관되게 이동하는 후보는 급변 가드가 따라간다. 좌우로 번갈아 튀는 후보는 이전 신뢰 경로 주변의 제한된 preview만 내며 누적 추격하지 않는다.
7. 불안정하지만 `found=true`인 후보 때문에 자동 정지하지 않는다. 실제 `found=false`일 때만 기존 LOST hold/stop 계약을 사용한다.
8. 새 경로로 전환 중에는 조향·목표가 수렴하기 전에 속도만 먼저 0.8 m/s로 올라가지 않는다. 감속은 즉시, 가속은 경로 수렴 뒤 적용한다.

## 독립 robust 프로필

`config/cone_corridor_robust.yaml`이 이번 시험의 단일 기준이다.

- 유효 전방 거리: 0.10~2.00 m
- 기본 통로 폭: 0.80 m, 유효 폭: 0.60~1.05 m
- cluster: 최소 점 2, 최소 span 0.03 m
- 경계 link: 근거리 0.35 m, 원거리 최대 0.58 m
- paired root: 중심 오차 0.25 m, 폭 오차 0.15 m, 최대 y 0.75 m, 최소 pair 2
- lookahead: 직선 0.90 m, 코너 0.62 m
- 속도: 직선 0.80, 코너 0.70, 단측 fallback 0.50 m/s
- 조향: gain 1.9, filter alpha 0.75, rate 5.0 rad/s
- 후보 확인: 2 frame
- 일반 LOST hold: 0.35 s / 0.50 m/s, 급조향 LOST hold: 0.12 s / 0.30 m/s
- bridge 실제 속도 상한: 0.80 m/s

첫 실차 기록은 launch hard cap을 `0.55 m/s`로 둔다. 이 경우 직선·코너는 모두 최대 0.55, fallback은 0.50, 급조향 LOST hold는 0.30 m/s다.

## 선택 배포 대상

실차의 현재 파일과 diff를 확인한 다음 아래 파일만 선택 반영한다.

### 주행 및 런타임

- `kobot_lane_drive/cone_corridor.py`
- `kobot_lane_drive/cone_path_guard.py`
- `kobot_lane_drive/cone_corridor_drive_node.py`
- `kobot_lane_drive/drive_command.py`
- `kobot_lane_drive/enable_lease.py` (신규)
- `kobot_lane_drive/twinmixing_center_drive_bridge_node.py`
- `launch/cone_corridor_drive.launch.py`
- `config/cone_corridor_robust.yaml` (신규)
- `setup.py`

### 회귀 테스트

- `test/test_cone_corridor.py`
- `test/test_cone_path_guard.py`
- `test/test_enable_lease.py`
- `test/test_runtime_contract.py`
- `test/test_active_run_endpoint_guards.py`

### 실차 활성 구간 기록 도구

- `scripts/cone_active_run.py`
- `scripts/cone_active_run.sh`
- `ACTIVE_RUN_LOGGING.md`

## 덮어쓰지 않을 것

- 팀 공용 `config/twinmixing_center.yaml`은 이번 배포에서 덮어쓰지 않는다. 개발본 안의 변경은 계약 테스트용이며, 새 launch는 독립 `cone_corridor_robust.yaml`을 기본으로 사용한다.
- `launch/twinmixing_center_record_drive.launch.py`는 로컬 테스트 fixture 완성을 위해 복사한 기존 파일이므로 선택 배포 대상이 아니다.
- `__pycache__`, `.pytest_cache`, build/install/log 산출물은 배포하지 않는다.
- 팀원이 실차에서 변경한 다른 차선·통합 런치·대시보드 파일은 건드리지 않는다.

## 기록 실행의 안전 계약

`scripts/cone_active_run.py`는 프로세스를 무차별 종료하지 않는다.

- 기존 enable, motor, cone status, cone preview publisher가 있으면 출발 전에 실패한다.
- recorder/planner/bridge의 DDS GID를 launch 중 고정하고, 추가·교체·소실되면 즉시 persistent `false`를 발행하고 중단한다.
- rosbag이 여섯 토픽에 실제 연결된 뒤에만 출발할 수 있다.
- 기본 출발 조건은 `TRACKING + found=true + motor_output=false`가 status gap 0.25초 이하로 0.5초 연속 유지되는 것이다.
- 하네스 실행에서만 enable heartbeat lease 0.30초를 켠다. SSH 단절이나 하네스 `SIGKILL`에도 bridge가 lease 만료 후 20 Hz로 모터 0을 발행한다.
- 정상 종료나 Ctrl+C에서는 `enable=false` 이후 실제 `/xycar_motor [*, 0]` 수신을 확인하고 자신이 만든 launch/rosbag process group만 종료한다.
- 일반 launch의 heartbeat 기본값은 0.0이므로 기존 대시보드 one-shot enable 동작은 유지된다.
- VESC `fault_code` 자동 해석·중단은 아직 포함하지 않는다. `/sensors/core` 원본은 bag에 저장한다.

## 차량이 비었을 때 다음 순서

1. Jetson의 현재 소스를 새로 백업하고 이 개발본과 diff한다.
2. 위 선택 배포 대상만 별도 test workspace에 반영한다.
3. `kobot_lane_drive`와 의존 패키지를 build하고 전체 pytest를 다시 실행한다.
4. 모터를 끈 preview에서 좌우 경계, 선택 경로, target, speed cap을 확인한다.
5. 센서·모터 스택을 기존 검증 명령으로 각각 실행한다.
6. `bash ./scripts/cone_active_run.sh --duration 15 --speed 0.55`로 첫 기록 주행을 수행한다.
7. `summary.json`, rosbag, status/preview/motor JSONL과 실제 충돌 위치를 함께 분석한다.
8. 첫 주행에서 경계 association이 정상임을 확인한 뒤 0.60 → 0.70 → 0.80 m/s 순서로 올린다. 인식·경로 오류가 있으면 속도부터 올리지 않는다.
