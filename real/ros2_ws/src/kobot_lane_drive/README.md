# kobot_lane_drive

차선 인식과 주행 판단을 담당하는 패키지다. 실차의 주행 로직은 전부 여기 있다.

주행 규칙 전문은 [../../../docs/DRIVING_POLICY.md](../../../docs/DRIVING_POLICY.md).

## 구조

```text
kobot_lane_drive/
├── geometry.py                        레이트 리밋 · 클램프 · 저역통과
├── twinmixing_center.py               마스크 → 중심 경로, 화면 오버레이
├── yellow_center.py                   노란 중앙선 검출 (LAB top-hat) + 끊긴 동안 유지
├── lane_policy.py                     차선 폭 측정 + 주행 라인 상태기계
├── obstacle_lidar.py                  /scan → 차량 좌표계 코리도 거리
├── drive_command.py                   조향·속도 → /xycar_motor 형식 변환
├── twinmixing_center_node.py          ROS 노드 (인식 + 판단, 모터 없음)
├── twinmixing_center_drive_bridge_node.py  승인 잠금 모터 브리지
└── recording_lane_viewer.py           :8001 녹화 재생 분석 뷰어
```

위 다섯 모듈(`geometry` ~ `obstacle_lidar`)은 ROS 에 의존하지 않는다.
차 없이 `test/` 의 단위 시험으로 검증한다.

## 한 프레임에 벌어지는 일

```text
/image_raw ──> TwinMixing ──> driving_mask (주행영역) + lane_mask (차선)
                                  │
       yellow_center.py ◄─────────┤  원본 프레임의 LAB b* 채널도 함께 본다
         │  사다리꼴 ROI 안에서 가로 top-hat → 좁은 노란 줄만 남긴다
         │  도로에서 40px 넘게 떨어진 후보는 버린다 (나무벽)
         │  끊기면 직전 목표를 1초까지 유지 (점선·순간 소실 대응)
         ▼
    목표선 선택  1) 노란 중앙선  1') 직전 노란선 1초 유지
                 2) 센터 갭  3) 영역 중앙  4) HOLD
         │
    lane_policy.py ── 목표선을 경계로 좌/우 차선 폭 측정
         │            + obstacle_lidar.py 의 옆 차선 코리도
         │            → CENTER / SHIFT_LEFT / SHIFT_RIGHT / RETURNING
         ▼
    목표 x (레이트 리밋) ──> 조향 = P(가로오차) + D(오차변화율), 여유구간 25px
         ▼
    drive_preview ──> (armed 일 때만) 모터 브리지 ──> /xycar_motor
```

## 실행

상시 인식은 `kobot_vehicle/sensors.launch.py` 가 띄운다. 이 패키지 단독으로
확인할 때는 다음을 쓴다.

```bash
# 무모터. 인식과 화면만
ros2 launch kobot_lane_drive twinmixing_center_record_drive.launch.py armed:=false

# 회피까지 켜서 확인 (여전히 무모터)
ros2 launch kobot_lane_drive twinmixing_center_record_drive.launch.py \
  armed:=false maneuvers_enabled:=true
```

`armed:=true` 는 실제 모터 출력을 만든다. 승인과 충돌 점검 뒤에만 실행한다.
보통은 웹 대시보드의 `기록 주행` 버튼이 이 launch 를 대신 실행한다.

## 토픽

| 토픽 | 내용 |
|---|---|
| `/kobot_lane_drive/twinmixing_center/debug_image` | 오버레이 영상 |
| `/kobot_lane_drive/twinmixing_center/status` | 상태 JSON (목표선 출처, 주행 라인, 차선 폭, 라이다 거리, 제어 입력) |
| `/kobot_lane_drive/twinmixing_center/drive_preview` | `[steering_rad, speed_mps, confidence]` |
| `/kobot_lane_drive/record_drive/enabled` | 웹 일시정지/재개 (transient-local `Bool`) |

`/xycar_motor` 를 발행하는 파일은 `twinmixing_center_drive_bridge_node.py`
하나뿐이다. `test_motor_free.py` 가 이를 강제한다.

## 설정

모든 튜닝값은 `config/twinmixing_center.yaml` 한 파일에 있다. **소스에 숫자를
새로 박지 않는다.** 그 파일 주석에 각 값의 근거까지 적혀 있다.

현장에서 손댈 일이 있는 값은 이 정도다.

```yaml
yellow_lab_margin: 14.0                   # 노란선이 안 잡히면 낮춘다 (오검출은 늘어난다)
yellow_maximum_road_distance_ratio: 0.0625  # 나무벽이 잡히면 낮춘다 (40px)
steering_output_gain: 3.2                 # 곡선을 못 돌면 올린다
steering_deadband_px: 25.0                # 직선에서 흔들리면 올린다
yellow_ghost_maximum_seconds: 1.0         # 노란선이 끊긴 뒤 직전 목표를 유지할 시간
maneuvers_enabled: false                  # 회피. 기본 꺼짐
policy_straight_only: true                # 회피는 직선에서만
```

**노란선이 안 잡힐 때 여기부터 만지면 안 된다.** 이 검출기는 색을 보므로
카메라 화이트밸런스·노출이 먼저다. 화이트밸런스 2800 / 노출 40 / 밝기 0 을
확인한 뒤에 임계를 손댄다 (`../../../docs/OPERATIONS.md` 5절).

## 시험

```bash
python3 -m pytest test -q   # 54개
```

| 파일 | 무엇을 지키나 |
|---|---|
| `test_yellow_center.py` | 노란선 검출, 점선 외삽, 나무벽 거르기, 오프셋 부호 |
| `test_lane_policy.py` | 차선 폭 측정, 상태 전이, 오검출 방지 |
| `test_obstacle_lidar.py` | 코리도 안팎 판정, 벽 배제, 낡은 스캔 거부 |
| `test_twinmixing_center.py` | 센터 갭·영역 중앙 폴백, 얇은 틈 메우기 |
| `test_motor_free.py` | 인식 계층에 모터 publisher 가 없음 |
| `test_imports.py` | 모듈을 옮기다 import 가 끊기는 것 (실제로 한 번 났다) |

`test_imports.py` 는 모든 모듈을 실제로 import 하고, 브리지가 import 하는 이름을
AST 로 확인한다. `drive_command.py` 를 legacy 로 옮겼다가 브리지가 죽은 적이 있어
넣었다.

## 차 없이 검증하기

```bash
ros2 run kobot_lane_drive recording_lane_viewer --prepare-session <세션> --prepare-only
```

지난 녹화에 현재 인식 코드를 그대로 돌린다. 결과는
`~/recordings/camera/<세션>/_twinmixing_replay/` 에 남는다.
`--prepare-only` 를 빼면 `:8001` 에서 원본과 오버레이를 나란히 재생한다.

주의: 이 스크립트는 CUDA 모델을 한 번 더 적재한다. **주행 중이나 상시 인식이
살아 있는 동안 돌리면 인식 노드가 멈추는 사례가 있었다** (3회 재현, 원인 미확정).

## 출처

`third_party/TwinMixing` 은 외부 원본이다. 자세한 내용은
[../../../docs/PROVENANCE.md](../../../docs/PROVENANCE.md).
