# kobot_lane_drive

차선 인식과 주행 판단을 담당하는 패키지다. 실차의 주행 로직은 전부 여기 있다.

주행 규칙 전문은 [../../../docs/DRIVING_POLICY.md](../../../docs/DRIVING_POLICY.md).

## 구조

```text
kobot_lane_drive/
├── geometry.py                        레이트 리밋 · 클램프 · 저역통과
├── twinmixing_center.py               마스크 → 중심 경로, 화면 오버레이
├── yellow_center.py                   노란 중앙선 검출 + 점선 외삽
├── lane_policy.py                     차선 폭 측정 + 주행 라인 상태기계
├── obstacle_lidar.py                  /scan → 옆 차선 코리도 거리
├── twinmixing_center_node.py          ROS 노드 (인식 + 판단, 모터 없음)
├── twinmixing_center_drive_bridge_node.py  승인 잠금 모터 브리지
└── recording_lane_viewer.py           :8001 녹화 재생 분석 뷰어
```

위 다섯 모듈(`geometry` ~ `obstacle_lidar`)은 ROS 에 의존하지 않는다.
차 없이 `test/` 의 단위 시험으로 검증한다.

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
| `/kobot_lane_drive/twinmixing_center/status` | 상태 JSON (목표선 출처, 주행 라인, 차선 폭, 라이다 거리) |
| `/kobot_lane_drive/twinmixing_center/drive_preview` | `[steering_rad, speed_mps, confidence]` |
| `/kobot_lane_drive/record_drive/enabled` | 웹 일시정지/재개 (transient-local `Bool`) |

`/xycar_motor` 를 발행하는 파일은 `twinmixing_center_drive_bridge_node.py`
하나뿐이다. `test_motor_free.py` 가 이를 강제한다.

## 설정

모든 튜닝값은 `config/twinmixing_center.yaml` 한 파일에 있다. 소스에 숫자를
새로 박지 않는다. 현장에서 가장 먼저 볼 것은 노란색 HSV 임계다.

```yaml
yellow_hue_low: 15          # 조명이 바뀌면 노출과 함께 다시 맞춘다
yellow_saturation_low: 55
yellow_value_low: 110
maneuvers_enabled: false    # 회피. 기본 꺼짐
policy_straight_only: true  # 회피는 직선에서만
```

## 시험

```bash
python3 -m pytest test -q   # 37개
```

| 파일 | 무엇을 지키나 |
|---|---|
| `test_yellow_center.py` | 노란선 검출, 점선 외삽, 오프셋 부호 |
| `test_lane_policy.py` | 차선 폭 측정, 상태 전이, 오검출 방지 4종 |
| `test_obstacle_lidar.py` | 코리도 안팎 판정, 벽 배제, 낡은 스캔 거부 |
| `test_twinmixing_center.py` | 센터 갭·영역 중앙 폴백 |
| `test_motor_free.py` | 인식 계층에 모터 publisher 가 없음 |

## 차 없이 검증하기

```bash
ros2 run kobot_lane_drive recording_lane_viewer --prepare-session <세션> --prepare-only
```

지난 녹화에 현재 인식 코드를 그대로 돌린다. 결과는
`~/recordings/camera/<세션>/_twinmixing_replay/` 에 남는다.
`--prepare-only` 를 빼면 `:8001` 에서 원본과 오버레이를 나란히 재생한다.

## 출처

`third_party/TwinMixing` 은 외부 원본이다. 자세한 내용은
[../../../docs/PROVENANCE.md](../../../docs/PROVENANCE.md).
