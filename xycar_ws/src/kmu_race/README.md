# kmu_race — 2026 국민대 자율주행 경진대회 (제9회) 본선 주행 스택

본선 2026.08.25 (화) · 3바퀴 · 4분 제한 · 지름길 1회

---

## 1. 규정 → 코드 매핑

구현의 근거를 전부 규정 페이지에 붙여 놓았다. 규정이 바뀌면
(주최측이 "대회 직전까지 변경될 수 있다"고 명시 — p.3) 아래 표에서 해당 파일만 찾아 고치면 된다.

| 규정 | 조항 | 구현 |
|---|---|---|
| p.17~20 | 4구 신호등 파란불 출발, 빨간불 출발 시 벌초 10초 | [`perception/signal_detector.py`](kmu_race/perception/signal_detector.py) · `FSMConfig.green_confirm_frames` |
| p.21 | **주행 멈춘 후 1분 내 재개 못하면 실격** | [`core/safety.py`](kmu_race/core/safety.py) — 정지보다 크립 우선 · STOP 은 터미널이 아니라 원인 해소 시 복귀 (`FSMConfig.stop_recover_*`) |
| p.23~25 | 라바콘 좌우 두 줄 사이 통과, 충돌 개당 3초 | [`perception/cone_detector.py`](kmu_race/perception/cone_detector.py) — 게이트 탐색 |
| p.27~28 | 1·2차선 구분 없음, 실선만 안 넘으면 됨 | [`perception/lane_detector.py`](kmu_race/perception/lane_detector.py) |
| p.30 | 고정장애물(고장난 차량) 회피 | [`core/target_passing.py`](kmu_race/core/target_passing.py) `moving=False` |
| p.32~33 | **방해차량이 주행하지 않는 차선 쪽으로만 추월** | `TargetPassing.choose_side()` |
| p.32 | 추월 후 뒷차와 90cm 벌어지기 전 차선 복귀 | `PassConfig.return_deadline_m = 0.90` |
| p.34 | 추돌 벌초 10초 (가해·피해 모두) | `PassConfig.min_gap_m` |
| p.35~36 | 좌회전 신호 → 지름길, 2 또는 3바퀴째 랜덤, 1회만 | [`core/race_context.py`](kmu_race/core/race_context.py) `may_take_shortcut()` |
| p.37 | 4분 초과 실격 | [`core/speed_policy.py`](kmu_race/core/speed_policy.py) 시간 압박 보정 |
| p.38 | 통과감지장치 3회 통과 후 종료 | `LapTracker` |

### 4구 신호등 점등 조합 (p.36)

램프 배치는 좌→우 **[빨강, 노랑, 좌회전←, 녹색]** 로 확정돼 있다.
그래서 '색'이 아니라 **'몇 번째 램프가 켜졌나'** 로 판정한다 —
색 기반 판정은 라바콘 주황을 빨강으로 오인하는 알려진 함정이 있다.

```
출발       빨강        → 녹색              직진 출발
2랩 시작   녹색        → 녹색+좌회전       직좌 동시신호 → 지름길 가능
3랩 시작   녹색                            직진만
```

---

## 2. 구조

```
kmu_race/
├── core/          ★ rclpy·cv2 없음. 순수 로직. 실차 없이 pytest 로 전부 검증됨
│   ├── types.py           데이터 계약 (Observation, DriveCommand …)
│   ├── race_context.py    랩 카운팅, 지름길 잠금, 시간 예산
│   ├── race_fsm.py        상태 전이 (규정 미션 순서)
│   ├── safety.py          안전 등급 — 이 대회에선 '정지'가 가장 비싸다
│   ├── target_passing.py  추월/회피 (규정 조항 인용)
│   ├── control.py         퓨어 퍼수트 · PID · 스무더
│   ├── speed_policy.py    모드별 속도 + 곡률 감속 + 4분 예산 보정
│   └── policy.py          모드 → 주행명령
├── perception/    cv2 사용. 센서 → 데이터클래스
├── io/motor.py    Float32MultiArray [angle, speed] → 'xycar_motor'
└── nodes/race_node.py   ★ rclpy 를 아는 유일한 파일
```

**왜 이렇게 나눴나.** 실차 접근 시간이 희소 자원이기 때문이다.
판단 로직이 ROS 에 묶여 있으면 차 앞에서만 디버깅할 수 있다.
`core/` 는 의존성이 없어서 노트북에서 초 단위로 돌려 볼 수 있다 —
지금 78개 테스트가 3초에 끝난다.

---

## 3. 빌드 & 실행

```bash
cd ~/xycar_ws && colcon build --symlink-install --packages-select kmu_race && source install/setup.bash
```

**대회 당일** (규정 p.16 — 실행 명령은 `ros2 launch` 여야 함):

```bash
ros2 launch kmu_race race.launch.py
```

**연습주행 튜닝용** (OpenCV 창 켜짐):

```bash
ros2 launch kmu_race debug.launch.py
```

---

## 3.5 도구 — 실차 시간을 아끼는 두 가지

### 프리플라이트 (차에 처음 붙였을 때)

```bash
ros2 run kmu_race preflight                # 10초 관측
ros2 run kmu_race preflight --motor-test   # 조향 서보도 확인 (speed=0, 차는 안 나감)
```

토픽 존재·타입·레이트·**QoS 일치**를 한 번에 본다. QoS 불일치는 "토픽은 있는데
콜백이 한 번도 안 불리는" 고전 함정이라 따로 잡아 준다. 동시에 이 자리에서
`min_gantry_span_px`, `angle_offset` 실측값을 뽑아 준다.

```
[ 라이다 — angle_offset 실측 ]
  최근접 물체: 0.98m @ +6.3°
     ✅ 정면 물체가 0° 부근 — angle_offset 보정 불필요

[ 신호등 — min_gantry_span_px 실측 ]
  ✅ span  최소 18  중앙 34  최대 51  (212프레임)
```

### bag 리플레이 (그 뒤로는 노트북에서)

```bash
# 차에서 수동주행 3바퀴, 한 번만. raw 는 3바퀴에 4GB+ 라 compressed 로 찍는다.
# (replay 는 raw/compressed 둘 다 읽는다. /imu 를 빼먹으면 요 튜닝이 통째로 막힌다)
ros2 bag record /image_raw/compressed /scan /imu -o run1

ros2 run kmu_race replay run1                   # 퍼셉션 값 요약
ros2 run kmu_race replay run1 --sweep-gantry    # ★ 랩 임계값 자동 탐색
ros2 run kmu_race replay run1 --fsm             # FSM 통째로 재생
ros2 run kmu_race replay run1 --csv out.csv
```

**`--sweep-gantry` 가 핵심이다.** 랩 카운팅은 이 대회 최대 리스크인데
(하나 놓치면 3바퀴 미완주, 하나 더 세면 조기 종료 — 둘 다 실격),
3바퀴 bag 을 넣으면 임계값을 훑어 가며 **정확히 4회 목격**이 나오는 구간을
찾아 준다. 트랙에서 시행착오로 맞출 값이 아니다.

```
    임계    목격  판정
      18      7  중복 카운트 (3회 초과) → 조기 종료
      26      4  ✅ 정확
      34      4  ✅ 정확
      44      2  랩 누락 (2회 놓침) → 3바퀴 미완주

✅ 정확한 구간: 22 ~ 38
   권장값 = 30  (구간 중앙 — 양쪽 여유가 가장 크다)
```

`--fsm` 은 녹화된 실주행을 FSM+정책에 통째로 흘려 상태 전이를 타임라인으로
찍어 준다. 지름길을 제때 잡는지, 3바퀴 후 FINISH 에 도달하는지를
차 없이 확인할 수 있다.

---

## 4. 대회 당일 체크리스트

- [ ] `debug_gui:=false` 확인 — 창을 띄우면 프레임이 밀려 4분 예산이 위태롭다
- [ ] 차량 커버 장착 (p.15, 필수)
- [ ] 오전 경주 전 `speed_to_mps` 재확인 — 배터리 전압에 따라 변한다
- [ ] `min_gantry_span_px` 를 당일 조명에서 재확인 — **랩 카운팅이 여기 달려 있다**
- [ ] 신호등 ROI 확인. 차량 위치를 전후좌우 20cm 안에서 조정 가능 (p.20)
- [ ] 오전/오후 조명이 다르다 → `bright_margin` 을 경주별로 따로 잡아 둘 것
- [ ] 심판에게 화면이 보이도록 노트북 배치 (p.16)

터미널에 1초마다 이렇게 찍힌다:

```
LANE_DRIVE   lap2/3 t= 87.3s steer=  -4.2 v= 19.8 sig=GREEN_LEFT front=2.41m SHORTCUT-USED
```

---

## 5. 실차 튜닝 순서

`config/params.yaml` 의 ★ 항목이 실측 대상이다. **이 순서대로** 하는 게 빠르다.

| 순서 | 항목 | 측정법 | 안 맞으면 |
|---|---|---|---|
| 1 | `speed_to_mps` | 직선에서 speed=15 로 5초 주행, 거리/5/15 | 고정장애물↔방해차량 오판 |
| 2 | `min_gantry_span_px` | 3바퀴 bag → `~/tune_laps.sh` (span = 채도 초록 램프 블롭 폭) | **랩 누락 → 3바퀴 미완주** |
| 3 | `px_per_m` | BEV 에서 알려진 거리의 픽셀 수 | 라바콘 조향 배율 오류 |
| 4 | `default_lane_width_px` | BEV 차선 두 줄 간격 | 한쪽 차선만 볼 때 추정 오류 |
| 5 | 신호등 `min/max_radius` | 정차 위치에서 램프 반지름 | 신호등 미검출 → 출발 실패 |
| 6 | `bright_margin` | 오전·오후 각각 | 오검출 → 빨간불 출발(10초) |
| 7 | 콘 HSV `lower/upper` | 트랙 조명에서 마스크 확인 | 구간 진입 실패 |
| 8 | `pass_offset_px` | 차선 폭 실측값 기준 | 추월 폭 부족/과다 |
| 9 | `lidar angle_offset` | 정면 물체 각도 확인 | 회피 방향 오류 |

**랩 카운팅(2번)이 가장 중요하다.** 하나 놓치면 3바퀴를 못 채우고,
하나 더 세면 2바퀴에 멈춘다. 둘 다 곧바로 실격이다.
연습주행 때 `sightings` 로그를 반드시 세 번 확인할 것.

---

## 6. 테스트

```bash
cd xycar_ws/src/kmu_race && python3 -m pytest test -q
```

ROS 없이 돌아간다 (`cv2`/`numpy` 만 필요, 없으면 퍼셉션 테스트는 자동 skip).

```
78 passed in 3.2s
```

무엇을 검증하는가:

- **`test_race_fsm.py`** — 빨간불에 출발하지 않는가, 지름길을 2랩/3랩 어느 쪽에서든 잡는가, 1회만 쓰는가, 3바퀴 후 종료하는가, 라바콘 구간에서 추월로 새지 않는가
- **`test_target_passing.py`** — **추월 방향 규정(p.33)**, 끼어들기 재전환, 90cm 복귀 시한, 추돌 방지 감속
- **`test_lap_tracker.py`** — 중복 카운트 / 누락 카운트 방지
- **`test_policy.py`** — 3바퀴 통합 시뮬레이션에서 **주행 중 정지 명령이 0프레임**인가 (규정 p.21)
- **`test_perception.py`** — 빈 프레임·NaN·스캔 길이 변화에 죽지 않는가, 부호가 맞는가
- **`test_replay_tools.py`** — 랩 임계값 자동 탐색이 올바른 구간을 고르는가

---

## 7. 알려진 한계 — 반드시 읽을 것

정직하게 적는다. 아래는 **연습주행에서 반드시 확인해야** 하고, 코드만으로는 보장할 수 없다.

1. **모든 튜닝값이 미검증이다.** 차선·콘 HSV 는 팀의 기존 실차 코드(`segi`)에서
   가져왔지만 나머지(신호등 반지름, 겐트리 span, px_per_m)는 합리적 추정치다.
   실차에서 맞추기 전에는 **아무것도 동작한다고 가정하지 말 것.**

2. **랩 카운팅은 신호등 겐트리 목격에 의존한다.** Gate(통과감지장치)는 심판 장비라
   우리가 읽을 수 없어서 이 방식을 택했다. 8/13 재설계: 마커는 원 검출 span 이
   아니라 **채도 초록 램프 블롭 폭**이다 — 대회장 천장 조명이 원 조합을 오염
   시킨다는 것이 실주행 bag 포렌식으로 확정됐기 때문 (signal_detector 모듈 주석).
   run4 bag 검증: 진짜 통과 32px vs 최악 오검 14px, 안전 구간 11~31.
   IMU 요 적산 교차검증은 경고 로그로 배선돼 있고(race_node), 자동 보정은
   하지 않는다 — 잘못 올리면 3바퀴를 못 채우기 때문이다.
   **최종 임계값은 3바퀴 정주행 bag 으로 재산출해야 한다** (차의 ~/tune_laps.sh).

3. **고정장애물 ↔ 방해차량 판별은 상대속도 하나에 의존한다.**
   2D 라이다 클러스터 속도 추정은 노이즈가 크다. 불확실하면 '주행 중'으로
   보게 해 놨는데(재평가하는 쪽이 정지 타겟에도 안전), 오판 시 추월 방향을
   틀릴 수 있고 그건 차선이탈 판정이다.

4. **지름길 진입 좌회전이 개루프다.** 분기점에서 차선 도색이 끊겨 검출을
   믿을 수 없어 고정 조향으로 돈다. 진입 속도가 다르면 회전 반경이 달라진다.
   `shortcut_steer_deg` 와 진입 속도를 세트로 맞춰야 한다.

5. **`px_per_m` 를 안 맞추면 라바콘 조향 배율이 틀린다.** 게이트 중앙은 잘
   찾아도 조향량이 과하거나 부족해진다.

6. **리플레이는 퍼셉션·FSM 만 재생한다.** 수동주행 bag 이라 조향 명령은
   실제 궤적과 무관하다. "이 조향이 맞았는가"는 여전히 실차에서만 알 수 있다.
   리플레이로 검증되는 건 인지 값과 상태 전이까지다.

---

## 8. 출처

설계와 알고리즘을 아래에서 참고했다.

- [doldolmeng2/2026-kookmin-contest](https://github.com/doldolmeng2/2026-kookmin-contest) — 아키텍처 원칙(rclpy 없는 순수 로직 + 얇은 노드), FSM 구성
- [mastic-choi/UMK](https://github.com/mastic-choi/UMK) — 4구 신호등 HoughCircles 인덱싱 + 조합 전수탐색, 추월 컨트롤러 설계
- [jeonghun43/xytron_convention](https://github.com/jeonghun43/xytron_convention) — 퓨어 퍼수트 기반 추월 기동
- 팀 기존 실차 코드 `segi/my_driver_*.py` — BEV 좌표, 차선 HSV, 콘 HSV, 모터 인터페이스

라이선스: MIT
