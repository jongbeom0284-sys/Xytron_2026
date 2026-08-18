# doldolmeng2 3개 레포 정밀 분석 (ddm-2026 / ddm-2026-qual / ddm-2025)

분석일: 2026-08-13. 대상 베이스: `scratchpad/repos/`. 세 레포는 같은 팀(팀명 orda→ericar→orda)의 연속 작업물이며,
**ddm-2025 `modular/` → ddm-2026 `orda/`가 직계 진화 관계**다(lane_detection·rubbercone·object_detection C++ 노드와
main 파이썬 오케스트레이터 구조가 그대로 이어짐). ddm-2026-qual `ericar/`는 예선(유니티 시뮬) 전용 별도 구현이지만
"노드 파일만 토픽을 만지고 나머지는 순수 로직 모듈" 원칙과 IMU yaw 좌회전, 하우징 우선 신호등 판정의 기원이다.

참고: 세 레포 모두 depth-1 클론이어서 `git fetch --unshallow`로 전체 이력을 복원한 뒤 분석했다.
ddm-2025의 HEAD(f8b8356)는 대회 후 2026-05 리팩토링본이므로, 실전값은 **final commit `b79f4ef`(2025-08-28)** 기준으로 인용한다.

---

# 1. ddm-2026 — doldolmeng2/2026-kookmin-contest (★최우선)

## A. 메타
- **2026 제9회 본선용, ROS2 (Humble 계열), 실차 Xycar** + rosbag 오프라인 검증 체계. 팀 코드는 `orda/` 폴더(나머지 `xycar_*`, `study/`는 Xytron 제공물).
- 커밋 활동 (마지막 푸시 2026-08-12, 어제):
  - 08-01: 2026 구조 개편(README 재작성, 역할별 폴더 재배치), 라바콘 탐색영역 제한+재튜닝
  - 08-04~05: **본선 FSM 대공사** — 10상태 순수 FSM 골격, 라바콘 세션 리셋 핸드셰이크, bag 통합 테스트 하네스, 유닛테스트 280→357개
  - 08-07: 방해차량 YOLO 박스 선택을 신뢰도→면적 기준으로(충돌 사고 해결)
  - 08-11: **4구 신호등 YOLOv8 노드(PR#3)**, 추월 판정 초음파→측면 LiDAR 교체, `/lane_position` 실측 차선 발행 복원
  - 08-12: **고정장애물 YOLO 재학습 + 회피 방향 카메라 단독 전환(PR#4)**, `zone_timeout_s` 8→12초, README 갱신
- 실주행 증거: 실차 bag 다수 실측이 코드 주석에 직접 인용됨 — `rubbercone_20260725_221245`, `20260724_*_kmu_real_lidar`,
  `rosbag2_fixed_obstacles_*` 6개, `cone_11`, `rosbag2_object1` 등 (`orda/TEST_LOG.md`, `orda/main/main/overtake.py:26-52`).
  연습주행 기간 7/1~8/21 명시(`orda/README.md:4`).

## B. 아키텍처 — 우리가 참고한 원본이 어떻게 진화했나
- **원칙 유지·강화**: "노드 파일은 `main.py` 하나뿐, 나머지는 rclpy를 import하지 않는 순수 로직 모듈" (`orda/README.md:135-137`).
  main 패키지 = `race_fsm.py`(10상태 FSM) + `race_context.py`(랩/지름길 전역) + `runtime_adapter.py`(콜백 엣지 큐→관측 스냅샷)
  + `control.py`/`control_selector.py`(제어) + `safety_monitor.py` + `cone_entry.py`/`lane_validity.py`/`overtake.py`(디바운스 가드)
  + 유닛테스트 15파일(357개 통과, `orda/TEST_LOG.md:265`).
- 인지·주행 노드는 C++: `lane_detection.cpp`(70KB), `rubbercone.cpp`(38KB), `object_detection.cpp`, `resize.cpp`.
  신호등만 Python onnxruntime(`traffic_node.py`).
- **상태기계 (race_fsm.py:23-33)**: INIT → WAIT_GREEN → LANE_DRIVE ⇄ {CONE_DRIVE→REJOIN, FIXED_AVOID, OVERTAKE, SHORTCUT} → FINISH, 전 상태→STOP.
  - **STOP은 더 이상 터미널이 아니다**: FINISH만 종료 상태(`race_fsm.py:98`). LiDAR 워밍업 1~2초/bag 루프/드라이버 없는 런치
    세 상황에서 즉사했던 경험이 주석에 남아 있고(`race_fsm.py:89-97`), 입력 회복 후 0.5초 연속 정상(`STOP_RECOVERY_HOLD_S`,
    `race_fsm.py:102`)이면 정지 직전 모드로 복귀한다(`race_fsm.py:198-220`).
  - **SafetyMonitor 기동 유예 5초**: 아직 한 번도 안 온 입력(missing)은 워밍업으로 보고 유예, 오다 끊긴 입력(stale)은 즉시 정지
    (`safety_monitor.py:163-181`). 규정 시간 감시 내장: 전체 240초/라바콘 60초 (`safety_monitor.py:64-66`).
  - **모든 미션 이벤트는 "수신시각 엣지"**: 엣지 1회 소비 + 단조 증가 타임스탬프 + 신선도 0.5초 + `state_entered_at` 이후 조건
    (`race_fsm.py:479-506`). sticky 값이 나중에 유효해지는 것을 원천 차단.
  - **legacy 어댑터 분리**: 내부는 문자열 Enum, 외부 `/mode_info`는 lane_detection.cpp가 쓰는 정수(0~5)로 명시 변환
    (`mode_info.py:9-36`). 미확정 상태는 새 숫자를 만들지 않고 STOP 코드 0 발행.
- **bag 루프 감지**: 시각이 2초 이상 역행하면 런타임 어댑터를 통째로 재생성(이전 수신시각이 전부 '미래'가 되는 문제,
  `main.py:53-56, 512-543`).

## C. 미션별 구현
### 신호등 (4구)
- **YOLOv8 5클래스** `['4-traffic'(하우징), green, left_green, orange, red]`, onnxruntime CPU, 640×640 단순 리사이즈,
  conf 0.5 / NMS 0.45 / **디바운스 3프레임**, 우선순위 **LEFT(3) > GREEN(2) > RED·ORANGE(1)** (`traffic_node.py:12-13, 86-88, 91-95`).
  `/traffic_detection`(Int32): 0=미인식, 1=정지, 2=직진, 3=좌회전.
- 튜닝 방향의 비대칭 원칙 문서화: **좌회전(3)은 미검출 최소화, 정지(1)은 오검출 최소화** — 라바콘이 주황이라 색 블롭만으로
  정지 판정하면 위험, 하우징 먼저 찾거나 ROI 상단 제한 (`orda/README.md:232-240`).
- ⚠️ **미배선**: `main.py:215-220`은 여전히 `Bool`로 `/traffic_detection` 구독 — YOLO 노드는 `Int32` 발행이라 **타입 불일치로
  현재 연결되지 않는다**. 4상태 라우트 신호(`RouteTrafficSignal`)는 test seam(`record_route_traffic`,
  `runtime_adapter.py:445-469`)만 존재. README도 "실제 traffic encounter publisher는 없음"이라고 자인(`orda/README.md:462`).
### 출발
- WAIT_GREEN에서 green 3프레임 연속 디바운스 후 LANE_DRIVE. **경기 시계는 디바운스 완료가 아니라 '첫 fresh green 수신 시각'부터**
  (규정 p.17 반영, `race_fsm.py:248-254`). 출발 후 3초간 LANE 속도 ≤5.0 (`main.py:667-674`).
- 2·3랩 정지 신호는 모드 전환 없이 **제어 계층 오버라이드**(`traffic_stop_override` → ControlSelector `traffic_hold` 0 명령,
  `runtime_adapter.py:599-610`, `control_selector.py:102-103`). WAIT_GREEN 재진입 금지 (`orda/README.md:36-39`).
### 차선 (흰 실선이 아니라 노란 중앙 점선을 추적!)
- BEV(사다리꼴 ROI: top_y=0.7H, top_w=0.9W, bottom_w=2W) + **노랑 3중 마스크(HLS∧HSV∧YCrCb)** + 슬라이딩 윈도우 20개
  (margin 100, minpix 15) + x=m·y+b 최소자승 (`lane_detection.cpp:1-16`, `lane_detection_parameter.json`).
- 오프셋 = BEV 높이 30%·80% 두 지점 x 평균 − ref_ratio×W (`lane_detection.cpp:649-662`).
  **ref 비율이 차선 선택을 결정**: L1=0.63, L2=0.35, CENTER=중간 0.49 (`json:38-39`, `cpp:920-927`).
  차선 변경 = ref 비율 전환 애니메이션(1.0s) — 조향 로직은 그대로 두고 기준점만 옮긴다.
- 수평/수직 노이즈 억제 단계 보유(정지선·횡단보도류 가로 밴드 제거: `horizontal_noise_*`, `vertical_noise_*`, `json:40-48`).
- `/lane_position` 실측 차선: BEV 최하단 중앙선 x 비율을 3기준(L1/중앙/L2) 최근접 분류, 1·2등 차이<0.05는 판정 보류,
  **5프레임 연속 디바운스** (`lane_detection.cpp:1155-1201`, `LANE_DETECT_STREAK_NEED_=5`, `LANE_CLASSIFY_DEADZONE_RATIO_=0.05`).
- `/lane_change_state` 완료 판정(스파이크 400px 후 50px/300px 이내 8프레임, `json:49-53`)은 **실차에서 스파이크 자체가 안 나와
  (실측 최대 217~313px) 성공 0회** → 완료 판정 ①을 실측 차선(`/lane_position`)==목표 로 교체 (`runtime_adapter.py:845-859`).
### 라바콘
- 알고리즘(2025 대비 전면 재작성): 반사점 클러스터(간격 0.08m로 분리, 폭≤0.22m만 콘 인정, 0.10m 내 병합) → 거리순 상위 6개만
  → y부호로 좌/우 분류(deadband 0.03m) → **거리 가중 1/(0.15+r) 최소자승 직선**을 좌우 각각 피팅(경계당 최대 3콘, 잔차 0.18m
  아웃라이어 1회 제거) → lookahead 0.70m 지점의 통로 중앙을 목표점으로 (`rubbercone.cpp:294-571`).
- 한쪽 경계만 보이면 **학습된 통로 반폭(adaptive_half_width, EMA 0.15, 초기 0.30m)** 으로 반대편 추정, 신뢰도 상한 0.78
  (`rubbercone.cpp:551-571`). 양쪽 신뢰도 = 0.65+0.35×품질(콘 수 50%+잔차 25%+커버리지 25%, `rubbercone.cpp:481-490`).
- 출력 스무딩 2단: 목표점 y 스텝 제한 0.20m+EMA(양쪽 0.85/한쪽 0.75) → offset = −y×**gain 150**(230은 0.17m에 ±40 포화,
  실주행 bag으로 확인, `rubbercone.cpp:176-180`), limit 45, 오프셋 스텝 20, EMA 0.80.
- 종료: conf≥55%가 5프레임 연속이면 armed(`kArmValidFrames=5`, `rubbercone.cpp:819`) → 유효 경로 **3프레임 연속 부재 시 end 래치**
  (`end_missing_frames=3`, `rubbercone.cpp:597-657`). 미검출 시 신뢰도만 −25/프레임으로 급감(메인이 감속하도록).
- 탐색창: 0.18~**1.10m**, ±85°, |y|≤0.70m(벽/옆 코스 제거), 전방 ±13°·0.32m 미만은 차체 반사 무시 (`rubbercone.cpp:114-127`).
- **진입은 FSM 쪽 디바운스**: conf≥75 인 콘 메시지 3회·0.2초 지속 + cone/scan 신선도 0.25초 (`cone_entry.py:14-25`).
  진입 확정 시 `/rubbercone_reset`(Reliable) 발행→검출기 세션 리셋 핸드셰이크 (`main.py:404-411`, `rubbercone.cpp:210-216, 276-280`).
  콘 진입 래치는 랩마다 재무장, **지름길 랩에서는 콘 신호 자체를 오검출로 무시** (`race_fsm.py:291-293`, `orda/README.md:385-393`).
### 고정장애물 vs 방해차량 판별
- `/object_info` 11필드로 확장, `is_moving`(0=고정, 1=방해차량) 신설 — **그러나 현재 모델이 고정장애물(빨간 차)만 학습해 항상 0**
  (`object_detection.cpp:371-382`). 판별은 사실상 미구현이고, 지금은 모든 검출을 FIXED_AVOID로 처리.
- LiDAR 클러스터: 전방 ±10°, 0.05~2.0m, epsilon 0.20m, **min_cluster_points 5→3**(1.5~2m 앞 차는 4~7점밖에 안 맺힘; 검출률
  32.4→89.7%, 오검출 8~11→11~20%, `object_detection.cpp:52-57`).
### 회피(FIXED_AVOID)와 추월 완료
- **구간 진입 = 카메라**: YOLO 박스 면적 > 1900px² (`main.py:58-60 FIXED_ENTRY_BOX_PX`, `_drive_mission_zones` `main.py:583-598`).
- **회피 방향 = 카메라 단독**: `car_lane`(박스 중심 vs `/lane_fit` 중앙선, margin 10px, `object_detection.cpp:63, 535-540`)의
  반대 차선으로 `lane_target` 변경 (`runtime_adapter.py:819-843`, `mission_types.py:37-44`). LiDAR `exists`를 요구했다가
  방해차량이 2m 밖(클러스터 1/103 스캔, exists 1810샘플 내내 0)이라 회피가 시작 안 되는 사고 → 제거 (`runtime_adapter.py:822-831`).
  car_lane 미확정(0)이면 인가하지 않음(정지), 박스가 없으면 회피 차선 유지.
- 두 대 동시 검출 시 **면적 최대 박스 선택**(신뢰도 기준이면 car_lane이 L1/L2로 플립, 게다가 먼 차를 판정; `object_detection.cpp:502-518`).
- **추월 완료 = 측면 LiDAR**: 현재 차선 반대쪽 60~100° 섹터 최소거리 < **0.40m** 인식 → **2.0초 후** 완료 확정
  (실측: 통과 순간 0.26~0.34m, 벽/빈 차선 0.60m+; 0.60으로 두면 회피 끝나기 전에 카운트다운 시작; `overtake.py:26-42`).
  감시 측은 실측 차선(`/lane_position`) 우선, 미확정 시 lane_target 폴백 (`overtake.py:157-192`).
  |각도|>105°는 자기 차체(0.10~0.14m)가 잡히므로 제외, 1.5m 이상은 벽으로 무시 (`overtake.py:47-53`).
  방해차량을 못 만난 랩 대비 **zone_timeout 12초**(어제 8→12로 상향, 커밋 9d99c5f).
- 복귀: **회피한 차선을 유지**(중앙 복귀 안 함, `orda/README.md:262-268`), 초음파는 고착 사고(우측 4cm 고착 98%→차선 복귀→충돌,
  `overtake.py:178-181`) 이후 판정에서 완전히 배제.
### 지름길
- FSM 전이는 완성: 신호 3 & lap∈{2,3} & 미사용 → **조건 없이 즉시 SHORTCUT**(직진 저울질 분기 없음), `shortcut_lap` 기록으로
  이중 소비 방지 (`race_fsm.py:464-477`). 지름길 랩에는 콘/고정/추월 구간 체인 전체 억제 (`race_fsm.py:271-293`).
- **그러나 주행 컨트롤러(`shortcut_turn.py`, IMU yaw 기반 좌회전 궤적)는 미작성** — 파일 자체가 없고 SHORTCUT의 외부 모드 코드도
  STOP(0) (`mode_info.py:20-28`, `orda/README.md:461`). 예선 `turn_left.py`(yaw+90°)가 설계 원형.
### 랩 카운팅
- **신호등 조우 = 랩 경계** (Gate는 주최측 장비라 차량이 신호를 받을 수 없음; 지름길이든 직진이든 매 랩 신호등 앞을 지남,
  `orda/README.md:396-399`). `_handle_traffic_encounter`: encounter 엣지 1회 소비 + 타임스탬프 단조 증가 검사 → `completed_laps`++,
  3회면 FINISH (`race_fsm.py:420-462`). 중복 방지는 min-gap 시간이 아니라 **엣지 소비+단조 타임스탬프** 방식.
- ⚠️ encounter를 만들어 줄 production publisher가 없다(위 신호등 미배선과 동일 뿌리).

## D. 제어 파라미터
| 항목 | 값 | 위치 |
|---|---|---|
| PD (kp, kd) | LANE (0.145, 0.3) / CONE (1.0, 0) / FIXED_AVOID (0.12, 0.35) | `control.py:24-33` |
| 속도 (max/min/scale) | LANE 43/16/0.5 / FIXED 30/14/0.6 — **모터 발행은 ÷2** (실제 21.5/8, 15/7) | `control.py:39-45`, `main.py:627-637` |
| 라바콘 속도 | cruise 22, cautious 18, min 13.5 (÷2 → 11/9/6.75), conf 35%→9.0, 85%→11.0 | `control.py:52-64` |
| 라바콘 코너 감속 | 15°부터 기울기 0.20, 하한 14(=7); 조향 한계 45° | `control.py:58-63` |
| 라바콘 슬루 | 가속 0.25 / 감속 0.35 / 조향 3.0 per 20ms (0.10은 순항 도달 2.2s→0.9s) | `main.py:44-51` |
| LANE 가속 | +0.1/사이클, 감속 즉시; 출발 후 3s 속도 ≤5 | `main.py:664-700` |
| 명령 신선도 | lane/cone 0.25s 넘으면 0 명령 (CONE에서 lane 폴백 금지 — 콘이 차선 위) | `control_selector.py:43-51, 76-80` |

## E. 튜닝값 총표 (파일:라인)
| 파라미터 | 값 | 파일:라인 |
|---|---|---|
| green 디바운스 | 3프레임, max_age 0.5s | `race_fsm.py:107-110` |
| 미션 엣지 신선도 | 0.5s | `race_fsm.py:111` |
| 콘 진입 | conf≥75, 3회, 0.2s, cone/scan age 0.25s | `cone_entry.py:21-25` |
| REJOIN 차선 유효 | true 엣지 3회·0.2s·age 0.25s | `orda/TEST_LOG.md:100-109`, `lane_validity.py` |
| 추월 완료 | side<0.40m → 2.0s, timeout 12s, FOV 60~100°, max 1.5m | `overtake.py:37-53` |
| 고정 진입 박스 | 1900 px² | `main.py:60` |
| YOLO(장애물) | conf 0.50, NMS 0.40, min 12px, 레터박스 114 패딩 | `object_detection.cpp:118-121, 434-451` |
| YOLO(신호등) | conf 0.5, NMS 0.45, 디바운스 3 | `traffic_node.py:33-35` |
| LiDAR 클러스터 | ±10°, 0.05~2.0m, eps 0.20m, ≥3점, lane margin 10px | `object_detection.cpp:46-63` |
| 라바콘 탐색 | 0.18~1.10m, ±85°, lateral 0.70m, 차체 13°/0.32m | `rubbercone.cpp:115-121` |
| 라바콘 클러스터 | break 0.08m, 폭≤0.22m, 병합 0.10m, 콘 6개/경계 3개 | `rubbercone.cpp:122-129` |
| 라바콘 경로 | lookahead 0.70m(min 0.35), half_width 0.30m, 통로 0.35~1.40m, slope≤1.20, 잔차 0.18m | `rubbercone.cpp:130-137` |
| 라바콘 출력 | gain 150, limit 45, step 20/0.20m, EMA 0.80/0.85/0.75 | `rubbercone.cpp:169-204, 843-860` |
| 라바콘 종료 | arm: conf≥55% 5프레임 / end: 3프레임 미검출 | `rubbercone.cpp:602-657, 819` |
| 차선 노랑 | HLS H20-36 L90+ S90+ / HSV 동일 / YCrCb Cr125-195 Cb0-175 | `lane_detection_parameter.json:2-19` |
| BEV/윈도우 | ROI top 0.7H·0.9W, bottom 2.0W; 윈도우 20/마진100/minpix15 | `json:23-31` |
| 차선 ref | L1 0.63 / L2 0.35 / 중앙 0.49, 전환 1.0s | `json:38-39,54`, `cpp:920-927` |
| 실측 차선 | 5프레임 디바운스, deadzone 0.05 | `lane_detection.cpp:975-976` |
| 차선변경 판정(구) | spike 400px, settle 50/300px, 8프레임, m_split 0.1 | `json:49-53` |
| 안전 한계 | race 240s, cone 60s, stale 0.5s, 기동유예 5s, STOP 복귀 0.5s | `safety_monitor.py:60-66`, `race_fsm.py:102` |
| bag 루프 감지 | 2.0s 역행 | `main.py:56` |

## F. 실전 노하우 / 함정 (주석·커밋에서)
1. **라바콘 오검출로 FSM 탈선**: 고정장애물 구간에서 벽·장비를 콘으로 오인, conf≥75가 5연속 → LANE_DRIVE→CONE_DRIVE로 샜다.
   네거티브 bag으로 임계 재검증 전까지 고정장애물 검증 시 rubbercone_node를 띄우지 않음 (`orda/README.md:467-472`).
2. **초음파 고착**: 우측 초음파가 전 구간 4cm 고착(샘플 98%) → 차선 바꾸자마자 "추월 완료" 오판 → 복귀 → 충돌 (`overtake.py:178-181`).
3. **LiDAR 사거리 함정**: range_max 2.0m 때문에 전방 방해차량 클러스터가 103스캔 중 1번 — exists를 방향 결정에 쓰면 안 됨 (`orda/README.md:283-289`).
4. **YOLO 전처리 불일치**: 640×360→640² 단순 리사이즈는 세로 1.78배 왜곡, 레터박스로 82.3→98.1% (`object_detection.cpp:434-437`).
5. **죽은 conf 임계**: 옛 모델 최고 conf 0.82인데 임계 0.83 → 사실상 전부 폐기 상태로 주행하고 있었음 (`object_detection.cpp:115-118`).
6. **mode 5↔3↔0 요동**: 인지 신선도에 따라 매 주기 뒤집혀 lane_detection 추적기가 진행도를 버림(변경 시작 279회/성공 0회) →
   pending 동안 명령 래치 (`main.py:424-438`), FIXED_AVOID 중 STOP 코드 발행 금지 (`mode_info.py:56-66`).
7. **negative 샘플 200장 포함 재학습**: 빈 라벨 프레임이 배경 오검출을 줄인다. mAP50 0.995 (`orda/README.md:213-227`).
8. **GTK imshow는 메인 스레드에서만**: rclpy 콜백/보조 스레드에서 부르면 창이 안 뜸(WSLg 등) → 표시 전용 스레드 분리 구조 (`main.py:252-268, 773-789`).
9. 모델/설정 경로는 개발 PC·실차 **후보 배열**로 이중화 — 한쪽만 하드코딩했다가 조용한 로드 실패 경험 (`object_detection.cpp:158-166`, `lane_detection.cpp:1274-1279`).
10. QoS 규약: 센서·수치 BestEffort/Volatile 통일, 엣지 명령(`/rubbercone_reset`)만 Reliable (`main.py:95-102`).
11. bag 검증 하네스가 `/xycar_motor` publisher 0개 인터록·scan-only 재생·모터 토픽 격리를 강제 (`orda/TEST_LOG.md:46-63`).

## G-1. ddm-2026 최근 커밋이 드러내는 '지금 고민하는 문제'
1. **인지→FSM 실배선이 마지막 관문**: 신호등 YOLO는 Int32, main은 Bool 구독(타입 불일치로 미연결), 랩 encounter·`/lane_valid`
   (REJOIN 탈출 조건)·shortcut 완료의 production publisher가 전부 부재. FSM/테스트는 완성됐는데 **실차 그래프가 아직 안 이어져 있다**.
2. **방해차량(이동체) 대응**: is_moving 상시 0, OVERTAKE 진입 경로 없음 — 고정장애물만 우선 완성하고 이동체는 후순위로 미룸.
   zone_timeout 8→12초 상향은 "방해차량 조우가 늦거나 없을 때"를 시간으로 때우는 중이라는 뜻.
3. **지름길 궤적 미작성**: SHORTCUT 진입 즉시 정지(외부 코드 0)하는 상태. IMU yaw 좌회전 컨트롤러가 다음 작업.
4. **라바콘 오검출 네거티브 검증**: 고정장애물 bag 6개를 네거티브 셋으로 쓰는 재검증이 백로그.

## H. 신뢰도: **상**
근거: (1) 실차 bag 실측 수치가 코드 주석·TEST_LOG·README 세 곳에서 교차 일치, (2) 유닛테스트 357개+bag 통합 하네스,
(3) 어제(08-12)까지 활발한 커밋. 단, 위 G-1의 배선 공백 때문에 "이대로 본선을 뛸 수 있는 코드"는 아직 아니며,
남은 2주(본선 8/25)의 작업 방향이 명확히 읽힌다.

---

# 2. ddm-2026-qual — doldolmeng2/2026-kookmin-contest-qualifier

## A. 메타
- 2026 9회 **예선**(유니티 시뮬레이터 + ROS-TCP-Endpoint), ROS2 Python 전용. 활동 2026-05-24 ~ 06-26.
- 커밋 "0143", "2'43'' time"(06-26) = 시뮬 랩타임 기록 갱신 흔적. 팀명 ericar.
- 예선 미션은 본선과 다름: 경찰차, 스쿨존, 보행자, 출발선(체커보드), 지름길 진입·탈출 좌회전 2회.

## B. 아키텍처
- `ericar/{perception, driving, main, function}` — **"토픽 sub/pub은 노드 파일(main/control/perception/driving)에서만,
  로직 모듈은 rclpy import 금지"** 를 README 지침으로 명문화 (`README.md:16-39`). → ddm-2026 본선 구조의 기원.
- main(FSM)과 control(offset→모터)을 **별도 노드로 분리**. 모드 10개: WAIT/CONE/LANE/LEFT_TURN/LANE_CHANGE/FOLLOW/
  SIGNAL_WAIT/SCHOOL_ZONE/S_CURVE/SHORTCUT (`main.py:31-33`). 디버그 창에서 숫자키 0~6으로 모드 강제 전환.
- perception은 상황 배열 `/perception/status` Int32MultiArray 13필드 하나로 통합 발행 (`main.py:53-66`).

## C. 미션별 구현 (본선에 계승된 것 위주)
- **신호등 = 검은 하우징 게이팅**: ROI에서 검은 픽셀 blob(≥120px², 가로/세로≥1.5)을 먼저 찾고 그 bbox 안에서만 색 카운트
  (`perception/traffic_light.py` 도입부) → ddm-2026 README의 "하우징 먼저 찾은 뒤 색 판정" 권고의 원형.
  다수결 히스토리 7프레임. 빨+초 동시 = 좌회전.
- **좌회전(지름길) = IMU yaw**: `turn_left.py` — 진입 시 현재 yaw+90°를 목표로 래치, 도달 판정 tol 5°, 도달 전까지 최대 좌
  오프셋 −1.0 고정 (`turn_left.py:16-21`). 진입(A)·탈출(B) 두 종류. → 본선 `shortcut_turn.py`(미작성)의 설계 원형.
- **랩 카운팅 = 출발선(체커보드)**: 화면 맨 아래 띠(높이 80~100%, 좌우 15~85%)에서 검정≥5%·흰색≥11% 동시 충족
  (`start_line.py:14-21`) + `lap_locked` 래치를 스쿨존 통과 후 해제해 중복 카운트 방지 (`main.py:100-101, 206-215`).
- 라바콘: 클러스터 0.20m/≤4점, 전방 ±40°, 좌우 최근접 콘 중점. 한쪽만 보이면 `CONE_SEPARATION 6.153m`(시뮬 스케일!)로 반대편 추정
  (`rubbercone.py:15-24`). 콘 종료 판정 = 전방 하단 ROI의 노란 픽셀 ≥400 (`main.py:80-85, 336-343`).
- 차선: BEV 400×300 + HSV 노랑(S 205~255 — 시뮬 채도) + 슬라이딩 14윈도우 + **2차 다항 피팅, 곡률 가중 K=200**
  (`lane_detection.py:29-42, 100-107`). ref_x {1차선 230, 2차선 155}.
- S커브 = yaw 0.78 + 차선 0.22 블렌딩 오프셋 (`lane_detection.py:44-50`).
- 추월: FOLLOW(저속 추종) → obstacle_passed → 반대 차선 LANE_CHANGE (`main.py:197-201`) — 본선과 달리 "따라가다 추월".

## D. 제어
- `angle = offset × MODE_RATIO`, 비율: CONE 250 / LANE 0.48 / LEFT_TURN 100 / LANE_CHANGE 0.37 / FOLLOW 0.45 / S_CURVE 1.4 /
  SCHOOL·SHORTCUT 200 (`control.py:25-36`). 속도표: CONE 15 / LANE 17(조향 비례 감속, min 7) / LEFT 11 / CHANGE 13 / FOLLOW 12 /
  SCHOOL 16 / S_CURVE 12 / SHORTCUT 18 (`control.py:39-50, 104-107`).

## E~F. 주요 값·메모
- 전략 메모(`README.md:199-204`): 스쿨존은 벌점 감수 고속 통과(5초 벌점 < 10초 손해), 신호 무시 손익 계산, 중앙선 침범 무감점 등
  **벌점-시간 트레이드오프를 명시적으로 계산**하는 팀 문화.
- 시뮬 전용 값 다수(HSV 채도 205+, CONE_SEPARATION 6.153m, ratio 250 등)는 실차 이식 불가.

## H. 신뢰도: **중**
시뮬 전용이라 실차 근거는 없지만, 아키텍처 원칙·하우징 신호등·IMU yaw 좌회전·랩 래치 패턴이 본선 코드로 계승된 것이
확인되므로 "설계 계보" 자료로서 가치가 있다.

---

# 3. ddm-2025 — doldolmeng2/2025-kookmin-contest (작년 본선 실전 코드)

## A. 메타
- 2025 제8회 대회, ROS2, 실차. 2025-07~08 매일 "backup 250823 16:26" 식 커밋 → **8/28 "final commit"** — 대회 직전까지 실차 튜닝한
  실전 코드가 맞다. HEAD(f8b8356, 2026-05-30)는 대회 후 리팩토링(주석 정리·모놀리식 제거·GUI 정비)본.
- 실전값 인용은 final commit `b79f4ef` 기준(리팩토링본과 제어값 동일함을 확인).

## B. 아키텍처
- `modular/{main(Python 정수 FSM), lane_detection(C++), rubbercone(C++), object_detection(C++ YOLO+LiDAR), traffic_light(C++),
  image_resize}` — ddm-2026 `orda/`의 전신. main은 콜백이 멤버에 직접 쓰고 50Hz 타이머에서 if-elif 전이하는 **단일 노드 정수 FSM**.
- 모드 6개와 전이 (`main.py`(final):144-168):
  `TRAFFIC_WAIT(0) →초록불→ RUBBERCONE_DRIVE(1) →end_flag→ RUBBERCONE_END(2, 1.4s 고정조향) → BEFORE(4)
  →초음파 추월확인→ LANE_DRIVE(3) →box>1900px²→ CHANGE_LANE(5, 차선 토글) → BEFORE(4) 루프`.
- **랩 카운팅 없음, 신호등은 출발 1회용** — 작년 미션 구조(라바콘이 출발 직후, 장애물 회피 반복)가 그대로 읽힌다.

## C. 미션별 구현
- 신호등: **우상단 ROI(가로 6/11~끝, 세로 상단 1/3) HSV 초록(H 50~150, S/V 100+) 픽셀 비율 ≥2%** (`traffic_light.cpp`(final):53-77).
  YOLO 아님. 디바운스 없음.
- 라바콘: **콘 간격 0.42m 기하 기반** — 좌/우 최근접 콘에서 간격 0.42±0.10m 체이닝으로 그룹(≤3개) 확장, 좌우 그룹 중점의 중점을
  목표로. `offset = −y × 300`. 탐색 0.18~1.00m, ±90°, 전방 ±13° 무시. 종료 = 양쪽 2+2 프레임 30회(armed) 후 유효 목표 상실
  (`/tmp` 추출본 `x_2025_rubbercone.cpp:55-186`; OFFSET_GAIN 300은 :20). → 2026에서 직선 피팅+신뢰도+래치로 전면 재설계됨.
- 차선: 2026과 같은 계보(노란 중앙선 BEV, ref 0.63/0.35). 대회 사흘 전 **YCrCb 마스크 추가**(commit 708d5d2, 08-27)로 3중 마스크화,
  HLS L 120→110, HSV S 40→20 완화. `ref_hist_sigma_ratio 999`(사실상 히스토그램 가중 끔), minpix 5.
- 차선 변경: mode 5 → lane_detection이 ref 비율 전환(1.0s). **완료 피드백은 대회 전날 밤 비활성화**(commit fac70d7
  "comment out updateLaneChangeState call") + `is_change_end()`는 항상 True (`main.py`(final):233-234) —
  즉 CHANGE_LANE은 1사이클 만에 BEFORE로 넘어가고, 실제 변경은 ref 전환+PD가 알아서 하게 뒀다.
- 추월 완료: **반대쪽 초음파 <25cm 10회 카운트** (`main.py`(final):236-247). (이 방식이 2026에서 고착 사고로 폐기됨)
- 장애물: LiDAR ±10°/2.0m/eps0.20/**min 5점** + YOLO conf **0.83**(레터박스 없음 — 2026 분석으로 "최고 conf 0.82라 사실상 전부
  버리는 상태"였음이 드러남), lane_split_margin 6px (`x_2025_objdet.cpp:34-45, 80`).

## D. 제어 (실전 투입값)
| 항목 | 값 | 위치(final) |
|---|---|---|
| PD | LANE/CHANGE (0.145, 0.3) — **2026과 동일, 2년 검증 게인** / CONE (1.1, 0) | `control.py:20-24` |
| 속도 | LANE/CHANGE 31/12/0.5, CONE 13 고정, BEFORE = LANE−2 | `control.py:28-32, 158` |
| 라바콘 종료 | 고정 조향 **−31° / 속도 15 / 1.4초** 후 차선 진입 | `control.py:36-39`, `main.py:65` |
| 출발 완속 | LANE 진입 후 3초 speed 5.0 | `main.py:190-194` |
| 가속 슬루 | +0.1/사이클(감속 즉시) | `main.py:178-181` |
- 주의: 2025는 모터에 그대로 발행(최고 31), 2026은 ÷2 발행(최고 21.5) — **속도 스케일 운용이 다르다**.

## F. 실전 노하우
- 대회 마지막 주 커밋 궤적이 곧 우선순위: 수직 노이즈 억제(08-25~26)→기준선 부드러운 전환→YCrCb 마스크(08-27)→피드백 기능
  통째 비활성(08-27)→best.onnx 교체(08-27). **마지막엔 기능을 더하는 게 아니라 끄고 단순화**했다.
- "하드코딩 잘작동 +타이머"(647cfba), "박스 찍어줘"(043039a) 등 — 실차 마감 국면의 임시방편이 그대로 본선 코드가 됨.
- bag 재생과 런치 병행 디버깅 체계(209559e)는 이때부터.

## H. 신뢰도: **상** (실전 투입 확실)
매일 백업 커밋+final commit, 값들이 2026 코드 주석에서 "작년 값"으로 재인용됨. 단 작년 미션 구조가 올해와 달라
(랩/신호등 4구/지름길 없음) FSM 자체보다는 **제어 게인·차선 파이프라인·실전 운용 방식**이 참고 대상.

---

# 4. 우리 kmu_race 대비 — 가져올 것 / 반면교사

## 가져올 것 (우선순위순)
1. **복귀형 STOP + 기동 유예**: 우리 safety도 '정지 최후수단' 철학이지만, ddm-2026처럼 (a) FINISH만 터미널, (b) 정지 직전 모드 기억
   후 0.5s 안정 시 복귀, (c) missing(워밍업)과 stale(주행 중 유실)을 구분하는 startup_grace 5s를 넣으면 "정지 후 1분 내 미재개 실격"
   리스크가 크게 줄어든다 (`race_fsm.py:89-102, 198-220`, `safety_monitor.py:163-181`).
2. **추월 완료 게이트 수치 이식**: 측면 60~100° 최소거리 0.40m(통과 0.26~0.34 vs 벽 0.60+) → 2.0s 지연 → zone timeout 12s,
   감시 측은 실측 차선 우선. 우리 lidar 클러스터+상대속도에 그대로 얹을 수 있는 검증된 게이트다 (`overtake.py:26-53`).
   초음파를 완료 판정에 쓰지 말 것(고착 사고 실증).
3. **회피 방향은 카메라, 완료는 LiDAR — 센서 역할 분리**: 진입(box>1900px²)·방향(car_lane)·완료(측면 LiDAR)의 역할표
   (`orda/README.md:275-281`). 우리 cone/lidar/lane 모듈 경계에 같은 원칙 적용 가능. LiDAR range_max(2.0m) 밖 목표에
   존재 판정을 걸지 않기.
4. **정지 신호는 모드가 아닌 제어 오버라이드**: 2·3랩 빨간불은 FSM 전이 없이 speed=0 오버라이드, 신호 회복 시 즉시 해제.
   우리 RaceFSM에 WAIT 계열 상태를 늘리지 않고 SpeedPolicy/safety에 hold 플래그 하나로 처리하는 편이 낫다.
5. **엣지 소비형 이벤트 규율**: 모든 미션 이벤트를 "수신시각 엣지 1회 소비+단조 타임스탬프+state 진입 이후" 로 통일한 것,
   그리고 콘 진입 디바운스(75/3회/0.2s)+**지름길 랩 콘 무시 가드**. 우리 LapTracker(min_gap_s 15s)와 상보적 —
   min-gap에 더해 '같은 엣지 재사용 금지'를 넣으면 중복/누락 방지가 이중화된다.
6. **신호등 튜닝 비대칭 원칙**: 좌회전(놓치면 만회 불가)=미검출 최소화, 정지(오검출=제자리 정지)=오검출 최소화.
   우리 HoughCircles 4구 인덱싱에도 램프별 임계를 따로 잡을 것. 라바콘 주황=정지 오인 리스크는 ROI 상단 제한/하우징 게이팅으로.
7. **부트스트랩 안전장치 소품들**: bag 루프(시각 역행) 감지 리셋, imshow 메인 스레드 규칙, 모델·설정 경로 후보 배열,
   `/xycar_motor` publisher-0 인터록이 있는 bag 하네스.

## 반면교사
1. **인터페이스 바꿀 땐 소비자까지 한 커밋에**: ddm-2026은 신호등을 Int32로 바꾸고 main은 Bool 구독인 채로 이틀째 —
   타입 불일치는 DDS에서 조용히 연결이 안 될 뿐 에러도 안 난다. 우리도 토픽 계약 변경 시 발행·구독·문서·상수 동시 변경 원칙 유지.
2. **존재하지 않는 발행자에 전이 조건을 걸지 말 것**: REJOIN 탈출이 production에 없는 `/lane_valid`에 걸려 있어, 지금 실차로 돌리면
   라바콘 뒤 영구 대기다. 우리 FSM의 모든 전이 조건은 "현재 그래프에 실제 publisher가 있는가"를 CI에서 검사하는 게 안전하다.
3. **오프셋 거동 기반 차선변경 완료 판정은 실차에서 깨진다**: 스파이크 400px 조건은 실측 최대 313px로 한 번도 성공 못 했다.
   완료 판정은 (그들처럼) 실측 차선 분류(디바운스 5프레임) 같은 위치 기반으로.
4. **대회 직전의 임시방편은 기록으로 남긴다**: 2025의 is_change_end()=True, 피드백 주석 처리 같은 결정이 이력 없이 코드에 남으면
   이듬해에 그대로 함정이 된다(실제로 2026이 이를 파헤치는 데 상당한 실측을 썼다).
5. **시뮬 값의 실차 이식 금지**: qual의 HSV 채도 205+, CONE_SEPARATION 6.153m 등은 스케일 자체가 다르다.

## 참고: 그들과 우리의 구조 대응표
| ddm-2026 | kmu_race | 비고 |
|---|---|---|
| race_fsm.Mode 10상태 | core/RaceFSM | 이들은 REJOIN·FIXED_AVOID·OVERTAKE·SHORTCUT을 별도 상태로 |
| 신호등 조우 랩 경계 | core/LapTracker(겐트리 목격+min_gap_s) | 같은 사상, 중복 방지 메커니즘만 다름 |
| control.py PD+각도 감속 | core/퓨어퍼수트+PID+SpeedPolicy | 이들은 퓨어퍼수트 없음(픽셀 PD 직결) |
| overtake.OvertakeGuard | core/TargetPassing | 이식 후보 1순위 |
| safety_monitor+복귀형 STOP | core/safety(정지 최후수단) | 복귀 로직·기동 유예가 우리에게 없음 |
| 노란 중앙선 추적+ref 전환 | perception/lane(BEV+흰선) | **차선 기준이 정반대** — 흰 실선 기반인 우리와 달리 중앙선 단일 추적. 중앙선이 가려지는 구간(라바콘 뒤 등) 거동 차이에 유의 |
