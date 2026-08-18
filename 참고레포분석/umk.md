# UMK (mastic-choi/UMK) 정밀 분석 보고서

- 분석일: 2026-08-13 / 분석 대상 HEAD: `f2a9f63` (2026-08-12 12:22 KST)
- 대상 경로: `scratchpad/repos/umk` (이하 경로는 레포 루트 기준. 인용은 `파일:라인`)
- 요약: **2026 제9회 국민대 자율주행 경진대회 본선을 노리는 활성 경쟁팀.** ROS2 Humble + 실차(xycar, Jetson) 기반. 차선인식은 TwinLiteNetPlus ONNX 세그멘테이션(자체 파인튜닝)이 기본, 조향은 Pure Pursuit. 실주행 튜닝 흔적이 매우 짙고(일자별 실측값·실패영상 분석 기록), 반면 미션 레이어(B2/B3/S2/S3)는 대부분 "실차 미검증"으로 자가 표기돼 있음.

---

## A. 메타

| 항목 | 내용 | 근거 |
|---|---|---|
| 대상 대회 | 국민대 자율주행 경진대회 2026년 9회 본선. 규정 PDF `2026년-9회대회-경주진행방법-7월29일자버전-1.pdf`(자이트론) 요약이 통째로 문서화됨 | `track_drive/track_drive/README.md:7-115` |
| ROS | **ROS2 Humble**(ament_python, `track_drive/package.xml:17`) + 모터/VESC는 Docker ROS1(noetic) 컨테이너 + `ros1_bridge` 경유 | `README.md:156-158`, `JETSON_SETUP.md:13-15` |
| 시뮬/실차 | 실차 전용(시뮬 코드 없음). Jetson 보드 교체 체크리스트까지 존재 | `track_drive/track_drive/JETSON_SETUP.md` 전체 |
| 커밋 | 총 160커밋, 2026-07-04 ~ 2026-08-12. 피크: 7/22(15), 7/24(18), 8/4(11), 8/5(22), 8/6(28), 8/7(11), 8/10(9), 8/11(13) — **거의 매일 실차 테스트 후 커밋** | `git log` |
| 저자 | mastic-choi(48), 이지유(43), CHOI JUNSOO(35), xycar(23, 실차에서 직접 커밋), yunyunsung(9) 등 7명 | `git shortlog` |
| 커밋 메시지 경향 | "실측값 반영", "실차 테스트 시작", "○○ 버그 수정(디버그 영상 시각 명시)", "회귀 발견 및 복원" 등 실측 기반. 예: `32690a8 DL_DA_MAX_AREA_PX 실측값 반영 (62478 플레이스홀더 → 13700)`, `a5b1046 B2/B3 실차 테스트 시작: TEST_DISABLE_B2_B3 True → False` | `git log` |
| 개발 방식 | **Claude Code로 개발** — 루트 `CLAUDE.md`에 레포 관례·git 자동화 범위·실차 교훈을 축적. README는 "날짜·원인·수정·알려진 한계"를 남기는 살아있는 설계문서 관례 | `CLAUDE.md:16-26,36-44` |
| 실주행 증거 | bag 파일은 없음(대신 `.mov` 실차 영상을 로컬 분석: `08_10 오후 1.03.02.mov` 프레임 단위 분석 기록 `README.md:1102-1116`). 실측값의 구체성(속도 회귀 원시데이터, BEV 클릭 좌표, da 면적 3프레임 실측치)과 "커밋 80aefe3에서 수정이 실수로 되돌아간 걸 git show로 발견"(`README.md:375-386`) 같은 흔적이 실주행을 강하게 뒷받침 | §H 참조 |
| 특이사항 | 루트 `README.md`는 "test" 한 줄뿐(위장?) — 실제 문서는 `track_drive/track_drive/README.md`(2,114줄). `track_drive`에 `setup.py`가 커밋 안 돼 있음(실차 로컬에만 있는 듯 — 이 레포만 받으면 colcon 빌드 불가) | `find` 결과 |

동봉 모델 파일(내용 미열람, 목록만): `track_drive/track_drive/models/`에 `twinlitenetplus_medium_v2.onnx(+.data 3.6M)`(현재 기본), `twinlitenetplus_small_bootstrap_v2.onnx`, `best.onnx`(원조 TwinLiteNet). `yolo_ros/`에 `cone_best_n.onnx(12M)`(라바콘 YOLO, 현재 사용), `cone_best.pt(50M)`, `yolov8n/8m.pt`(미사용).

## B. 아키텍처

- **단일 노드 모놀리스**: `track_drive/track_drive/track_drive.py`(2,115줄)의 `TrackDriverNode` 하나가 인지→판단→제어→발행 전부 수행. 모듈은 `perception/`(dl_lane, hough_lane, lane_util, perc_lavacon, perc_floor, traffic_signal, yolo_cone), `controller/`(pure_pursuit, lqr, stanley, obstacle_avoidance), `planner/`(hybrid_astar, occupancy), `localization/`(pose_estimator)로 분리.
- **튜닝값 단일화**: 거의 모든 파라미터가 `config.py`(973줄) 한 파일. 모듈들이 `from .config import *`로 공유(`config.py:1-22`).
- **2중 FSM** (`config.py:34-52`):
  - `MissionState`: `S0_WAIT_GREEN → S1_LANE_FOLLOW → S2_INTERSECTION → (직진→S1 / 좌회전→S3_SHORTCUT→S1) → S4_FINISH`
  - `BehaviorState`: `B0_NORMAL / B1_LAVACON / B2_OBSTACLE / B3_VEHICLE` — S1 안에서만 활성(`track_drive.py:1992-1996`)
  - `Phase`: `LAVACON → FIXED_OBSTACLE → VEHICLE → DONE` **순차 고정**(우선순위 판단 없음). `RESET_PHASE_EACH_LAP=True`로 매 랩 LAVACON부터 재시작(`config.py:817`, `track_drive.py:1008-1017`)
- **전이 조건 요약**: S0→S1 = `signal_straight_confirmed`(3프레임 디바운스). S1→S2 = 정지선 감지+쿨다운(`track_drive.py:1110-1112`). S2→S1/S3 = 신호 확정 후 `S2_COMMIT_T=1.0s` 동안 비전 무시 직진 커밋(`track_drive.py:1133-1146`) 후 직진이면 Behavior 활성화+S1, 좌회전이면 개루프 좌회전→S3. S3→S1 = 정지선 or `SHORTCUT_MAX_T=15s` 타임아웃 후 개루프 좌회전. S4 = 3랩 완료(`_begin_new_lap`, `track_drive.py:1004-1006`).
- **메인 루프**: `create_timer(0.05, control_loop)` = **20Hz**(`track_drive.py:330`). DL 추론과 YOLO는 각각 **별도 데몬 스레드**에서 자기 페이스로 돌고 `detect()`는 논블로킹 최신값 반환(`dl_lane.py:30-42`, `yolo_cone.py:17-19`).
- **모터 출력**: `xycar_motor` 토픽, `Float32MultiArray [angle, speed]`, **한 틱에 7회 반복 발행**(`track_drive.py:368-382`). ROS1 xycar_motor 노드가 이 표준 메시지를 받도록 개조돼 있음(커스텀 `XycarMotor`는 ros1_bridge 통과 불가 — 우리와 동일한 우회).
- 센서 구독: `/usb_cam/image_raw/front`(RELIABLE depth1), `/scan`, `/imu`, `/vesc_speed_erpm`(ROS1 중계) (`track_drive.py:316-329`). launch에서 카메라는 `/dev/videoCAM` + `pixel_format='yuyv'` 강제(mjpeg2rgb는 SIGABRT 재현, `launch/track_drive.launch.py:17-47`).

## C. 미션별 구현

### C-1. 신호등 (S0 출발 / S2 교차로 — 통합 4구)
- **방법**: HoughCircles + **위치 인덱싱 + 밝기 대비** (색상 Hue 안 봄). `perception/traffic_signal.py:90-100` — `cv2.HoughCircles(HOUGH_GRADIENT, dp=1, minDist=20, param1=40, param2=20, minRadius=15, maxRadius=25)`.
- **4구 인덱싱**: 검출 원을 x정렬 → 좌→우 = [빨강, 노랑, 좌회전, 직진](`traffic_signal.py:149`). 각 원 중심의 r/2 사각 패치 평균 그레이 밝기를 구해 `밝기-평균 > SIG4_BRIGHT_MARGIN(15)`이면 점등(`traffic_signal.py:150-155`, `config.py:928`). `left_on=좌회전등`, `straight_on=직진등∧¬좌회전`, `red_on=빨강∧¬(좌회전∨직진)`(`traffic_signal.py:157-159`).
- **4개 초과 대응**: `pick_best_4()`가 4개 조합 전수탐색으로 배치검사(세로퍼짐≤2R_max, 가로퍼짐≤11R_max, 인접간격≥3R_min — `traffic_signal.py:20-22`) 통과 조합 중 세로퍼짐 최소를 채택(`traffic_signal.py:71-88`). 10개 초과면 노이즈로 즉시 실패(`config.py:929`). **4개 미만이면 폴백 없이 그 프레임 실패**(README 자인, `README.md:554-556`).
- **ROI**: 프레임 비율 T/B=0.08~0.28, L/R=0.04~0.78 (`config.py:925-926`) → 640x480 기준 y 38~134, x 25~499.
- **출발 디바운스**: `SIG_CONFIRM_FRAMES=3` 연속 유지 시 confirmed 승격(`config.py:887`, `track_drive.py:534-537`). S0는 `signal_straight_confirmed`(=초록만 점등)로 출발(`track_drive.py:1070-1078`) — **빨간불 소등 감지 없이 초록 점등만 봄**.
- 진단 로그가 강력: `[SIG] roi=/circles=/reason=(circle_count·vert_spread·gap 등)/bright=`로 실패 단계를 즉석 구분(`track_drive.py:2035-2066`).

### C-2. 차선
- **3중 백엔드 플러그블**(`config.py:67`): `'dl'`(기본, TwinLiteNetPlus ONNX) / `'hough'`(대안·라바콘 실차검증됨·dl 초기화 실패 시 자동 폴백 `track_drive.py:394-403`) / `'classic_cv'`(보존용).
- **DL 백엔드**: 입력 640x384(`dl_lane.py:95-96`), 원본 프레임 전체 리사이즈→BGR→RGB→/255(정규화 없음). 출력 `da`(주행가능영역)/`ll`(차선) 각 (1,2,384,640) logit→softmax. provider 우선순위 TensorRT>CUDA>CPU + TRT 엔진 캐시(`dl_lane.py:270-296`). 자체 파인튜닝 모델 `twinlitenetplus_medium_v2.onnx`(1,068장 학습, da mIoU 0.937 / ll IOU 0.567 — `README.md:1489-1519`).
- **ROI/BEV**: ROI는 원본 480행 기준 y=250~390(`config.py:209-210`). `DL_USE_BEV=True`(`config.py:182`) — 실측 4점 `(246,257),(455,257),(635,333),(60,333)`(`config.py:186-191`), 스케일 `DL_PIXELS_PER_METER=200px/m`(설계값), 근거리 기준 0.7m 초과 원거리 크롭(`config.py:206`). 캔버스 크기는 4점+스케일에서 자동 역산(`dl_lane.py:143-168`).
- **이진화**: da `DL_FG_THRESHOLD=0.5`, ll `DL_LL_FG_THRESHOLD=0.5`(0.7→0.5 인하 이력, `config.py:212-232`).
- **밴드 처리**: 세로 8밴드(`DL_N_SLICES=8`), 근거리 2/원거리 2밴드로 offset/lookahead 산출, 이상치 40px 제외, 디바운스(20px 점프/2프레임 연속)(`config.py:237-263`).
- **중심선 계산 모드 3종**(`config.py:406` `DL_CENTER_MODE='ll'`이 현재 기본):
  - `'da'`: da 최대 덩어리 무게중심. 덩어리 선택 우선순위 **①시드(ROI 최하단 10행×중앙±70px와 맞닿은 덩어리) → ②직전 프레임 근접(연속성) → ③면적순위**(`config.py:284-295`, README §2.8/§2.11). **면적 임계값만으로 고르는 방식은 실차에서 실패 확정, 포기**(`CLAUDE.md:28-34`).
  - `'ll_da'`: "corridor" — ll 선 1~3번째를 도로 폭으로 삼고 그 안의 da 열린 구간 중심(`config.py:352-367`).
  - `'ll'`(기본): ll을 CCA 단위 HSV 노랑 투표(`DL_LL_YELLOW_VOTE_RATIO=0.35`)로 흰/노랑 분리 후, **노란 중앙선+판정된 한쪽 흰선 짝짓기**(`_ll_yellow_white_centers`, `config.py:368-405`). 노랑-흰 간격 EMA(초기 80px=실측 0.4m, 클램프 50~110px — 폭주 사고 실측 후 도입 `config.py:498-508`). 노랑 소실 시 150px 광역 흰선 탐색 3분기(2W/1W/LOST). 2차 스위치 `DL_LL_ALGO='yw'|'lr'`(두 팀원 구현 공존, `config.py:408-433`).
- **한쪽 차선만 보일 때**: `'ll'` 모드는 간격 기반 반대편 추정(저신뢰 플래그 → 속도 `SPEED_LL_DEGRADED=5.0` 강제 `track_drive.py:1370-1373`); hough 백엔드는 러닝 도로폭(초기 380px, 180~400 클램프, EMA 0.1)으로 반대편 추정(`hough_lane.py:67-72`).
- **da-옆차선 병합 방어**: ll 잔상 decay(α=0.8, 3~4프레임 유지)+기대 반폭 가상경계로 da 클리핑(`config.py:327-342`, README §2.14). 침식(erosion)은 실차 캡처로 무효 확인 후 폐기.
- **프레임 간 안정화**: 경로 EMA `PATH_EMA_ALPHA=0.25`(`config.py:867`), 밴드 간 속도예측+프레임 간 앵커링(α=0.35)을 세 모드 전부에 적용(README §2.27, 2026-08-12 — 최신 작업).
- **인지 생존 가드**: DL 워커 `result_seq`가 `LANE_STALE_SEC=2.0s` 동안 정지면 `lane_stale` → 속도 5.0 제한(`config.py:716-737`, `track_drive.py:459-472`).
- 정지선: 640x480 고정 ROI y270~320/x150~480, 그레이 180 초과 비율 6% 임계(`perc_floor.py:12-13`, `config.py:932-933`).

### C-3. 라바콘 (B1)
- **진입 트리거**: (YOLO 카메라 콘 검출 **AND** 라이다 좌·우 클러스터 동시검출)이 5프레임 연속(`config.py:871`, `track_drive.py:816-824`). 라이다 트리거 ROI는 전방 0.3~0.5m × 좌우 ±2.0m, 클러스터=인접 인덱스 2점 이상+거리편차≤0.35m(`track_drive.py:749-753`). YOLO(YOLOv8n ONNX, nms 그래프 포함, conf 0.5, 별도 스레드 — `yolo_cone.py`, `config.py:880-882`)는 초기화 실패 시 라이다 단독으로 자동 폴백.
- **경로 생성**: HSV가 아니라 **라이다 점군 → scipy Voronoi 골격**. 콘 후보(전방 0~4m, |y|<2.5m) 점군의 보로노이 정점 중 ROI 내 정점을 x오름차순 정렬한 것이 중심선 경로(`perc_lavacon.py:100-168`). 좌우 콘 개수 비대칭(급커브)에도 위상적으로 올바른 중심선이 나온다는 논리.
- **조향**: [2026-08-11] 전용 P게인(`LAVACON_KP=210`) 폐기 — 보로노이 경로를 `1m=200px` 스케일로 변환해 **라인주행과 완전히 동일한 Pure Pursuit/LQR에 태움**(`track_drive.py:712-731,1703-1716`).
- **속도**: `SPEED_LAVACON=2.5`(`config.py:80`).
- **이탈 판정**: 좌·우 모두 콘 소멸 시 done(한쪽만 비면 계속 — 틈 오판 방지, `perc_lavacon.py:107-114`) + 80프레임(≈4초) 디바운스 후 Phase.FIXED_OBSTACLE 전환(`track_drive.py:1718-1726`).
- **진입 latch**: 트리거 확정 후 한쪽 클러스터가 끊겨도 B1 유지(`_lavacon_engaged`, `track_drive.py:1267-1275`).

### C-4. 고정장애물 vs 방해차량 판별
- **핵심: 실시간 판별을 하지 않음.** 대회 규정상 미션 순서가 고정이므로 **Phase 순서(라바콘→고정장애물→방해차량)가 곧 분류**(`obstacle_avoidance.py:24-26`, README §4.1). 
- 보조 신호 2개는 로그 전용:
  - 라이다 **횡폭 분류**: 타겟 폭 ≥ `OBSTACLE_VEHICLE_WIDTH_M=0.24` → 'vehicle'(실측: 고정장애물 20cm / 방해차량 28cm 폭의 평균, `config.py:969-972`, `track_drive.py:606-614`). 점 개수 분류는 거리 의존성 때문에 폐기(주석에 실측 반례 기록).
  - **교차확인 로그**: `target_speed_est = v_mps(VESC 실측) + obstacle_rate(라이다 접근율 EMA)`가 0.3m/s 미만이면 "정지처럼 보임" — Phase 가정과 어긋나면 경고만(`track_drive.py:1681-1700`, `config.py:894`).
- 트리거: B2 = 전방 장애물 거리 < `SAFETY_DIST=5.0m`(`config.py:884`), B3 = 거리 < `OVERTAKE_TRIGGER=6.5m` 5프레임 연속(`config.py:885-886`, `track_drive.py:914-920`).

### C-5. 추월 (B2/B3 공용 `TargetPassing`)
- **방향 결정** `choose_side()`(`obstacle_avoidance.py:81-117`): ①타겟 횡위치(왼쪽에 있으면 오른쪽 통과 — 규정 "주행하지 않는 차선") ②정면(|y|<0.12m)이면 비어있는 쪽(`left_clear/right_clear` — 라이다 측면 ROI 점수) ③둘 다 비면 노란선 건너편(현재 `lane_side`의 반대). 0(양쪽 막힘)이면 'blocked'.
- **궤적 = 횡오프셋 스텝 응답**: `SHIFT`에서 목표 오프셋 `lane_offset + side*PASS_OFFSET(80px=0.4m 실측 차선폭×200px/m)`로 1차 지연 수렴(`LATERAL_ALPHA_OUT=0.12`/틱), 조향은 그 오프셋을 기존 **차선 PID**(`LANE_KP=0.70, KI=0.0008, KD=0.15` + lookahead 0.4 가중 preview)에 입력(`track_drive.py:1941`, `obstacle_avoidance.py:159,180-181`). 시간 기반 개루프 아님.
- **복귀(90cm 규정)**: `ALONGSIDE`에서 타겟 미검출 6프레임 연속 → `RETURN`, 복귀 수렴율 `LATERAL_ALPHA_BACK=0.16`을 나갈 때(0.12)보다 **의도적으로 빠르게** 설정("90cm 규정 때문에 늑장 부리면 차선이탈" — `config.py:906`). 오프셋<5px면 완료. 90cm를 거리로 재지는 않고 "빨리 복귀"로만 대응.
- **B3 특화**: SHIFT 중 타겟이 내 통과 방향으로 넘어온 상태 8프레임(`SWITCH_FRAMES`) 지속 시 반대쪽이 비어있으면 방향 전환, 아니면 RETURN 후 재시도(`obstacle_avoidance.py:141-155`). 속도: shift 8.0/alongside 12.0/return 10.0(생성자 기본값, `obstacle_avoidance.py:57-63`). 추돌 방지: 거리<0.6m면 shift 속도의 30%로(`obstacle_avoidance.py:184-187`).
- **blocked 시 절대 정지 안 함**: `max(SPEED_NORMAL*0.15, 0.5)` 서행 유지 — "정지 후 1분 실격" 규정 명시적 대응(`track_drive.py:1943-1949`).
- **B2 게이트**: `_da_avoidance_failed()` — da 경로가 알아서 피하면 개입 안 함 설계지만, da가 장애물 미학습이라 현재 `da_unaware_of_obstacle=True` 하드코딩으로 **항상 TargetPassing 개입**(`track_drive.py:1759-1763`).
- **B3 Hybrid A* 대안**(`USE_HYBRID_ASTAR_FOR_B3=False` 기본): 그리드/충돌검사 매틱 + 재탐색 트리거(경로무효/타겟진입/4틱 주기), 실패 3틱이면 TargetPassing 폴백 latch, Stanley 추종 + IMU yaw/VESC 적분 로컬 pose(`track_drive.py:1798-1909`). B2용 A*는 "구조화된 2차선에서 검색은 과함" 결론으로 삭제.
- **교차로 근처 기동 금지**: 정지선 목격 후 2.0s간 회피/추월 시작 금지(`config.py:952`, `track_drive.py:946-951`). 장애물 인식 끊김 0.6s 보상(`OBSTACLE_HOLD_T`).

### C-6. 지름길 (S2→S3)
- **좌회전 신호 인식**: 4구 신호등의 좌회전 화살표 위치 점등 = `signal_left_confirmed`(3프레임). S2는 신호 인식까지 **완전 정지 대기**(`track_drive.py:1148`).
- **커밋 구간**(핵심 아이디어): 신호 확정 후 `S2_COMMIT_T=1.0s` 동안 **비전 무시하고 각도 0, `APPROACH_SPEED=2.0` 직진** — 신호 지점과 물리 분기 지점이 ~1m 떨어져 있고, 분기가 보이는 순간 da가 반대 갈래로 끌려가는 실측 실패를 차단(`config.py:150-165`, `track_drive.py:1133-1146`).
- **진입 좌회전 = 개루프 스크립트**: `TURN_ANGLE=-60°`, `TURN_SPEED=15`, `TURN_FRAMES=40`(2초) 고정 유지 후 S3 전환(`config.py:137-142`, `track_drive.py:1236-1254`). IMU yaw는 플래그로만 쓰고 각도 피드백 없음.
- **S3 주행**: 차선 유효하면 `_lane_drive()`(차선 추종), 무효면 직진. 진입 1초 후 기준 yaw 저장 → `SHORTCUT_VISION_CUTOFF_T=10s` 이후엔 비전 끄고 **IMU 헤딩홀드**(오차×100, ±30° 클램프)로 직진(합류부에서 da가 먼저 벌어지는 실패 대응, `track_drive.py:1158-1213`, `config.py:936-947`).
- **합류(끝) 판정**: `SHORTCUT_MIN_T=3s` 이후 정지선 검출 or `SHORTCUT_MAX_T=15s` 강제 → 감속 1초(헤딩홀드) → 진출 좌회전(-60°, 40프레임) → S1 복귀 + 정지선 쿨다운 3s(`track_drive.py:1183-1198,1215-1222`).
- 전체가 `TEST_DISABLE_INTERSECTION=True`로 아직 봉인 상태(실차 미검증 자인, `config.py:783-793`).

### C-7. 랩 카운팅
- **주 신호 = IMU yaw 누적**: 프레임 간 yaw 증분(±π 언랩)을 적분, `|누적| ≥ 330°`가 10프레임(0.5s) 연속이면 랩 확정(`track_drive.py:959-991`, `config.py:811-816`).
- **보조 = 정지선**: 누적 ≥270°인 상태에서 정지선을 보면 조기 확정(랩당 1회 제한 `_lap_stopline_used`). 정지선 단독 신뢰는 "흰 노면표시 오검출 재현됨"으로 금지(`track_drive.py:985-988`).
- **중복 방지**: 랩당 최소 20s(`LAP_MIN_T`) + 임계 초과 연속 10프레임 디바운스. 출발 시 S0→S1 전환 때 기준 리셋(대기 시간 미포함, `track_drive.py:1057-1063`).
- **약점(자인)**: 휠/비전 폴백 전무 — IMU 고장 동안 랩이 영원히 0이었던 실사례(`README.md:2094-2110`). IMU 죽음을 알리는 전용 디버그 창도 아직 없음(`README.md:2112-2114`).

## D. 제어

- **조향 컨트롤러 플러그블**: `STEERING_CONTROLLER='pure_pursuit'`(기본) | `'lqr'`(실차 미검증)(`config.py:595`).
- **Pure Pursuit**(`controller/pure_pursuit.py`, 픽셀 좌표계 1m=200px):
  - lookahead = clip(`90 + 4.0×speed`, 40, **190**) × curvature 감쇠(`config.py:600-613`). 상한 150→190 상향은 "속도 20에서 진동" 실차 증상의 수정.
  - curvature 감쇠 = `1/(1+100×max(|probe_curvature|, |imu_curvature|))` — probe는 **매 프레임 고정 기준거리에서 재계산**(직전값 재사용 시 자기순환 락업 버그를 실차 영상으로 확인 후 수정, `pure_pursuit.py:205-236`). IMU 항은 `yaw_rate EMA(α=0.3)/v_mps/200`으로 실측 곡률 병합(IMU+VESC 생존 시만, `track_drive.py:1401-1431`).
  - 조향각 = `atan(curvature × PP_WHEELBASE_PX)` — `PP_WHEELBASE_PX=67.0 = 실측 축거 0.335m × 200px/m`(80→67 물리 기반 교체, `config.py:614-625`).
  - 스무딩: 조향각 EMA `PP_ALPHA=0.35`(0.5→0.35), dx 데드존 `15px`(6→15), 경로 EMA `PATH_EMA_ALPHA=0.25`(0.4→0.25) — 전부 2026-08-12 "직진 진동" 대응 3종 세트(`config.py:626-637,867`).
  - ld 분모 바닥 `min(90, lookahead)`(짧은 경로 노이즈 증폭 방지 + 코너 축소 효과 보존, `pure_pursuit.py:247-259`).
- **LQR**(`controller/lqr.py`): 2-state [횡오차, 헤딩오차] 운동학 모델. **단위 버그 교훈**: Q=diag(1,1)로 px와 rad를 같은 가중 → e_y=-40px에 raw 1234° 폭주. BEV의 200px/m로 e_y를 미터 환산하는 "미터 모드"로 해결(`lqr.py:9-24`). 파라미터 `config.py:658-674`(Q=1/1, R=1, α=0.5, wheelbase 0.335m).
- **구 PID**: `LANE_KP=0.70, KI=0.0008, KD=0.15`, anti-windup ±10°, 코너 부스트 1.8배(offset 40~120px 구간), 데드존 40px — 현재 B2/B3 오프셋 조향에만 사용(`config.py:833-840`, `track_drive.py:1643-1659`).
- **속도 정책**(`_lane_drive`, `track_drive.py:1336-1391`):
  - 기본 `SPEED_NORMAL=15.0`(8→25→15 이력, `config.py:73`), 코너 하한 `SPEED_CORNER_MIN=5.0`(모터 데드존≈1.4 위로, `config.py:82-90`), 라바콘 2.5, 접근 2.0, blocked 서행 `max(2.25,0.5)`.
  - 감속 신호 = `max(코너 EMA 신호, 원거리 lookahead preview×0.3)`의 **3제곱** 감속식 + Nav2 Regulated PP식 회전반경 감속(`반경<250px`이면 비례 감속, 하한 배율 0.35)(`track_drive.py:1356-1363`, `config.py:130-134`).
  - **진동-코너 구분**: 감속 판단 입력을 조향각의 **signed EMA(α=0.15)** 후 abs — 진동은 상쇄되고 실제 코너만 남음(`track_drive.py:1340-1356`). 시뮬 검증 수치까지 기록(README §0.5.3).
  - 가속 램프 `SPEED_ACCEL_STEP=0.4`/틱(0.85→0.4, **배터리 전압강하 LVC 트립 의심** 완화 — `config.py:107-115`), 감속은 즉시. 코너 hold peak-decay(0.92~0.97).
  - 저신뢰/스테일 상한: `SPEED_LL_DEGRADED=5.0`, `SPEED_LANE_STALE=5.0`.
- **출력 스무딩**: `ANGLE_MAX=80°`(100→80), **조향 변화율 제한 `ANGLE_RATE_MAX=12°/틱`(=240°/s)** — 개루프 좌회전·behavior 전환의 계단 명령까지 일괄 완충(`config.py:105-106`, `track_drive.py:371-378`).

## E. 튜닝값 총표 (우리 초기값 후보)

### E-1. 차량/트랙 실측 (동일 xycar 플랫폼 — 최우선 재사용 후보)
| 항목 | 값 | 출처 |
|---|---|---|
| 축거(wheelbase) | **0.335 m** (줄자 실측) | `config.py:667` |
| 차체 | 폭 0.31 × 길이 0.64 × 높이 0.20 m | `config.py:967-968` |
| 도로 전체폭(흰선-흰선) | 0.8 m, 노란선 정중앙 → 차선폭 **0.4 m** | `config.py:965`, README §6.1 |
| 고정장애물 모형 | 20×41×16 cm | README.md:1838 |
| 방해차량 모형 | 28×54×19 cm | README.md:1839 |
| 차량/고정물 폭 분류 임계 | 0.24 m | `config.py:972` |
| **라이다 장착 오프셋** | **80°** (정면=인덱스 80) | `config.py:830`, README §6.2 |
| 라이다 차체 자기가림 | 인덱스 **215~305** 마스킹 | `track_drive.py:555`, `perc_lavacon.py:51` |
| 속도단위→m/s | **0.1347 m/s per unit** (speed 5→0.487m/s, 10→1.16m/s 회귀; **데드존 ≈ speed 1.4 미만**) | `config.py:964,958-963`, README §6.5 |
| VESC ERPM 게인 | speed_to_erpm_gain = **4614.0** | `config.py:684` |
| BEV 4점(640x480) | TL(246,257) TR(455,257) BR(635,333) BL(60,333); W=0.8m, L=1.0m | `config.py:186-191`, README §6.3 |
| BEV 스케일 | 200 px/m (설계값) | `config.py:192` |
| IMU | SparkFun 9DoF Razor M0, /dev/ttyIMU 57600bps | README §8, `JETSON_SETUP.md` |

### E-2. 속도/조향
| 파라미터 | 값 | 위치 |
|---|---|---|
| SPEED_NORMAL | 15.0 (이력 8→25→15) | config.py:73 |
| SPEED_LAVACON | 2.5 | config.py:80 |
| SPEED_CORNER_MIN | 5.0 | config.py:90 |
| SPEED_LL_DEGRADED / SPEED_LANE_STALE | 5.0 / 5.0 | config.py:97,104 |
| APPROACH_SPEED(+EXIT) | 2.0 / 1.0s 유지 | config.py:145-148 |
| ANGLE_MAX / ANGLE_RATE_MAX | 80° / 12°·틱⁻¹ | config.py:105-106 |
| SPEED_ACCEL_STEP | 0.4/틱 (배터리 대응) | config.py:115 |
| CORNER_SIGN_EMA_ALPHA | 0.15 | config.py:124 |
| CORNER_MIN_RADIUS_PX / MIN_SCALE | 250 px / 0.35 | config.py:133-134 |
| 코너 감속식 | `SPEED_NORMAL×(1−0.90×turn³)`, hold decay 0.92/0.97 | track_drive.py:1359-1360, config.py:116-117 |
| TURN_ANGLE/SPEED/FRAMES (S2→S3) | −60° / 15 / 40프레임 | config.py:137-139 |
| TURN_EXIT_* (S3→S1) | −60° / 15 / 40 | config.py:140-142 |
| S2_COMMIT_T | 1.0 s | config.py:165 |

### E-3. Pure Pursuit / LQR / PID
| 파라미터 | 값 | 위치 |
|---|---|---|
| PP_LOOKAHEAD_BASE/GAIN/MAX/MIN | 90 / 4.0 / 190 / 40 px | config.py:600-613 |
| PP_LOOKAHEAD_CURVATURE_GAIN | 100.0 | config.py:612 |
| PP_WHEELBASE_PX | 67.0 (=0.335×200) | config.py:625 |
| PP_ALPHA / PP_DX_DEADZONE_PX / PP_MIN_LOOKAHEAD_PX | 0.35 / 15.0 / 90.0 | config.py:630-637 |
| PATH_EMA_ALPHA | 0.25 | config.py:867 |
| IMU_YAW_RATE_EMA_ALPHA | 0.3 | config.py:706 |
| LQR Q/R/α/dt | 1.0/1.0/1.0, R=1.0, α=0.5, dt=0.05 | config.py:660-666 |
| LQR 미터모드 | wheelbase 0.335m, probe 0.3m, min_path 0.3m | config.py:667-674 |
| PID(B2/B3용) | Kp 0.70, Ki 0.0008, Kd 0.15, I클램프 ±10°, 데드존 40px, 코너부스트 1.8(40~120px) | config.py:833-840 |

### E-4. DL 차선 (핵심만)
| 파라미터 | 값 | 위치 |
|---|---|---|
| 입력/ROI | 640×384 추론, ROI y=250~390 | dl_lane.py:95-96, config.py:209-210 |
| DL_FG / DL_LL_FG | 0.5 / 0.5 | config.py:212,232 |
| 밴드 | 8밴드, 밴드당 최소 da 40px / ll 15px | config.py:237-238,468 |
| 이상치/디바운스 | outlier 40px, 점프 20px, 2프레임 | config.py:251-263 |
| da 면적 | 하한 1560px², (구)상한 16000px²(직선 실측 13,219±) — 상한은 'da'모드에서 제거됨 | config.py:268-282 |
| da 시드 | 하단 10행 × 중앙±70px | config.py:294-295 |
| ll 탐색창 | 반경 60px, 미검출 시 +15px씩 최대 120px 확장 | config.py:480,548-549 |
| 노랑-흰 간격 | init 80px(실측 0.4m), EMA 0.1, 클램프 50~110px | config.py:495-508 |
| 노랑HSV | lower(15,80,80) upper(40,255,255), 투표비 0.35 | config.py:564-577 |
| ll 잔상 | decay 0.8, 최소값 128 | config.py:341-342 |
| 속도예측/앵커링 | vel EMA 0.3(클램프 40px), anchor 0.35 | config.py:546-560 |
| LANE_STALE_SEC | 2.0 s | config.py:737 |

### E-5. 미션/트리거
| 파라미터 | 값 | 위치 |
|---|---|---|
| SIG_CONFIRM_FRAMES | 3 | config.py:887 |
| SIG4 ROI / 반지름 / 밝기마진 | (0.08,0.28,0.04,0.78) / 15~25px / +15 | config.py:925-928 |
| HoughCircles | dp1, minDist20, param1=40, param2=20 | traffic_signal.py:94-98 |
| 정지선 | ROI y270-320 x150-480, 흰색>180, 비율>6% | perc_floor.py:12-13, config.py:932-933 |
| 라바콘 트리거 | 전방 0.3~0.5m × ±2.0m, 클러스터 2점·gap≤0.35m, 5프레임 | track_drive.py:749-753, config.py:871 |
| 라바콘 경로 ROI | 전방 0~4m × ±2.5m, 최소 4점, offset 클램프 ±0.8m | perc_lavacon.py:55-61 |
| LAVACON_DONE_FRAMES | 80 (≈4s) | config.py:870 |
| YOLO 콘 | conf 0.5, 입력 640 | config.py:880-881 |
| B2/B3 트리거 | 5.0m / 6.5m, 5프레임 | config.py:884-886 |
| 전방 장애물 ROI | 0~5m × ±1.5m, 최소 2점; 측면 0.8~5.5m × 0.7~1.5m, 차단임계 좌8/우5점 | track_drive.py:544-552 |
| PASS_OFFSET | 80px (=0.4m) | config.py:901 |
| ALPHA_OUT/BACK, DONE | 0.12 / 0.16 / 8px(내부 5px) | config.py:905-907 |
| CLEAR/SWITCH 프레임 | 6 / 8 | config.py:903-904 |
| MIN_GAP_M / CENTER_DEADZONE_M | 0.6 / 0.12 m | config.py:908,902 |
| 추월 속도 | shift 8 / alongside 12 / return 10 | obstacle_avoidance.py:57-63 |
| 랩 | 330° 확정, 270°+정지선 조기, 최소 20s, 확정 10프레임 | config.py:812-816 |
| OBSTACLE_HOLD_T / MANEUVER_BLOCK | 0.6s / 2.0s | config.py:951-952 |
| SHORTCUT MIN/MAX/CUTOFF | 3 / 15 / 10 s | config.py:936-947 |
| STALE 가드 | VESC 0.5s, IMU 0.5s, VESC_MIN 0.05m/s | config.py:686-698 |

### E-6. Hough 백엔드(대안)
ROI 264~336행(/480), Canny 60/75, HoughLinesP(th40, minLen30, maxGap20), 도로폭 초기 380px(180~400, EMA 0.1) — `hough_lane.py:24-72`.

## F. 실전 노하우 (주석·커밋·문서 속 함정/트릭)

1. **"센서 죽음은 크래시가 아니라 조용한 고정값"** — IMU/VESC/DL 전부 `_*_t` 타임스탬프+`*_STALE_SEC` 가드 패턴 필수(`CLAUDE.md:22-26`). 실사례: IMU 고장으로 랩카운트가 소리 없이 0 고정(README §8.4).
2. **배터리 전압강하로 주행 중 순간 정지** — 명령은 25로 계속 나가는데 VESC 실측만 0으로 추락. LVC 트립 추정, 가속 램프 0.85→0.4로 완화(README §7.1). *고속 세팅 시 배터리 만충 필수.*
3. **모터 데드존 ≈ speed 1.4** — 코너 감속 하한이 이보다 낮으면 "명령은 나가는데 정지"로 굳음(README §2.3). 감속 하한은 데드존 위로.
4. **Pure Pursuit 자기순환 락업** — 직전 프레임 curvature로 lookahead를 줄이면 `curvature=2sin(α)/ld` 증폭 루프에 갇혀 직진에서도 -65.7° 15초 고정(실차 영상). 감쇠 판단은 고정 기준거리 probe로 매 프레임 독립 재계산(README §0.5.2).
5. **진동을 코너로 오인한 감속 요동** — abs를 먼저 취하면 안 되고 **signed EMA 후 abs**(README §0.5.3). 이 수정이 무관한 커밋(80aefe3)에 섞여 조용히 롤백됐다가 재발견됨 — *리뷰 없는 커밋에 diff가 섞이는 사고 실례.*
6. **속도 올리면 진동** — lookahead 상한이 속도 증가를 못 따라가서(150 클램프). 상한을 `base+gain×v_max`로 재계산(README §0.5.6).
7. **LQR 단위 버그** — px와 rad를 같은 Q로 두면 K 폭주(e_y=-40px→raw 1234°). 픽셀↔미터 환산 먼저(README §0.5).
8. **da 블롭 선택은 면적 임계값만으로는 절대 안 됨** — 시드(차량 접촉)>연속성>면적 순. 파라미터 재조정으로도 실패 확인(`CLAUDE.md:28-34`).
9. **BEV 원거리 blur** — 확률맵을 워프하면 원거리 경계 blur가 확대돼 ll이 두껍게 잡힘 → 원거리 0.7m 크롭 + 이진화 임계 분리(README §2.2/§2.5).
10. **글레어/역광에서 세그멘테이션 전멸**(`ll_cov:0.000`) — 잔상/재구성으로도 못 버팀, 미해결 과제로 명시(README §2.18 말미).
11. **ㅓ교차로·지름길 합류부에서 da가 옆 갈래로 끌려감** — 신호/시간으로 확정된 구간은 **비전을 끄고** 커밋 직진/헤딩홀드(S2_COMMIT_T, SHORTCUT_VISION_CUTOFF_T).
12. **EMA 러닝 추정치는 클램프 필수** — 노랑-흰 간격 EMA가 글레어 노이즈로 161px(실측 80의 2배)까지 부풀고 얼어붙어 waypoint가 차선 밖으로(README §2.18). 실측 근방 상하한 강제.
13. **카메라**: `Ctrl+Z` 종료 금지(좀비가 /dev/video0 점유 → `sudo fuser -k /dev/video0`), usb_cam은 `yuyv` 강제 아니면 SIGABRT(README 공통 주의, launch:42-46).
14. **HoughCircles 원 4개 초과는 흔함**(반사광) — 조합 탐색으로 완화하되 미만은 포기. ROI/반지름은 정지 위치에 강결합(README §1 한계).
15. **정지선 오검출**(흰 노면표시) 재현됨 — 정지선 단독으로 랩/전환 판단 금지, yaw 270° 이상+쿨다운과 AND(README, `track_drive.py:985-988`).
16. **라이다 각도/마스크 상수의 파일 간 비동기화 사고** — 구 규약(오프셋0/마스크99~262) 잔존 코드로 좌우 반전+done 상시 True 버그(perc_lavacon.py:28-31). 상수는 단일 소스로.
17. **점 개수로 차량/고정물 분류 금지** — 거리에 따라 뒤집힘(실측: 같은 차가 2.0m에선 11점=fixed, 1.5m에선 13점=vehicle). 횡폭(m)으로(track_drive.py:606-609).
18. 대회 규칙 대응 디테일: blocked 시 완전정지 대신 서행(1분 실격 방지), 복귀 알파를 이탈 알파보다 크게(90cm), 신호 대기 시간은 랩타임 기준점에서 제외(track_drive.py:1059-1063).

## G. 우리 kmu_race 대비

### ① 가져올 것 (구체적)
1. **하드웨어 실측값 전부(E-1 표)** — 같은 xycar 실차라 대부분 그대로 이식 가능: 축거 0.335m, 속도단위 환산 0.1347m/s·unit(+데드존 1.4), VESC ERPM 게인 4614, 도로폭 0.8m/차선 0.4m, 장애물 모형 치수와 폭 분류 임계 0.24m. *단 라이다 오프셋 80°와 차체 마스크 215~305는 개체별 장착 편차 가능 — "정면에 사람 세우고 컴퍼스 디버그 창으로 확인"하는 그들의 캘리브레이션 절차만 가져와 우리 값 재실측.*
2. **조향 변화율 제한 + signed-EMA 코너 신호**: 우리 SpeedPolicy에 `ANGLE_RATE_MAX`(12°/틱)와 "감속 판단은 조향각 signed EMA(0.15) 후 abs" 두 줄이면 퓨어퍼수트 진동↔코너 오인 감속 문제를 선제 차단. 근거와 시뮬 수치까지 문서화돼 있음(README §0.5.3).
3. **스테일 가드 3종 세트**: `_*_t`+`*_STALE_SEC`(IMU/VESC 0.5s, 비전 2.0s) + 스테일 시 "속도 5.0으로 눈에 띄게 감속해 사람이 알게 하는" 신호 설계(config.py:98-104). 우리 race_node에 즉시 이식 가능한 저비용 안전장치.
4. **S2/합류부 "비전 커밋 오프" 패턴**: 신호로 방향이 확정된 분기 구간(S2_COMMIT_T=1.0s)과 지름길 합류부(10s 이후 IMU 헤딩홀드)는 차선인식을 끈다 — 우리 지름길 미션에서 BEV 흰선 검출이 분기에서 끌려가는 동일 실패가 예상되므로 그대로 채택 가치.
5. **추월 복귀 비대칭 게인**(OUT 0.12 < BACK 0.16, 90cm 규정 근거) + **blocked 서행 유지**(정지 1분 실격 방지) + **정지선 후 2s 기동 금지** — 규칙 파생 디테일 3종. 우리 TargetPassing에 수치만 얹으면 됨.
6. **라바콘 보로노이 중심선**(scipy.spatial.Voronoi, perc_lavacon.py): 우리 HSV 게이트 중앙 방식의 라이다 보완/대체 후보. 좌우 콘 개수 비대칭(지그재그 배치)에 강하고, 나온 경로를 기존 퓨어퍼수트에 그대로 태우는 통합(라바콘 전용 게인 폐기)도 검증할 만한 단순화.
7. **신호등 pick_best_4 조합 탐색**(traffic_signal.py:71-88): 우리 signal_detector가 원 4개 초과(반사광) 상황을 못 견디면 이 완화가 그대로 답. 배치 검사 상수(세로≤2R, 가로≤11R, 간격≥3R)도 재사용 가능.
8. **[SIG]/[TRIG]/[LAVA-ROI] 진단 로그 포맷**(track_drive.py:2011-2089): "어느 단계에서 막혔는지"를 실차 옆 터미널만으로 판별하는 설계. 대회 당일 디버깅 속도에 직결.
9. **배터리/가속 램프 교훈**: 고속 세팅에서 출발·재가속 시 LVC 트립 의심 정지(README §7.1). 우리도 가속 스텝 제한과 만충 배터리 체크리스트 반영.

### ② 반면교사 (우리 방식이 나은 지점)
1. **랩 카운팅**: UMK는 IMU yaw 적분 단독(폴백 없음) — IMU가 죽자 몇 주간 랩이 0인 걸 못 알아챔(README §8.4). 우리 LapTracker(신호등 겐트리 목격 크기 기반)는 IMU 무관이라 구조적으로 우월. 다만 그들의 **디바운스 3종(최소 랩타임 20s, 임계 유지 10프레임, 랩당 정지선 1회)**은 우리 쪽에도 이식 가치.
2. **DL 세그멘테이션 차선의 비용**: 7월 말부터 3주간 커밋의 절반 이상이 da/ll 실패모드 대응(§2.1~§2.27, 모델 3회 교체, 자체 라벨링 1,068장)이고, 그 결과조차 "실차 미검증"이라 SPEED_NORMAL을 25→15로 내림. 글레어에서 전멸하는 한계도 미해결. **우리 BEV+흰선 고전 파이프라인 유지가 옳고**, 그들의 실패 카탈로그(원거리 blur, 옆차선 병합, 교차로 끌림)를 우리 검출기의 테스트 시나리오로만 활용.
3. **정지/주행 장애물 판별**: UMK는 런타임 판별을 포기하고 Phase 순서에 의존(라이다 상대속도는 로그 전용). 우리 lidar_tracker(클러스터+상대속도)가 더 일반적이고, 순서가 어긋나는 돌발(콘 잔존 오인 등 — 그들 스스로 B3 오인 진입 위험 명시)에 강함. 단, 그들의 지적("실시간 속도추정은 라이다 노이즈에 취약")은 유효 — 우리 판별에 **트랙 구간 prior(Phase)를 게이트로 결합**하면 양쪽 장점을 다 취함.
4. **2,100줄 모놀리스 노드**: rclpy 없는 순수 로직 분리(우리 core/)가 없어 유닛테스트 불가, 회귀(80aefe3 사고)에 취약. 우리 구조 유지. 단 **config.py 단일 튜닝 파일 관례는 그들이 낫다** — 우리도 게인 산재 시 이 패턴 채택 권장.
5. **미션 레이어 완성도**: B2/B3 실차 테스트는 8/11에야 시작, S2/S3는 아직 플래그로 봉인(`TEST_DISABLE_INTERSECTION=True`), 좌회전은 40프레임 개루프. 본선까지 12일 남은 시점에 미션 검증이 우리보다 뒤일 가능성이 높음 — 우리는 미션 통합 리허설에 시간을 쓰는 것이 상대 우위 전략.

### ③ 우리에 없는 미션 대응 (있는지 여부)
- **지름길(S2/S3) 전체 시퀀스**: 신호 대기 정지 → 커밋 직진 → 개루프 좌회전 → 차선/헤딩홀드 → 정지선/타임아웃 합류 → 진출 좌회전. 우리 kmu_race에 지름길 상태기계가 미비하다면 이 골격(+시간 상수들)이 그대로 출발점.
- **정지선 검출기**(perc_floor.py 6%/180 임계): 우리 스택에 없음 — S1→S2 전환과 랩 보조에 씀. 단순해서 이식 1시간짜리.
- **비전 커밋 오프 구간**(위 ①-4), **기동 금지 가드**(정지선 후 2s), **장애물 홀드**(0.6s): 우리 FSM에 없는 규칙 파생 가드들.
- 반대로 **우리에게 있고 그들에게 없는 것**: 초음파 8ch 활용(그들 전무), 겐트리 기반 랩카운트, 정지/주행 실시간 판별.

## H. 신뢰도 평가

**종합: 상(주행 코어) / 중하(미션 레이어)** — "실제 트랙에서 돌았는가"는 확실히 **예**.

근거(상 방향):
- 160커밋이 7/22~8/12에 매일 집중, 커밋 메시지가 실측·증상 기반("실측값 입력" a75c03f, "B2/B3 실차 테스트 시작" a5b1046, "envidia 설치 리스트"는 실차 Jetson 계정 xycar가 직접 커밋).
- 실측 데이터의 원시값까지 기록: 속도 회귀 4포인트(README §6.5), da 면적 3프레임(13,349/13,361/12,946px), BEV 클릭 좌표, 라이다 오프셋 확인 절차("정면에 사람 세우고"), 실차 영상 타임스탬프 분석(15초 지점 offset +25.3→+87.4 등, §2.18).
- 실패→수정→회귀 재발견의 사이클(80aefe3 사고, §0.5.3)과 하드웨어 사건(IMU 고장/수리, 배터리 전압강하, 카메라 SIGABRT)은 지어내기 어려운 실주행 흔적.
- 문서가 "실측값"과 "설계값(미검증)"을 의도적으로 구분 표기(README §6 도입부).

근거(하 방향, 미션 레이어):
- B2/B3/S2/S3/LQR/'ll'모드/2026-08-12 최신 안정화 전부 **"실차 미검증"을 스스로 명시**. `TEST_DISABLE_INTERSECTION=True` 봉인 상태로 마지막 커밋.
- bag/영상 파일이 레포에 없어 제3자 검증 불가(영상은 로컬 참조만).
- `track_drive/setup.py` 미커밋 — 이 레포 단독으로는 빌드 불가(실차 로컬 상태와 레포가 100% 동일하다는 보장은 없음).

결론: **차선주행+라바콘+신호등 출발까지는 실차에서 상당히 다져진 팀**이고, 튜닝값·실패 카탈로그는 그대로 우리 초기값/체크리스트로 쓸 가치가 높다. 미션(장애물/추월/지름길)은 8/11 이후 막 실차에 진입한 단계라, 본선까지 그들의 완성도는 미지수.
