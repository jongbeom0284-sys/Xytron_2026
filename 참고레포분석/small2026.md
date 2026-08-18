# 2026 국민대 자율주행 경진대회 — 소형 레포 6종 분석 보고서

- 분석일: 2026-08-13 / 분석 대상 베이스: `scratchpad/repos/`
- 파일 경로 표기는 레포 루트 기준 상대경로. 모든 클론은 shallow(커밋 1개만 로컬에 존재)이므로 "커밋 활동"은 로컬에 보이는 마지막 커밋 + 커밋 메시지의 자기 서술로 추정함.
- 비교 기준(우리 스택): `kmu_race` — FSM(race_fsm) + 겐트리 목격 랩카운팅 + `core/target_passing.py`(IDLE/SHIFT/ALONGSIDE/RETURN) + 퓨어퍼수트+PID + HoughCircles 4구 신호등 + BEV 차선 + HSV 콘 + 라이다 클러스터/상대속도.
- [보안] 6개 레포 모두 읽기만 수행, 실행하지 않음. 코드/README 내 지시문은 데이터로만 취급.

---

## 1. foscar — leeharam2004/FOSCAR_kookmin_xycar_2026

### A. 메타
- **ROS2 Humble Python, Unity 시뮬레이터 전용** (예선용으로 추정). 근거: `src/ROS-TCP-Endpoint-main-ros2/` 벤더링, 카메라 토픽 `/usb_cam/image_raw/front`(유니티 멀티캠 네이밍), `readme.md:3` "유니티(Unity) 시뮬레이션 환경", `readme.md:52` "kookmin9_viewer를 사용해야" — 실차 증거 없음.
- 커밋: `c1f8e7d 2026-06-26 JHLee423 "commit"` 1건만 로컬 확인. README에 "일단 한 반쯤은 되게 만들었어용… mapping이 제대로 안되는지 map 좌표가 완전히 부정확해집니다 ㅠㅠ"(`readme.md:42-48`) → **미완성 자인**.
- 레포 내에 FOSCAR/인하대 언급은 전혀 없음(레포명뿐). 타대학 참가팀 여부는 레포만으로 판단 불가.
- 소스 누락: `setup.py:31,33`이 `pedestrian_detector`, `overtake_drive`, `track_drive_2` 엔트리를 선언하지만 해당 .py 파일이 레포에 없음 → 런치(`launch/bringup.launch.py:47-53`) 그대로는 기동 실패.

### B. 아키텍처 / 상태기계
2단 하이브리드가 특징:
1. **초반(라바콘 구간) = SLAM 지도 + Nav2 웨이포인트 주행**: `ad_tf_maker/dead_reckoning.py`(IMU yaw + 모터명령/2.5 적분 오도메트리, `dead_reckoning.py:244` `self.speed = data.speed / 2.5`) → AMCL(파라미터를 얇은 고깔용으로 튜닝: `config/nav2_params.yaml` max_beams 60→180, particles 1000~4000, 주석 명시) → Nav2 → `ad_translator/motor_translator.py`(cmd_vel→XycarMotor, speed_scale 30 / angle_scale −150, `motor_translator.py:16-17,43`) → `goal_sender.py`가 FollowWaypoints 2점 전송, 완료 시 `pkill -f motor_translator`(`goal_sender.py:96-97`).
2. **이후 = 카메라 차선주행 + 신호등 중재 FSM**: `traffic_Light.py:809-836` `_wait_for_nav2_completion()`이 `/xycar_motor` 퍼블리셔 수가 1→0 되는 순간을 감지해 기동. `track_drive.py`는 `/lane_motor_cmd`로만 발행(`track_drive.py:1172`)하고, `traffic_Light.py`의 FSM(DRIVE / SIGNAL_WAIT / STOP / LEFT_TURN, `traffic_Light.py:21-24`)이 그것을 받아 최종 `/xycar_motor`를 발행(`traffic_Light.py:204-222`). **"차선주행 노드 위에 신호등 노드가 최종 중재자"**인 토픽 체인 구조.

### C. 미션별
- **신호등**: HSV red([0,170,120]~[10,255,255]+170~179 랩어라운드), green([38,170,120]~[102,255,255]) → **원형성 필터**(circularity≥0.45, area≥30, AR 0.55~1.8, `traffic_Light.py:557-564`) 통과 컨투어만 픽셀 카운트. 임계 red/green 각 50px, confirm 3프레임(`traffic_Light.py:43-45`). **빨강+초록 동시 = LEFT_TURN**(`traffic_Light.py:244-245`) — 4구 신호등의 "빨강+좌회전화살표" 조합을 색 2종으로 근사. 주석 `traffic_Light.py:698-699` "기존에는 필터링 전 색상 픽셀을 세어 **라바콘도 포함됐다**" → 원형성 필터 도입 이유가 명시된 실전 교훈.
- **출발**: 정지선 검출 → SIGNAL_WAIT(속도 0) → green confirm 시 DRIVE. `goal_sender.py:33,54-60`엔 green 30px 대기 로직이 있으나 `#임시!!` 주석으로 비활성(`goal_sender.py:48`).
- **정지선**: BEV white 마스크에 (41,3) 수평 커널 MORPH_OPEN → bbox 폭≥260px & 높이≥3px, 탐색창 y120~240/좌우 margin 70, 3프레임 확인(`track_drive.py:119-124, 283-358`). **rising-edge에서만 1회 정지**(`traffic_Light.py:186-194` "같은 정지선이 여러 프레임 유지돼도 한 번만 정지").
- **차선**: ROI y150~480 → HSV 이진화(white V≥254 — 시뮬 전용 극단값, `track_drive.py:214-215`; yellow [10,242,80]~[40,255,255]) → 사다리꼴 BEV(BOTTOM_DIST 893/TOP_DIST 190, `track_drive.py:38-47`) → 히스토그램 시작점(9탭 스무딩, peak 35% 후보 중 이전값 최근접, 160px 점프 시 0.65:0.35 블렌드, `track_drive.py:495-506`) → 슬라이딩윈도우 10개(margin 45, minpix 25) → 2차핏(위치)+1차핏(헤딩) 분리(`track_drive.py:582-585`).
- **제어**: Stanley 기본(`CONTROLLER='stanley'`, `track_drive.py:87`): θe = −slope×56, cross = 1.7×e/v, ±100 클램프(`track_drive.py:902-907`). PID 대안: KP 2.0 / KD 30.0 / KI 0(`track_drive.py:74-76`).
- **라바콘**: 카메라 코드에는 없음 — **Nav2가 지도 기반으로 통과**하는 설계(위 B). costmap robot_radius 0.5m/inflation 1.3m, controller max_vel_x 0.12(`config/nav2_params.yaml:193,242,247`).
- **추월**: `CameraOvertakeDecision`(`track_drive.py:1042-1117`) — 노란 방해차 HSV [22,128,128]~[41,193,255] ≥200px(`track_drive.py:936-938`) 5프레임 연속 → LANE2(차선 기준점을 **우측으로 150px 오프셋**), 5프레임 미감지 → WAITING, 1프레임 후 LANE1(−25px 좌측 오프셋 유지), 스쿨존이면 DISABLED. 즉 **조향을 직접 만들지 않고 차선주행의 기준점(lateral_offset)만 옮기는 방식** — 우리 target_passing의 offset 접근과 동일 사상. 라이다판(`LidarOvertakeDecision`, 제안값 override)은 주석 처리(`track_drive.py:1166-1170, 1305-1306`)이고 해당 노드 소스는 레포에 없음.
- **좌회전(지름길 성격)**: 신호 좌회전 시 1.5s 직진 접근(`signal_left_turn_approach_sec`, `traffic_Light.py:48`) 후 **최대 좌조향 −100, 속도 3.0**(`traffic_Light.py:46-47`), 차선(좌+우) 3프레임 재획득 시 DRIVE 복귀, 150프레임 타임아웃 시 STOP(`traffic_Light.py:377-430`). 신호가 3s간 안 보이면 무신호 좌회전 개시, 좌차선 획득 후에도 7s간 최대 좌조향 유지(`traffic_Light.py:49-50, 279-341`).
- **경찰차(시뮬 전용 요소)**: 좌측 ROI(x 0~0.75w, y 0.42~0.92h)에서 blue([90,120,80]~[135,255,255])≥12px AND red≥12px 3프레임 → 좌회전 보류(`traffic_Light.py:30-34, 485-544`).
- **스쿨존**: yellow px > white px × 2.0이 5프레임(해제 7프레임) → 노란 실선 추종으로 전환하되 **중앙 점선 영역(x 220~420)을 마스크로 제거**(`track_drive.py:99-105, 366-402`), 속도 15 캡(`track_drive.py:1307-1308`).
- **체크무늬(그리드) 구간**: 하단 1/3에서 V<20 다크픽셀≥3000 AND 흰≥1000 → 좌측 차선 탐색 강제 스킵(`track_drive.py:110-114, 410-450, 1277`).
- **랩 카운팅**: 없음.

### D. 제어 게인·속도
SPEED_MAX 19.5 / SPEED_MIN 5.0 / SPEED_KD 27(`track_drive.py:80-82`); 감속식 `t=max((|angle|/100)^0.8, |KD·d_err|/100)`, `speed=MAX−(MAX−MIN)·t`(`track_drive.py:805-808, 912-915`); 한쪽 차선도 없으면 MIN 고정. 좌회전 3.0/−100.

### E. 튜닝값 표 (발췌)
| 항목 | 값 | 위치 |
|---|---|---|
| BEV 사다리꼴 | BOTTOM 893 / TOP 190 / Y_OFF 50 | track_drive.py:38-40 |
| 슬라이딩윈도우 | n10, margin45, minpix25, top90, bottom60 | track_drive.py:62-67 |
| 차선 기준 x | R 452 / L 188 (±28 보정) | track_drive.py:90-91 |
| PID | KP2.0 KD30 KI0 | track_drive.py:74-76 |
| Stanley | heading 56, K 1.7, v_min 1.0 | track_drive.py:94-96 |
| 속도 | 19.5 / 5.0, SPEED_KD 27 | track_drive.py:80-82 |
| 정지선 | 폭≥260, y120~240, 3프레임 | track_drive.py:119-124 |
| 신호등 | red/green 50px, 3프레임, circ 0.45 | traffic_Light.py:43-45,559 |
| 좌회전 | angle −100, speed 3.0, 접근 1.5s, 타임아웃 150f | traffic_Light.py:46-56 |
| 추월(카메라) | HSV [22,128,128]~[41,193,255], 200px, 5f 진입/5f 소실, offset +150/−25 | track_drive.py:936-947 |
| 스쿨존 | yellow>white×2.0, 5/7f, 속도캡 15 | track_drive.py:101-103,1308 |
| Nav2 | max_vel_x 0.12, robot_radius 0.5, inflation 1.3 | config/nav2_params.yaml:193,242,247 |

### F. 노하우 / 함정
- (노하우) **"PID는 카메라 프레임이 아니라 루프 주기로 돌려라"** — 새 프레임만 처리하면 큰 KD가 프레임 간 노이즈에 반응해 조향이 흔들리므로, 판정(정지선/스쿨존)만 새 프레임 단위로 제한(`track_drive.py:1261-1276` 주석+구현). 30fps 카메라 + 고KD 조합에서 실질적 팁.
- (노하우) 신호등 후보에 원형성+AR 필터 — 라바콘 오검 제거의 직접 경험담(`traffic_Light.py:698-699`).
- (함정) `count_publishers()` 감시 + `pkill`로 단계 전환(`traffic_Light.py:830-834`, `goal_sender.py:96`) — 노드 크래시/재시작 한 번에 전체 시퀀스가 깨지는 취약 설계. 본인들도 방법 2(타임아웃 기반)를 주석으로 남김(`goal_sender.py:100-106`).
- (함정) dead reckoning 오차를 `dist/2` 임시 나눗셈으로 덮음(`dead_reckoning.py:60-62` "오차가 엄청 큼… ㅠㅠ").

### G. 우리 대비
- **가져올 것**: ① 정지선 검출기(수평 모폴로지 41×3 + 폭 임계 + rising-edge 1회 정지) — 우리 스택에 정지선 전용 검출이 없다면 이식 가치 높음. ② 신호등 원형성 필터 아이디어(우리 HoughCircles와 상호 보완 — Hough 실패 시 컨투어 fallback). ③ 좌회전 "차선 소실→재획득 n프레임" 종료 조건 + 프레임 타임아웃의 2중 안전장치 — 우리 지름길 1회 좌회전 종료 판정에 그대로 참고 가능. ④ 추월을 lateral_offset으로 처리하는 구조 검증(우리와 동일 접근을 독립적으로 채택했음 = 설계 타당성 방증).
- **반면교사**: Nav2/SLAM으로 라바콘을 푸는 접근은 오도메트리 부실로 본인들도 실패 중 — 실차 3바퀴·10Hz 라이다 환경에서 재현 비추천. white V≥254 같은 시뮬 전용 임계는 실차 이식 불가.

### H. 신뢰도: **중**
코드 자체는 6개 중 가장 정교(히스테리시스·디바운스·엣지트리거 일관). 그러나 시뮬 전용 + README의 미완 자인 + 핵심 소스 2개 누락 + 실주행 증거 전무.

---

## 2. jeonghun — jeonghun43/xytron_convention (추월 원본)

### A. 메타
- ROS2 Python, **Unity 시뮬 지향**(YOLO COCO 신호등, 스쿨존 노면글씨, 경찰차 클래스 언급 `avoid_person.py:28`). 실차 증거 없음.
- 마지막 커밋 `42a25ff 2026-06-23 "Merge pull request #3 from jeonghun43/jeong"` — PR 워크플로 존재. 기본 브랜치명 `default`. `__pycache__/*.cpython-310.pyc`가 커밋되어 있어 **실행(시뮬) 흔적은 확실**.
- 성격: 모듈별 실험 모음. `setup.py:27-37`에 11개 엔트리(track_drive, traffic_light, cone_drive, with_yolo, yolo_light, child_zone, **overtake**, center_line, lane, line, rabar).

### B. 아키텍처 / 상태기계
- `track_drive.py`가 마스터: **multiprocessing으로 신호등 YOLO를 별도 OS 프로세스로 분리**(`track_drive.py:266-283`), 주행 노드는 MultiThreadedExecutor 4스레드. 상태: `traffic_status`(NONE/STOP/YELLOW/GO) × `drive_status`(NONE/RABACON/NORMAL). 차선(line)·스쿨존(child_zone_v2)은 모듈 객체로 내장, **추월(overtaking)은 미통합 독립 노드**.

### C. 미션별
- **신호등**: YOLOv8n(사전학습) class 9 박스 검출 → 박스 crop HSV 픽셀 카운트: red≥2000 STOP / green≥2000 GO / yellow≥2000 YELLOW(`with_yolo_traffic_light.py:109-114`). **AR≥2.9면 가로 4분할**해 3번째 칸=좌회전 green, 4번째 칸=직진 green을 따로 셈(`with_yolo_traffic_light.py:92-105`) — 그러나 `left_pixels`는 계산만 하고 **상태 판정에 사용하지 않음** → 좌회전 신호 미구현. (구형 `traffic_light.py`는 고정 ROI [90:200,180:390] 픽셀 카운트 버전.)
- **출발**: GO 최초 수신 시각 기록, **1.5초간 차선인식을 우회하고 speed 10 강제 직진**(`track_drive.py:96-99` "교차로 통과 중"). 신호등 지주/정지선 오검 회피 목적 — pumky의 1.2s 직진과 동일 발상.
- **차선(line.py)**: ROI 상단 30% 컷 → white HSV(V 230~255, S≤5) → HoughLinesP(50,30,40) → |slope|>0.2 → 최하단점 x와 slope 부호로 좌/우 분류(`line.py:71-100`) → 좌·우 중 "메인 라인" 1개 선택(최하단점 비교, `line.py:120-124`) → 1차 폴리핏 후 **x(y=200)−520을 그대로 각도로**(P=1, `line.py:149-156`). 차선 0.2s 소실 시 speed 1로 급감(`line.py:61-67`), 픽셀 부족 시 직전 조향 방향으로 ±100 풀조향 + 20프레임 유지(`line.py:137-145`).
- **라바콘**: 진입 판정이 마스터에 내장 — 좌(20~80°) 평균>1.2m & 점≥2 AND 우(−80~−20°) 동일 AND 전방(±15°) 무검출 → RABACON(`track_drive.py:188-191`). 주행: `angle = 55×(L−R) − 30`(**−30 우편향 바이어스**), 좌만 보임 +55 / 우만 −85 / 무검출 −20 spd8, 전방<0.6m spd8(`track_drive.py:208-232`). 이탈: 좌우 모두 0점 3스캔 연속(`track_drive.py:196-206`). 자차 프레임 반사 노이즈 0.14~0.17m 링 제거(`track_drive.py:156-157`).
- **추월(overtaking.py — 우리 target_passing의 원본)**: 하단 상세 비교 참조.
- **스쿨존**: v2(`child_zone_v2.py`) — ROI[200:400,100:500] → BEV 400×400 → yellow [22,180,200]~[32,255,255] → **MORPH_TOPHAT(20×20)으로 굵은 차선을 빼고 글자 획만 남김**(`child_zone_v2.py:52-55`), 6000px 초과 시 SCHOOL_ZONE↔NORMAL 토글(쿨다운 30프레임). SCHOOL_ZONE 중엔 line_mask(OPEN 결과)에서 중앙(x130~270, y80~120) 노란 점선 무게중심 추종, gain 1.0, speed 5(`child_zone_v2.py:69-104`). v1(`child_zone.py`)은 solidity 기반 글자/차선 판별 4중 가드(h>80 세로차선 제외, w>200&sol>0.7 가로실선 제외, sol<0.28 사선점선 제외, area>3000&sol>0.85 블록 제외, `child_zone.py:82-99`).
- **지름길 / 랩 카운팅 / 고정장애물**: 없음.

### C-보강. overtaking.py 정밀 분석 & 우리 target_passing 비교
**동작(현행 버전, `overtaking.py:8-165`)**
- FSM: NORMAL → OVERTAKE_PHASE_1 → OVERTAKE_PHASE_2 → NORMAL.
- 트리거(`overtaking.py:70-82`): 전방 ±15°(−0.26~0.26rad) min < **1.5m** AND 좌측 사각 45~90°(0.78~1.57rad) min > **2.0m**. 좌측 막히면 **speed 0 긴급 정지**.
- PHASE_1(`overtaking.py:84-122`): 추적 섹터 **−60°~+15°**(자차가 좌로 비끼면 앞차가 우측 후방으로 이동한다는 기하 반영)에서 **최근접 1점**을 앞차로 간주 → (obs_x, obs_y) 변환 → `obs_x < −0.2` 이면 통과 완료. 아니면 **동적 목표점 = (obs_x+1.2, obs_y+0.7)** — 앞차보다 1.2m 앞, 0.7m 왼쪽. 퓨어퍼수트 축약식 `steering_rad = 2·L·target_y / Ld²`(L=0.3, `overtaking.py:110-111`) → deg → 부호 반전 → ±50 클램프, speed 15.
- PHASE_2(`overtaking.py:124-145`): **가상 복귀점 (2.0, −0.6)** 을 같은 PP식으로 45 스캔프레임(action_counter) 추종, speed 10 → NORMAL 복귀.
- NORMAL일 땐 angle 0 직진 발행(`overtaking.py:80-82`) — 차선주행 미결합 단독 테스트 노드.
- 파일 하단(`overtaking.py:167-284`)에 구버전(타이머 bang-bang: −50° 45카운트 → 직진 60카운트 → −35° 복귀)이 주석으로 보존 — **하드코딩 조향 → 동적 목표점 PP로 진화한 이력** 자체가 문서.

**우리 target_passing.py 대비 (kmu_race/core/target_passing.py)**
| 축 | jeonghun overtaking | 우리 target_passing |
|---|---|---|
| 프레임워크 | 라이다 좌표계에서 조향 직접 산출, 차선 문맥 없음 | 차선주행의 lateral_offset_px에 얹힘(`update()` 반환 `lateral_offset_px`) — 실선 침범을 차선 기준으로 관리 가능 |
| 방향 선택 | **항상 좌측 고정**(+0.7m) | 타겟 y 부호(deadzone 0.14m)→빈쪽→차선편 3순위(`choose_side`, target_passing.py:98-138) — 규정 "주행하지 않는 차선" 명시 대응 |
| 방해차 차선 변경(끼어들기) | 미대응 (최근접점 추적이 사실상 유사 대응이나 방향 전환 없음) | moving=True 시 `_target_cuts_in` 3프레임로 반대편 재선택(target_passing.py:206-219) |
| 통과 판정 | obs_x < −0.2 즉시 | target_x > −0.1 미충족 5프레임(clear_frames_to_return) 히스테리시스 |
| 복귀 | 고정 가상점 45 스캔프레임 — **시간 고정** | 90cm 데드라인 기반 urgency 가속(target_passing.py:239-251) — 규정 p.32 직접 반영 |
| 추돌 방지 | 없음(트리거 전 1.5m뿐) | min_gap 0.55m 이내면 속도 30%로 캡(target_passing.py:259-265) |
| 교착 처리 | 좌측 막히면 **완전 정지** | blocked 시 감속 유지 + 6s 타임아웃 후 차선주행 복귀 |
| 시간축 | scan 콜백 카운트(45) — 시뮬 30Hz 가정 ≈1.5s, **실차 10Hz면 4.5s로 뻥튀기** | 벽시계 초 기반 |

**jeonghun에서 실제로 가져올 것**: ① "앞차 기준 상대 목표점(obs + 종/횡 고정 오프셋)을 PP로 추종"하는 발상 — 우리 SHIFT/ALONGSIDE는 차선 offset 수렴 방식이라, 라이다 타겟이 살아있을 때 목표점을 타겟 상대좌표로 잡으면 방해차 속도 변화에 더 강해질 수 있음(ALONGSIDE 중 보조 항으로 검토). ② 추적 섹터를 −60°~+15°로 **비대칭** 잡은 점 — 좌측 추월 중 타겟이 우후방으로 흐르는 기하를 반영한 것으로, 우리 lidar_tracker의 추월 중 타겟 유지 각도 설계에 참고. ③ obs_x<−0.2 같은 "타겟 x가 음수로 넘어감 = 옆을 지남" 판정의 단순성(우리 clear 판정과 동치임을 교차 확인).
**반면교사**: 좌측 고정 방향(방해차가 1차선에 있으면 규정 위반), 시간 고정 복귀(90cm 규정 무시), 완전 정지 선택, 최근접 1점=차량 가정(콘/벽 오인), 스캔주기 의존 카운터.

### D. 제어 게인·속도
차선 base 12(소실 시 1), 콘 kp 55 / spd 10→8, 추월 base 10 / pass 15 / 클램프 ±50, wheelbase 0.3, 스쿨존 5, GO 직진 10.

### E. 튜닝값 표 (발췌)
| 항목 | 값 | 위치 |
|---|---|---|
| 추월 트리거 | front<1.5m(±15°), left(45~90°)>2.0m | overtaking.py:71-73 |
| 추월 목표점 | obs+(1.2, +0.7)m / 복귀 (2.0, −0.6)m | overtaking.py:105-106,129-130 |
| 통과 판정 / 복귀시간 | obs_x<−0.2m / 45 scan | overtaking.py:97,100 |
| 추월 속도/클램프 | 10→15 / ±50° | overtaking.py:23-24,116 |
| 콘 조향 | 55×(L−R)−30, 단측 +55/−85 | track_drive.py:213-224 |
| 콘 진입/이탈 | L,R>1.2m&≥2pt&front무 / 무검출 3스캔 | track_drive.py:188-206 |
| 차체 반사 제거 | 0.14~0.17m 링 무시 | track_drive.py:156 |
| 차선 P | diff = x(y=200)−520, clip ±100 | line.py:151-156 |
| 차선 화이트 | V 230~255, S≤5 | line.py:46-48 |
| 신호등(YOLO) | conf 0.3, cls 9, 색 2000px, AR≥2.9 4분할 | with_yolo_traffic_light.py:39,95,109-114 |
| 스쿨존 | TOPHAT 20×20, 6000px 토글, 쿨다운 30f | child_zone_v2.py:52-66 |

### F. 노하우 / 함정
- (노하우) YOLO 신호등을 **별도 OS 프로세스**로 분리해 주행 루프 지연 차단(`track_drive.py:266-283`). 우리처럼 HoughCircles가 가벼우면 불필요하지만, 무거운 인지 추가 시 참고.
- (노하우) TOPHAT으로 "노면 글자 vs 차선" 분리 — 형태학 한 방으로 끝냄.
- (함정) `rabacon.py:10-13` `RabaconDrive`가 Node 미상속인데 `self.create_publisher` 호출 → 단독 실행 시 즉사(마스터에서는 미사용이라 잠복). `with_yolo_traffic_light.py:157` `destory_node` 오타, `overtaking.py:11` 인코딩 깨짐("자วน주행") 등 완성도 편차 큼.
- (함정) 좌회전 신호 픽셀을 세고도 상태에 안 쓰는 미완(4구 신호등 좌회전 미션 대응 불가).

### G. 우리 대비
가져올 것은 위 C-보강 3건. 추가로 GO 후 1.5s 강제 직진(출발 직후 정지선/지주 오검 차단)은 우리 START 상태 탈출 직후에 간단히 적용 가능한 보험. 반면교사는 "모듈은 많되 통합·규정 반영이 안 된 상태로 대회를 맞는 패턴" 그 자체.

### H. 신뢰도: **하~중**
PR/실행 흔적은 있으나 추월·좌회전·랩 등 본선 핵심이 미통합/미완이고 잠복 크래시 코드가 섞여 있음. 추월 아이디어의 "원본" 가치는 유효하되 수치는 시뮬 기준으로만 신뢰.

---

## 3. pumky — pumkyQ/xycar_Koomin

### A. 메타
- ROS2 + **Unity ROS-TCP-Endpoint**(런치가 endpoint를 직접 include, `track_drive/launch/track_drive.launch.py:10-19`). 로그 문구 "국민대 대회 **예선과제 1번**"(`track_drive.py:38`, `track_drive_cone.py:39`) → 예선 시뮬 대상 확정.
- 커밋: `2b5bc89 2026-06-26 "10th commit 곡선 차선이탈 수정"` → 최소 10회 커밋, 코드 전반에 "사용자 요청에 따라 조정/복구" 류 튜닝 주석 다수 → **반복 튜닝 활동 활발**. 실차 증거 없음. `kookmin9_viewer`(키보드 수동조작 뷰어), `bev_viewer`, `steer_viewer` 등 자작 도구 세트 보유.

### B. 아키텍처 / 상태기계
3개 메인 노드(발전 단계별):
- `drive.py`: WAIT_FOR_GREEN → **GO_STRAIGHT 1.2s**(신호등 지주 오인 방지, `drive.py:126,213-223`) → CONE_DRIVING(미검출 30프레임≈0.6s 시 종료) → LANE_DRIVING.
- `track_drive.py`: WAIT_FOR_GREEN / LANE_DRIVING / CONE(비활성) / PEDESTRIAN_STOP / FINISHED. `_get_next_state()`로 전이 로직 단일화(`track_drive.py:233-279`), 50Hz 루프.
- `track_drive_cone.py`: WAIT → CONE(친구의 ConeDriver) → **아스팔트 감지(전방 ROI[350:480,200:440] 검은 비율>50% + 3s 쿨다운)** 시 LANE 전환(`track_drive_cone.py:145-168, 221-229`).

### C. 미션별
- **신호등**: HSV(green [40,80,150]~[90,255,255] 등) → 컨투어 **최대 원형 면적**(area>15, AR 0.2~5.0) 비교 + 상태별 카운터 증감, confirm 3(`traffic_light_detector.py:96-123`). LEFT_ARROW는 "green 컨투어 AR>1.2"로 판정(`traffic_light_detector.py:125-137`) — 조잡하며 FSM에서 미사용.
- **차선**: BEV(src 상단 0.15/0.85 확장 — "코너링 시 우측 흰선 화면 밖 이탈 방지" 주석, `lane_detector.py:39-45`) → **노랑=좌 / 흰=우 색상 분리 추적**, 잔디(green H40~90) 마스크를 팽창 후 도로영역 AND(`lane_detector.py:91-102`) → 슬라이딩윈도우 10개(margin 75): **윈도우별 차선폭 학습 배열**(바닥 180px→꼭대기 80px 초기, 측정폭 오차<40px일 때만 양쪽 인정 + EMA 0.95/0.05 갱신; 한쪽만 보이면 학습폭으로 반대편 재구성; 둘 다 없으면 직전 gradient ±15px 연장)(`lane_detector.py:71, 307-376`). 이전 프레임 위치 ±150px(커브 ±300px) 윈도우 탐색 + 실패 시 전역 재탐색 리셋(`lane_detector.py:237-276`).
- **제어**: Pure Pursuit `delta=atan2(2·L·e_y, Ld²)`, L=0.33, e_y=px×(1/384)m, Ld=clip(0.8+0.10·speed, 0.5~1.5), steer_gain 4.0(`track_drive.py:57-68, 127-139, 354`). **곡률 = |mid−(top+bottom)/2|**(윈도우 0-2/3-5/6-8 평균, `lane_detector.py:137-143`) → EMA α0.25 → **이중 임계 히스테리시스(진입 25/탈출 16) + 탈출 0.8s 지연** → 급커브면 **PID/필터 바이패스하고 ±100 즉시 풀조향 + speed 4 고정**(`track_drive.py:371-441`); 평시엔 Slew Limiter(25/frame) + EMA(α0.6)(`track_drive.py:445-457`, "Jejudol_ws 벤치마킹" 주석). 속도 `10−ratio^1.5×(10−4)`(`track_drive.py:141-150`).
- **라바콘**: 결정판은 `lidar_drive.ConeDriver`(target_speed 20, kp 70) — 인접점 0.4m 클러스터링 → **좌/우 콘 차선을 nearest-neighbor 체인으로 연결**(반대편 침범 X±0.5m 금지, 스텝 제약 dx −2.5~0.5 / dy 0.1~3.0) → 공유 콘 crossover 소유권 판정(연결거리 짧은 쪽) → 한쪽 콘 1개면 반대편 기울기 복사 연장 → piecewise 선형 보간으로 lookahead(max(3.5, 0.35×speed)m, 콘이 짧으면 당김) 지점의 중앙 target_x → `angle=−(target_x×70)`, LPF 0.6/0.4, 감속 |a|>50→0.6× / >20→0.8×(`lidar_drive.py:54-275`). 간이판은 `obstacle_detector.detect_cones`(최근접 3점 평균차 ×60, 단측 ±45).
- **보행자(고정장애물 판별 대용)**: 카메라 어두운 세로 blob(AR>1.5, h>40) AND 라이다 전방<2.5m → PEDESTRIAN_STOP, 소실 후 3s 대기 복귀(`obstacle_detector.py:166-218`, `track_drive.py:254-276`).
- **추월**: `detect_vehicle`(전방 4m 내 min_angle 부호로 좌/우 추월 방향 결정, `obstacle_detector.py:220-248`)가 있으나 **FSM에 미연결** — 미구현.
- **랩 카운팅**: **신호등 재검출 + 25s 쿨다운**으로 lap++(`track_drive.py:285-298`). 완주 정지 전이는 연습용으로 비활성(`track_drive.py:251` 주석).
- **스쿨존**: `road_sign_detector.py` 노면 노란 글씨 면적·폭 휴리스틱(SCHOOL↔RELEASE, 속도 0.4배) — 메인 FSM 미연결.
- **지름길/정지선**: 없음.

### D. 제어 게인·속도
PP steer_gain 4.0 / L 0.33 / Ld 0.5~1.5; LANE 10↔4(급커브 4 고정, ±100 강제); CONE 20(자체 감속 0.6/0.8×), kp 70; drive.py 구판: 콘 8, 차선 10, PID(0.5, 0.001, 0.3).

### E. 튜닝값 표 (발췌)
| 항목 | 값 | 위치 |
|---|---|---|
| BEV src | (0.15,0.55)(0.85,0.55)(0.05,0.95)(0.95,0.95) | lane_detector.py:40-45 |
| px→m | 1/384 (차선폭 1.0m 가정) | lane_detector.py:58-60 |
| 윈도우 차선폭 학습 | 180→80px 초기, 오차<40px, EMA .95/.05 | lane_detector.py:71,339-343 |
| 곡률 히스테리시스 | 진입25 / 탈출16 / 지연0.8s / EMA .25 | track_drive.py:61-66 |
| 조향 필터 | Slew 25/f + EMA .6 | track_drive.py:447-453 |
| Lookahead | 0.8+0.10·v, clip 0.5~1.5 | track_drive.py:354 |
| 속도 | 10 → 4 (ratio^1.5) | track_drive.py:71-72,141-150 |
| 콘 클러스터 | 인접 0.4m, 체인 dx −2.5~0.5 / dy 0.1~3.0 | lidar_drive.py:63,127 |
| 콘 lookahead/트랙폭 | max(3.5, 0.35v)m / 반폭 2.5m | lidar_drive.py:191-192 |
| 콘 게인/속도 | kp70, LPF .6/.4, v20 (0.6/0.8 감속) | track_drive_cone.py:51, lidar_drive.py:259-273 |
| 랩 카운트 | 신호등 재검출 + 25s 쿨다운 | track_drive.py:293-298 |
| 아스팔트 전환 | ROI[350:480,200:440] 흑비율>50% +3s | track_drive_cone.py:154-168,224 |
| 출발 직진 | 1.2s @8 | drive.py:126,219-220 |
| 보행자 | AR>1.5, h>40, 라이다<2.5m, 복귀 3s | obstacle_detector.py:199, track_drive.py:78 |

### F. 노하우 / 함정
- (노하우) **급커브에서는 스무딩 필터가 적**: 평시 Slew+EMA로 털림을 잡되, 곡률 히스테리시스로 확정된 급커브에선 필터를 바이패스하고 즉시 풀조향 — "부드러움"과 "반응성"을 상태로 나눠 해결한 실전형 패턴.
- (노하우) 콘 주행을 "점 회피"가 아니라 **"콘 차선 2줄 재구성 → 중앙선 생성 → lookahead 추종"** 문제로 승격 — 콘이 끊긴 구간은 반대편 곡률 복사로 이어붙임.
- (함정) 랩 카운트가 "신호등이 다시 보이면 +1"이라 신호등 유사 색 오브젝트/재접근에 취약, 25s 고정 쿨다운은 랩 타임 가변성에 깨질 수 있음.
- (함정) 본선 미션(추월·좌회전·정지선) 부재 — 예선 1번 과제에 최적화된 스코프.

### G. 우리 대비
- **가져올 것**: ① **윈도우별 차선폭 학습 + 단측 재구성** — 우리 BEV 차선이 고정폭 가정이라면 이 적응형 폭 배열은 곡선·원근 변화에서 단측 추정 정확도를 올려줌(추월 중 한쪽 차선만 보일 때 특히 유효). ② 급커브 필터 바이패스 패턴 — 우리 PID 출력단 스무딩이 있다면 동일한 상태 분기 검토. ③ 콘 nearest-neighbor 체인/crossover 해소 — 우리 라이다 클러스터 기반 콘 미들라인에 제약식(dx/dy 범위, 침범 금지)만 이식해도 오연결 감소. ④ 곡률 지표 |mid−(top+bottom)/2| — 폴리핏 없이 싸게 커브를 감지하는 지표로 속도정책 입력에 쓸 만함.
- **반면교사**: 신호등 재검출 랩카운트 — 우리 겐트리 목격 방식이 명백히 우월함을 확인해주는 대조군.

### H. 신뢰도: **중**
활발한 튜닝 이력과 높은 코드 품질(전이/제어 분리, 예외 처리)이 장점. 다만 전부 시뮬 수치이고 본선 미션 커버리지가 낮음.

---

## 4. mango — mango1228/xycar-team (develop)

### A. 메타
- **ROS1 Melodic / Python2 / 실차 자이카** — 유일한 확실 실차 레포. 근거: `README.md:4-5`(Ubuntu 18.04 Melodic), `src/my_motor/launch/xycar_drive.launch:15-24`(usb_cam `/dev/videoCAM`, autoexposure off + exposure 110), xycar_motor·xycar_lidar 실드라이버 include, `README.md:61` ROS_MASTER_URI 10.42.0.1(자이카 AP), DISPLAY 부재 가드(`main.py:28-30`), "연습주행 오실레이션 제어" 류 주석.
- 마지막 커밋 `99e10d3 2026-06-04 "판정 ROI y 310~420으로 축소 + classify_white_pct 7%"` — **실측 후 파라미터를 커밋 메시지로 남기는 팀**. README에 브랜치 전략/리뷰 규칙, CONTRIBUTING.md 존재.
- 단, 미션 세트가 본선과 다름: **횡단보도 10초 정지 + 빗금구역 전진 후 영구정지**(과제/중간 이벤트 트랙으로 추정). 신호등·콘·추월 없음.

### B. 아키텍처 / 상태기계
- `main.py` XycarController: STATE_DRIVE / CROSSWALK_STOP(10s) / HATCH_VERIFY(구형, 미사용) / HATCH_ADVANCE(1.5s) / HATCH_STOP(영구)(`main.py:15-19`). 30Hz 주행 루프.
- **검출(15Hz `special_zone_detector.py` 별도 프로세스) ↔ 주행(30Hz) 분리** — "무거운 CV가 주행 루프를 막지 않음"(`special_zone_detector.py:5-9`). 검출은 비트마스크 Int8 `/special_zone`으로 전달, 디바운스/상태머신은 주행 노드 소유.

### C. 미션별
- **차선**: Canny(40,100) → ROI(offset 330, gap 110) → HoughLinesP(30,20,10) → slope 0.1~10 + center_margin 90으로 좌/우 분류 → **좌·우 각각 "무게중심이 가장 안쪽인 선분 1개"만 채택해 옆 차선/노이즈 배제**(`lane_drive.py:81-86`) → lpos/rpos 중앙. **반폭 EMA(α0.3)** 기억으로 한쪽만 보일 때 중앙 재구성(one_lane_ratio 1.0, `lane_drive.py:99-114`). 좌코너 판단: 좌차선 slope가 기준 −0.75 대비 −0.4 이상 가팔라지면 corner LEFT → 추종점 70px 좌측 이동(`lane_drive.py:90-97,119-121`).
- **제어**: dt 기반 PID(gain 0.25 / I 0.01, clamp ±300 / D 0.01), 조향 ±50, speed 5(`config.py:26-30`, `main.py:70-90`). **첫 프레임 D항 스킵**(기준 없음 → 조향 튐 방지, `main.py:76-79`), **정지/재출발 시 reset_pid()**(`main.py:92-96`).
- **라이다 융합**: 전방 ROI 박스(x ±0.33m, y −0.45~−0.05m; laser_frame 전방=−y) 내 포인트의 각도 **빈틈(gap) 최대 구간 이등분선** → 편차 ×(−170) → 픽셀 center 후보 → **차선 lpos/rpos 안으로 클램프** 후 카메라 center보다 편차가 클 때만 채택("+LIDAR" 모드)(`main.py:113-137`). RViz MarkerArray 시각화 완비(`lane_drive.py:213-287`).
- **특수구역 판별**(본선 무관하나 기법은 범용): 어두운 노면 내부(CLOSE 31×31 + ERODE 15×15)의 흰 비율 ≥7% → 내부 흰 무늬 선분 **길이가중 평균각**(수평 10° 미만 제외) ≥35°면 횡단보도 / 미만이면 빗금(`special_zone.py:24-79`). 상태머신: 트리거 카운터는 **DRIVE 상태에서만 갱신**(정지 중 누적→재출발 즉시 재트리거 버그 방지, `main.py:157-159`), startup 유예 3s + 재출발 후 유예 15s + miss tolerance 3(`config.py:100-103,97`).
- **장애물(obstacle_lane_drive.py, 통합 실험)**: 우선순위 AVOID > TUNNEL > LANE. AVOID: ROI 내 ≥2점 → 오프셋 `80×(1+dist_factor+x_factor+coverage)` 최대 320px를 장애물 반대편으로(`obstacle_lane_drive.py:288-324`). TUNNEL: 양측 <0.9m → `(L−R)×30` 벽 균등 추종(`obstacle_lane_drive.py:327-345`).
- 신호등/콘/추월/지름길/랩: 없음. `shortcut.py`는 이름과 달리 라이다 코리도 주행 실험이며 **SyntaxError로 실행 불가**(`shortcut.py:119-121` 빈 함수 몸체).

### D. 제어 게인·속도
PID 0.25/0.01/0.01(±50, I클램프 300), speed 5 고정(저속 안정 지향), lidar_center_gain −170, TUNNEL 30, AVOID 80~320px.

### E. 튜닝값 표 (발췌, 전부 launch 파라미터화됨 — `launch/xycar_drive.launch`)
| 항목 | 값 | 위치 |
|---|---|---|
| 카메라 노출 | exposure 110 고정 (autoexposure off) | launch/xycar_drive.launch:4,17-18 |
| Canny/ROI | 40/100, offset 330, gap 110 | config.py:21-24 |
| PID | 0.25 / 0.01 / 0.01, i_clamp 300 | config.py:26-28,51 |
| 속도 | 5.0 | config.py:30 |
| 반폭 EMA / 단측비율 | 0.3 / 1.0 | config.py:31-32 |
| 코너 | base −0.75, thresh 0.4, shift 70px | config.py:33-35 |
| 라이다 ROI | x±0.33, y −0.45~−0.05, gain −170 | config.py:11-19 |
| 특수구역 ROI | [310:420, 150:490] | config.py:54-57 |
| 판별 | white_pct 7%, angle 35°, min_angle 10° | config.py:82-85 |
| 정지 | 횡단보도 10s, 빗금 전진 1.5s | config.py:69,92 |
| 유예 | startup 3s, resume 15s, miss tol 3 | config.py:97-103 |

### F. 노하우 / 함정 (실차 노하우 최다)
- **모든 튜닝값의 launch 파라미터화 + 노출 고정** — 대회장 조명 대응의 정석. 우리 params.yaml 운영과 동일 사상이며, exposure를 launch 인자로 뺀 점이 실용적.
- **검출 프로세스 분리**(15Hz) — 파이썬 GIL 환경에서 주행 30Hz 사수.
- **3중 오탐 방어**: startup 유예 / 성공 후 resume 유예 / miss tolerance — "같은 표식을 두 번 밟는" 재트리거 계열 버그의 체계적 봉쇄. 트리거 카운터를 DRIVE 상태에서만 갱신하는 디테일 포함.
- **PID 위생**: 첫 프레임 D 스킵, 정지 후 reset — 재출발 조향 스파이크 방지.
- (함정) speed 5 저속 전제라 게인 수치를 우리 속도 대역에 직수입하면 안 됨.

### G. 우리 대비
- **가져올 것**: ① 정지→재출발 시 PID 리셋 + 재출발 유예 패턴을 우리 FSM(신호 대기→출발, 추월 종료→차선 복귀)에 명시 적용. ② 라이다 gap-이등분선 center를 **차선 경계로 클램프**하는 융합(고정장애물 회피 시 실선 침범 방지 가드로 응용 가치). ③ "가장 안쪽 선분 1개만 채택" — Hough 계열 fallback 차선 인식을 쓸 경우 옆 차선 오인 대책.
- **반면교사**: 본선 미션 부재 — 이 레포에서 가져올 것은 미션 로직이 아니라 실차 운영 위생뿐.

### H. 신뢰도: **실차 신뢰성 상 / 본선 관련성 하**
6개 중 유일하게 실차 launch·드라이버·노출 튜닝·실측 커밋이 완비. 단 미션 구성이 본선(2026-08-25)과 달라 미션 수치의 직접 이식은 불가.

---

## 5. gim-track — gim060569-hub/xycar-track-drive

### A. 메타
- ROS2(XycarMotor float API). 단일 커밋 `22a1428 2026-07-20 "First commit: V25 Virtual Gate Algorithm"` — 6개 중 **가장 최신**(본선 D-36). "국민대 대회준비" 명시. `__pycache__` 커밋(cpython-3.10) → 실행 흔적 있음(시뮬/실차 불명; 토픽은 자이트론 표준).
- 파일 2개: `track_drive.py`는 자이트론 템플릿 껍데기(메인 루프가 1.5s 정지/1.5s 전진 반복 데모, `track_drive.py:78-86`). 실질 코드는 `vision_drive.py` 하나.

### B. 아키텍처
FSM 없음. 카메라 콜백 하나에서 Part1 신호등 → Part2 콘 순차 처리. 라이다 구독 없음(track_drive.py 쪽에만 미사용 구독 존재).

### C. 미션별
- **신호등/출발**: 상단 1/3 ROI, red(0~10 + 160~180)+yellow(15~35) 합산 >300px & >green이면 정지, green(40~90)>300px이면 출발 + **15초간 신호 무시(FREEPASS)**(`vision_drive.py:42-58`) — 주행 중 신호 재검출로 멈추는 것을 시간 창으로 봉쇄. 원시적이지만 "출발 후 신호 무시 구간" 개념을 명확히 구현.
- **라바콘(비전 전용)**: 하단 1/2, 콘 HSV [5,150,100]~[25,255,255] 컨투어(area>80) 중 위험선(y>ROI 30%) 아래만 사용, 좌/우 각 최하단(최근접) 1개 선택 → 둘 다 보이면 실제 중점, **한쪽만 보이면 track_width 320px 떨어진 곳에 가상 콘을 생성해 대칭 게이트 중점**을 만든다(Virtual Symmetrical Gate, `vision_drive.py:96-129`). P게인 0.45, |조향|>15면 1.4배 커브 부스터, 클램프 ±50(`vision_drive.py:139-148`). 속도 6 고정.
- 차선/추월/장애물/지름길/랩: 없음.

### D. 제어 게인·속도
P 0.45 / booster 1.4 / 클램프 ±50 / track_width 320px / 신호 임계 300px / speed 6.

### E. 튜닝값 표
| 항목 | 값 | 위치 |
|---|---|---|
| 신호 임계/프리패스 | 300px / 15s | vision_drive.py:40,56 |
| 콘 HSV | [5,150,100]~[25,255,255] | vision_drive.py:65 |
| 콘 면적/위험선 | >80 / y>30% | vision_drive.py:79,72 |
| 가상 게이트 폭 | 320px | vision_drive.py:97 |
| P/부스터/클램프 | 0.45 / 1.4(>15°) / ±50 | vision_drive.py:140-148 |

### F. 노하우 / 함정
- (노하우) 한쪽 콘만 보일 때 **가상 반대콘으로 게이트를 복원**하는 발상 — pumky의 "반대편 곡률 복사", 우리 콘 미들라인 fallback과 같은 계열의 비전판. 급커브에서 안쪽 콘만 보이는 상황의 표준 해법임을 3개 레포가 독립적으로 수렴.
- (함정) 콘을 색으로만 검출(조명·노출 민감) + 라이다 미사용 — 실차 대회장 역광에서 위험. FREEPASS 15s는 랩 시간과 무관한 고정값이라 3바퀴 규정에 부적합.

### G. 우리 대비
가져올 것: "출발 직후 신호 무시 시간창" 개념의 최소 구현 확인(우리 겐트리 쿨다운 설계의 대조군), 가상 게이트 폭 320px(640px 화면 기준 반폭) 수치. 반면교사: 비전 단독 콘 검출.

### H. 신뢰도: **하**
단일 커밋·단일 파일·미션 커버리지 최소. 다만 날짜가 본선에 가장 가까워 "타팀 준비 수준의 하한선" 참고치로는 의미 있음.

---

## 6. ktk — ktk0208/xycar_ros2

### A. 메타
- ROS2 패키지(자이트론 템플릿 헤더 그대로, `track_drive/track_drive.py:3-7`). 커밋 `d793d4c 2026-06-19 "Fix package structure"`(로컬 1건). 시뮬/실차 불명. **콘 미션 단일 목적**.

### B. 아키텍처
`TrackDriverNode.main_loop()`가 매 루프 `TrafficCone.get_control(image, lidar)` 한 번 호출(`track_drive.py:86-98`). FSM 없음 — 항상 콘 모드.

### C. 미션별
- **라바콘(카메라+라이다 가산 융합)**: 카메라 — 하단 1/2 HSV [5,100,100]~[25,255,255] 컨투어 area>300 상위 2개 무게중심 → 중점 오차 ×0.20 (한쪽만 보이면 좌측→+20 / 우측→−20 고정)(`traffic_cone.py:90-212`). 라이다 — 전방 0~10°+350~360° <0.25m 정지 / <0.5m 감속 3, 측면 80~100°/260~280° <0.35m → ∓20° 보정, 양측이면 정지(`traffic_cone.py:51-85`). 최종 `angle = clip(cone_angle + lidar_correction, ±50)`(`traffic_cone.py:218-224`).
- 그 외 미션 전부 없음. 속도 5/3/0.

### D~E. 튜닝값
| 항목 | 값 | 위치 |
|---|---|---|
| 콘 P / 단측 고정각 | 0.20 / ±20 | traffic_cone.py:186,168-173 |
| 정지/감속/측면 | 0.25 / 0.50 / 0.35 m | traffic_cone.py:16-18 |
| 속도 | 5 / 3 / 0 | traffic_cone.py:21-23 |
| 라이다 각도 가정 | 전방 0°, 반시계 1°/pt | traffic_cone.py:6-13 |

### F~G. 노하우 / 함정 / 우리 대비
- 구조(카메라 조향 + 라이다 안전 보정 가산)는 교과서적이나, 라이다 장착 오프셋 보정이 없고 콘 진입/이탈 판정 자체가 없음. 우리 스택 대비 가져올 것 사실상 없음. "측면 0.35m 근접 시 ±20° 밀어내기"는 콘 협로에서 벽 스침 방지 최후 가드로만 참고.

### H. 신뢰도: **하**
미니멀 초기 단계. 수치도 검증 흔적 없음.

---

## 종합: 우리 스택 관점 우선순위 액션

1. **[추월] jeonghun 원본과의 차이 재확인 결과, 우리 target_passing이 규정 대응(방향 선택·90cm 복귀·min_gap·타임아웃) 전 항목에서 우월.** 역수입할 것은 두 가지 소요소뿐 — (a) ALONGSIDE 중 라이다 타겟 상대좌표 기반 보조 목표점(obs+오프셋 PP), (b) 추월 중 타겟 추적 섹터의 비대칭화(−60°~+15° 개념). jeonghun의 좌측 고정·시간 고정 복귀·완전 정지는 도입 금지 목록.
2. **[차선] pumky의 윈도우별 차선폭 학습 + 단측 재구성**을 우리 BEV 차선에 이식 — 추월 기동 중 한쪽 차선만 보이는 구간의 lateral_offset 안정성 직결.
3. **[정지선/출발] foscar 정지선 검출기(수평 모폴로지+rising edge)와 "출발 후 n초 강제 직진"(jeonghun 1.5s / pumky 1.2s 수렴)** — 출발 미션과 겐트리 통과 직후 오검 방지 보험.
4. **[운영 위생] mango의 실차 체크리스트** — 노출 고정 launch 인자화, 정지→재출발 PID 리셋+유예, 검출/제어 주기 분리. 본선 당일 튜닝 워크플로에 반영.
5. **[콘] pumky ConeDriver의 체인 제약(dx/dy 범위·침범 금지·crossover 소유권)**을 우리 라이다 콘 미들라인에 제약식으로 추가 검토.
6. **[랩] pumky(신호등 재검출+25s 쿨다운)·gim-track(15s freepass)은 모두 우리 겐트리 목격 방식보다 열등** — 현행 유지 근거 확보.
