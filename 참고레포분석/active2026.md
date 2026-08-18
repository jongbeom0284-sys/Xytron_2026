# 2026 활동 레포 3종 정밀 분석 (junjun / luna / dltldhs)

분석일: 2026-08-13 | 본선 D-12 (2026-08-25)
경로 기준: `scratchpad/repos/` 하위. 모든 인용은 `파일:라인`.
세 레포 모두 shallow clone이라 git 히스토리는 각 1커밋만 보임 → 커밋 활동 분석은 제한적, 파일 내용 기반 판단.

---

## 1. junjun — junjunjunman/kookmin_xytron (푸시 2026-08-03/04)

### A. 메타
- **ROS2** (rclpy, `create_subscription`/`QoSProfile` 사용), **실차** 대상: `/image_raw`, `/scan`(BEST_EFFORT), `xycar_motor` 구독·발행, `/dev/videoCAM` 계열 워크플로.
- 마지막 커밋: `12f98bb` 2026-08-04 "Rename cone_individual_areas.csv ..." (junjun/.git). 모델 파일명 `best_records_0724.pt`(7/24 학습 추정, cone_drive_prepare/lidar_yolo.py:43), 개발자 로컬경로 `/Users/seongjun/Documents/국민대 자율주행/…` (macOS에서 오프라인 분석 + 실차 별도).
- **실주행 증거**: recording.py(카메라+`xycar_motor` 텔레메트리 CSV 녹화), 실측 라이다 캡처 파일명(lidar_0235.png 등), 캘리브레이션 결과 CSV 2종 존재 → 벤치/트랙 테스트는 실제로 함. 단, 레포 자체는 **준비용 스크래치**이고 본 주행 코드는 차량 내 `/home/xytron/xycar_ws/src/drive/`(cone_cam_node.py:25 참조 경로)에 별도 존재 추정.
- 9회(작년) 참가팀의 연습 레포로 추정되나 올해 대회 준비 정황(7~8월 커밋, 라바콘 집중)이 뚜렷.

### B. 아키텍처
- 단일 노드 `CamNode` + 외부 모듈 `cone_drive.start_cone_mode()` 콜 구조. FSM 없음 — **불리언 플래그 1개**(`is_cone_driving_active`)로 "차선주행 ↔ 콘주행" 2모드 전환 (cone_cam_node.py:118-137).
- 인식 스택이 **전부 YOLO(v11) 세그멘테이션** 중심: 카메라용 `best.pt`(클래스 left/right/yellow/cone), 라이다 렌더 이미지용 `lidar.pt`(클래스 left/right/current).
- 특이: **라이다 스캔을 matplotlib 이미지로 렌더한 뒤 그 이미지에 YOLO를 돌려** 콘 좌/우 열을 분류(lidar_node.py + lidar_yolo.py + lidar_limit.py). 창의적이지만 실시간성이 의심되는 간접 구조.

### C. 미션별 구현
- **신호등/출발/추월/지름길/랩카운팅: 레포에 없음.** (본 레포는 라바콘 미션 특화 준비물)
- **차선주행**: YOLO 세그 마스크(left/right) → ROI(y=0.60H~0.70H, x=0.05~0.95W; cone_cam_node.py:52-55) → 히스토그램 + 슬라이딩윈도우 4개(margin 40, minpix 30; :165-168) → 좌우 윈도우 중점 error → PID. 오프라인 대안으로 undistort→YOLO→BEV→Canny→HoughLinesP→선분각 평균(img_hough.py:82-106).
- **라바콘 진입 판정**: cone 클래스 최대 마스크 면적 > **6000 px** (cone_cam_node.py:71,118). 면적 임계값은 cone_width_checking.py(다각형 클릭 면적 측정)로 실측 튜닝.
- **라바콘 주행**(cone_drive.py):
  1) left/right 콘 박스를 y내림차순 정렬 후 i번째끼리 페어링 → 중점 연결 = 중앙경로 (:85-99)
  2) 로봇점(current 클래스 or 화면 하단중앙)→중점들 선분각의 **산술평균** = 조향각 (:107-122) — "선분각 평균화(look-ahead)"로 급커브 언더스티어 완화 주장(cone.md:21-23)
  3) 한쪽만 보이면 좌콘+200px / 우콘-200px 가상 목표 (:125-131)
  4) 라이다 보정: 정면 인덱스 230~275 최소거리 < **0.4 m** → 좌(mean ranges[350:400]) vs 우(mean ranges[100:150]) 넓은 쪽으로 **±30°** 긴급조향 (:167-182); 측면 < **0.25 m** → **±10°** 반발 (:185-191); 최종 클램프 ±40° (:199-200)
  5) 종료: left·right 모두 미검출 → 차선주행 복귀, 복귀 첫 프레임 angle=0 (cone_cam_node.py:130-133)
- **라이다 인덱스 체계**: `ranges[1:505]` = 504빔, **정면 = 252** (cone_cam_node.py:82, cone_drive.py:28,154). 실측 CSV상 전방 콘 코스 유효 인덱스 약 215~288 (lidar_index_ranges.csv).

### D. 제어
- PID: **kp 0.4 / ki 0.0002 / kd 0.75**, 적분 상한 500, error 부호 반전 시 적분 리셋 (cone_cam_node.py:63-68, 216-227).
- 속도: **고정 5.0** (cone_cam_node.py:59). 조향 클램프 ±40 (:60). 키보드 노드는 MAX_SPEED/ANGLE 50 (conrtol_keyboard.py:36-37).

### E. 튜닝값 총표
| 항목 | 값 | 위치 |
|---|---|---|
| YOLO conf (주행) | 0.25 | cone_cam_node.py:89 |
| YOLO conf (오프라인) | 0.4 | img_hough.py:52 |
| 콘 진입 면적 | 6000 px | cone_cam_node.py:71 |
| ROI | y 0.60~0.70H, x 0.05~0.95W | cone_cam_node.py:52-55 |
| 슬라이딩윈도우 | n=4, margin=40, minpix=30 | cone_cam_node.py:165-168 |
| PID | 0.4 / 0.0002 / 0.75, I-max 500 | cone_cam_node.py:63-68 |
| 속도/조향 | 5.0 고정 / ±40° | cone_cam_node.py:59-60 |
| 전방 안전거리 | 0.40 m (idx 230~275) | cone_drive.py:167-170 |
| 긴급 회피각 | ±30° | cone_drive.py:180-182 |
| 측면 반발 | <0.25 m → ±10° | cone_drive.py:185-191 |
| 한쪽콘 오프셋 | ±200 px, 전방가상 150 px | cone_drive.py:126-131 |
| BEV 원근변환 | src [[77,389],[616,402],[506,330],[201,324]] → dst [[100,300],[400,300],[400,100],[100,100]], 출력 500×350 | img_hough.py:19-22 |
| Canny/Hough(BEV) | 50/150, th30, minLen30, gap20 | img_hough.py:81-82 |
| 픽셀-거리 기준 | 0.11 m 레퍼런스 클릭 캘리브 | cone_length_checking.py:31-34 |

### F. 실전 노하우 / 함정
- 노하우: ① 임계값(면적·거리·인덱스)을 **전용 측정 툴 5종**으로 실측해서 CSV화(면적 체커, 길이 체커, 라이다 인덱스 탐색기, 피자조각 마스킹) — 튜닝을 감이 아닌 도구로 함. ② `xycar_motor` 텔레메트리를 프레임 단위 CSV+PNG로 남기는 recording.py — 학습·리플레이 겸용. ③ 라이다 시각화를 RViz 좌표계와 일치시키는 변환 주석(lidar_node.py:86-99).
- **함정(반면교사)**:
  1) cone_drive.py·cone_cam_node.py 전체에 **`[cite: 4, 6]` 등 LLM 출력 인용 마커가 그대로 커밋됨** — 이 상태로는 구문/런타임 에러로 실행 불가. AI 생성 코드를 검수 없이 커밋한 흔적.
  2) 라이다 무효값 처리 오류: `np.isfinite`→10.0 치환(cone_drive.py:163-164)인데, 표준 YDLidar 설정은 `invalid_range_is_inf: false`(dltldhs ydlidar.yaml:24)라 **무효값이 0.0으로 들어옴** → 0.0이 그대로 min에 잡혀 상시 "40cm 이내 장애물" 오검출 가능.
  3) 주행노드는 `drive_cmd` 토픽 발행(cone_cam_node.py:31), 키보드/레코더는 `xycar_motor`(conrtol_keyboard.py:30) — 모터로 가는 변환 노드가 레포에 없어 토픽 계약 불일치.
  4) 콜백 안 `cv2.imshow` — 헤드리스 실차에서 죽는 패턴.
- **G. 우리 대비**: 가져올 것 — 전방 0.4m/측면 0.25m 2단 라이다 반발 로직(우리 퓨어퍼수트 위 안전레이어로 이식 용이), 임계값 실측 툴링 문화, 콘 페어링-중점-선분각평균(우리 HSV 게이트 방식의 교차검증용). 반면교사 — AI 코드 무검수 커밋, 라이다 0.0 처리, 토픽 계약 불일치.
### H. 신뢰도: **중** — 실측 데이터·툴은 진짜지만 커밋된 주행 코드는 실행 불가 상태. "이 팀의 실제 본선 코드"가 아니라 "준비 과정 창고"로 읽어야 함.

---

## 2. luna — LunaGunwoo/xycar_ws_mgw (푸시 2026-08-12, 어제)

### A. 메타
- **ROS2 Humble, 실차 전용** 워크스페이스 전체 + 독립 학습 프로젝트(`ai/`). 개발자 Gunwoo Moon 단독, 마지막 커밋 "feat(ai): add stateless guided curriculum" 2026-08-13 00:30.
- 인프라 수준이 이례적으로 높음: Tailscale로 차량 SSH 배포(README.md:10-22), 에이전트 하네스 규칙 AGENTS.md(실차 안전규칙 포함), uv 잠금 학습환경, 모델 artifact를 manifest+SHA256으로 버전관리(xycar_ai_drive/README.md:45-62).
- **실주행 증거 강함**: 차량 CPU(Ryzen 7 7730U)에서 추론 p95 11.802 ms 실측(xycar_ai_drive/README.md:180-182), 텔레옵 데이터셋 11세션 4,614샘플·split 3,227/685/702(ai/README.md), VESC 펌웨어 bin + 2026-06-17자 모터 설정 XML(etc/motor_vesc/), 차량 2대 운용(레거시 `xycar`=Ryzen CPU, 신형 `xycar-gpu`=Jetson CUDA 컨테이너; xycar_ai_drive/README.md:221-236, deploy/jetson/x27-jetson.sh). ai/README.md에 "기존 예선 및 stateless 모델과 같은 loss"라는 문구 → **예선을 이 모델 계열로 치렀음을 시사**.

### B. 아키텍처
독립 스택 2개 + 표준 워크스페이스:
1) **차선주행 = 카메라 단독 end-to-end 모방학습**(xycar_ai_drive + ai/): 640×480 프레임 → (옵션) 도로 사다리꼴 BEV warp → 224×224 → timm **ViT-tiny/small** → angle/speed 각 **201-클래스 분류** head. TorchScript로 배포, 차량에서 20 Hz 추론·발행.
2) **라바콘 = 라이다 단독 기하 계획**(track_drive 라이브러리 + xycar_debug 뷰어/스페이스키 주행기): 클러스터→좌우 경계 그래프 탐색→중앙선→적응형 lookahead 퓨어퍼수트.
3) FSM/미션 오케스트레이터는 **아직 없음**(track_drive/README.md:17-18 "전체 맵 주행용 실행 진입점 없음"). 신호등·추월·지름길·랩카운팅 코드 부재 (8/12 기준).

### C. 미션별 구현
- **차선(트랙) 주행**: 모방학습.
  - 라벨 계약: `class_id = round(clamp(cmd,-100,100))+100`, angle=id-100, speed=max(0,id-100) (xycar_ai_drive/control.py:9-10,70-96) → **모터 스케일이 ±100임을 전제**.
  - 학습: CE_angle + 0.5·CE_speed + 0.2·(EMD_angle + 0.5·EMD_speed) (ai/README.md:320-322). 수평 flip 시 angle 부호 반전(p=0.5). warp 학습은 train angle 타깃을 **동일 세션 5프레임 중심 평균**으로 스무딩(ai/README.md).
  - 데이터: 게임패드 텔레옵(전진 7~27 스케일). `max_forward_speed>=20` 세션만 채택.
  - **guided curriculum(DAgger식)**: 모델 출력 + 사람 잔차 보정 `executed_angle = clamp(model + stick*200, ±100)`, `executed_speed = clamp(model + RT*2 - LT*5, 0, cap)`; 실행된 명령을 라벨로 재수집, 세대별 sampling mass = 0.5^(현재세대-g) 지수감쇠 (xycar_ai_drive/README.md:70-81, ai/README.md:369-386). 1라운드 cap=9, 설정파일상 cap=27 (guided_policy_collection.yaml:26).
- **라바콘**(track_drive/cone_following.py):
  - 클러스터 필터: 거리 0.30~10 m, 전방 y≥0.20 m, |x|≤4 m, 인접점 gap ≤0.35 m·인덱스 gap ≤2, 포인트 2~8개, 지름 ≤0.35 m (:39-49, config/cone_drive.yaml:12-23). 스캔 wrap-around(0°) 병합 처리(:772-781).
  - 경계 구성: 좌/우 각각 콘 그래프에서 edge 0.70~5.0 m·전진성 ≥0.25 m·누적회전 ≤105°·길이 ≤5 경로를 최대 120개 열거, `0.85·개수 + 0.35·전방길이 − 0.30·회전비용` 점수화 (:462-505).
  - 중앙선: 좌×우 경로 조합마다 0.45 m 간격 샘플, **차로폭 2.0~6.2 m 게이트**, 폭 변동 ≤1.7 m (:508-537). 실패 시 좌/우 단독 2차 다항 피팅 + 추정 반폭(EMA α=0.2, 1.4~3.0 m) 오프셋 폴백 (:263-329).
  - 조향: lookahead를 경로 곡률로 3단 선택(|κ|≥0.12→1.2 m, ≥0.045→1.8 m, else 2.6 m; :602-620) → 퓨어퍼수트 κ=2x/d² (:450-459) → `angle = κ·160 + heading·220`, 클램프 ±30(하드웨어 한계 ±100) (control.py:98-103,22).
  - 속도: |κ|≤0.025→5, ≥0.045→3, 사이 선형보간; 한쪽경계/폴백은 3 고정 (control.py:113-123,87-94).
  - **안전 게이트**: scan 0.30 s 초과 시 정지, 계획 horizon <1.6 m 정지, confidence(paired 0.50/양측폴백 0.35/단측 0.22) 미달 정지, 이전 계획 재사용분(≤3프레임)은 **표시 전용, 주행 금지** (control.py:54-96, cone_drive.yaml:47-48).
- **신호등/출발/고정vs방해 판별/추월/지름길/랩카운팅: 없음.**

### D. 제어
- 콘 주행: 위 참조(gain 160/220, 속도 3~5, ±30 클램프).
- AI 주행: 모델이 angle·speed 직접 출력, 후처리 cap 없음(README 계약: reverse만 금지). 20 Hz 발행, OFF 시 [0,0] 상시 발행. Joy/카메라 0.25 s stale, 추론 실패, 모터 구독자 소실, **경쟁 모터 퍼블리셔 출현 시 즉시 OFF + [0,0]** (front_cam_policy.yaml:24-29, xycar_ai_drive/README.md:26-34). 종료 시 정지명령 5회 반복.

### E. 튜닝값 총표 (핵심)
| 항목 | 값 | 위치 |
|---|---|---|
| 모터 토픽/스케일 | `/xycar_motor` 절대경로, Float32MultiArray[angle,speed], angle ±100 | cone_drive.yaml:5-10, control.py:22 |
| 콘 클러스터 | gap 0.35 m/idx 2, 2~8 pt, 지름 0.35 m, 0.3~10 m | cone_drive.yaml:12-21 |
| 차로폭 게이트 | 2.0~6.2 m (Δ≤1.7) | cone_drive.yaml:33-35 |
| lookahead | 1.2/1.8/2.6 m (κ 0.12/0.045) | cone_drive.yaml:39-41,50-51 |
| 조향 게인 | curvature 160, heading 220, 클램프 30 | cone_drive.yaml:67-72 |
| 콘 속도 | 5(직선)~3(급커브·단측) | cone_drive.yaml:62-64 |
| 안전 타임아웃 | scan 0.30 s, joy/추론 0.25 s, 정지 5회 발행 | cone_drive.yaml:55, front_cam_policy.yaml:25-28 |
| AI 모델 | ViT-small/tiny 224², 201클래스×2 head, TorchScript | ai/README.md, control.py:9-10 |
| 추론 성능 | CPU p95 11.8 ms (Ryzen 7 7730U, thread 8, warmup 3) | xycar_ai_drive/README.md:180-182, front_cam_policy.yaml:38-39 |
| 텔레옵 스케일 | angle ±100, 전진 7(수동)~27(guided cap), 후진 5 | gamepad_teleop.yaml:25-27, guided_policy_collection.yaml:26 |
| 잔차 보정 | 스틱×200, RT +2, LT −5, deadzone 0.05 | guided_policy_collection.yaml:23-27 |
| 세대 감쇠 | generation_decay 0.5 | ai/README.md:381-386 |
| 카메라 | v4l2_camera, /dev/videoCAM, YUYV 640×480@30 | xycar_cam.launch.py:29-33 |

### F. 실전 노하우
- **fail-closed 원칙의 교과서**: 시작은 항상 OFF, 상승엣지 토글+release 요구, stale→정지, 경쟁 퍼블리셔 감지(DDS 그래프 검사, `/ros_bridge`만 예외 허용), 재사용 경로 주행 금지, 경로 유실 0.5 s 내 복구만 자동재개 (xycar_debug/README.md:52-63).
- `ROS_NAMESPACE=xycar` 환경에서 상대 토픽명이 `/xycar/...`로 리매핑되는 것을 피하려 **모터/스캔 토픽을 절대경로로 고정** (cone_drive.yaml:6-10 주석) — 표준 배포 환경(x0.sh: `ROS_DOMAIN_ID=7`, `ROS_NAMESPACE=xycar`)에서 실제로 밟는 함정.
- 조향 부호 검증 절차 명문화: "Viewer/차량 들어올린 상태 검사로 반전 확인 후에만 steering_sign=-1" (cone_drive.yaml:69-70).
- 데이터셋 규율: 진행중 `_recording_*` 전송 금지, incomplete 세션 학습 자동배제, 모델은 checksum 검증 후 로드.
### G. 우리 대비 — 가져올 것 / 표준 배포 코드에서 발견한 사실
- 가져올 것: ① 콘 클러스터 파라미터 세트(0.35 m gap·지름, 2~8 pt)와 wrap-around 병합 — 우리 라이다 클러스터와 직접 비교 가능. ② 곡률 기반 lookahead 3단 + 곡률 기반 속도 선형보간 — 우리 퓨어퍼수트+속도정책에 그대로 이식 가능한 형태. ③ 경쟁 퍼블리셔 감지·stale 정지·정지명령 N회 반복 등 안전 게이트. ④ 차로폭 게이트로 잘못된 경계쌍 기각하는 아이디어(단, 2.0~6.2 m 값 자체는 실코스 콘 간격보다 넓어 보임 — 그대로 복사 금지).
- **표준 배포 코드 발견 사실(중요)**:
  1) **모터 경로 = ROS1**: VESC 드라이버는 `osrf/ros:noetic-xycar` 도커에서 돌고, 호스트에서 `ros1_bridge dynamic_bridge --bridge-all-topics`로 ROS2와 연결 (etc/motor_vesc/motor:5,40-53,81). 장치 `/dev/ttyMOTOR`(:17). Jetson 차량에선 브리지 노드명이 `/ros_bridge`이며 `ROS_LOCALHOST_ONLY=1` (deploy/jetson/x27-jetson.sh:8).
  2) **모터 스케일 angle ±100 / speed ±100** (control.py:22, 텔레옵 max_angle 100, my_motor/go.py 속도 스위프 ±100).
  3) 표준 launch 환경: `ROS_DOMAIN_ID=7`, `ROS_NAMESPACE=xycar` — 상대 토픽명 리매핑 주의.
  4) 카메라 노출은 `v4l2-ctl -d /dev/videoCAM -c auto_exposure=1 -c exposure_time_absolute=N`으로 제어(표준 hough 앱은 100 사용).
### H. 신뢰도: **상** — 실측 수치·테스트·배포 스크립트가 일관되고 자체 테스트 코드 존재. 단 미션 커버리지가 (차선+콘)뿐이라 "본선 전체 미션 대응"으로는 미완. 남은 12일 내 모방학습으로 신호등/추월까지 갈 수 있을지는 미지수 → 트랙주행 속도 경쟁에선 위협, 미션 완주 안정성에선 우리가 우위 가능.

---

## 3. dltldhs — dltldhs03-code/xycar_ws (푸시 2026-07-21)

### A. 메타
- **ROS2 Humble**, "Initial commit" 단일 커밋(2026-07-21)으로 워크스페이스 통째 업로드. 62 MB의 대부분이 자이트론 표준 배포물.
- **팀 자작 주행 코드 사실상 전무**: luna와의 전체 diff 결과 자작 변경은 ③개뿐 — study/my_motor/go.py의 속도 스위프 주석 해제(−100~+100 램프 테스트), study/my_lidar/lidar_scan.py 로그 한 줄 제거, app_ultra_viewer는 배포 버전 차이. `src/yolo_ros/`는 빈 디렉터리(중첩 git 미푸시).
- 실주행 증거 없음. 백업/학습 초기 단계로 판단.

### B~D. 아키텍처/미션/제어
- 자작 코드가 없어 해당 없음. 대신 이 레포는 **자이트론 2026 표준 배포의 원본 스냅샷**으로 가치가 큼.

### E. 표준 배포에서 뽑은 파라미터 총표 (우리가 몰랐을 수 있는 것)
| 항목 | 값 | 위치 |
|---|---|---|
| LiDAR 포트/속도 | /dev/ttyLIDAR, 512000 baud, 10 Hz | src/xycar_device/xycar_lidar/params/ydlidar.yaml:3,6,23 |
| LiDAR 각도/거리 | −180~180°, 0.1~16 m, fixed_resolution·reversion·inverted 모두 true | ydlidar.yaml:12-14,19-22 |
| **무효값 표현** | `invalid_range_is_inf: false` → **무효 = 0.0** | ydlidar.yaml:24 |
| 카메라(표준) | usb_cam 패키지, 파라미터는 usb_cam share의 params.yaml | xycar_cam/launch/xycar_cam.launch.py |
| 노출 제어 | v4l2-ctl auto_exposure=1, exposure_time_absolute=100 | app_hough_drive.py:85-88,355 |
| 모터 메시지 | 실차 표준 = Float32MultiArray[angle,speed] (XycarMotor msg는 존재하나 주석 처리) | app_hough_drive.py:13,44-45,55 |
| 표준 환경 | ROS_DOMAIN_ID=7, ROS_NAMESPACE=xycar | etc/gui-shell/x0.sh~ |
| 표준 YOLO | `yolo_bringup yolo.launch.py` (yolo_ros가 표준 탑재) | etc/gui-shell/x4.sh |
| 모터 실행 | 데스크톱 x27 = `motor`(ROS1 도커+브리지) | etc/gui-shell/x27.sh |
| 모터 캘리브레이션 | ROS1 `xycar_motor_calibration.launch` (x18) | etc/gui-shell/x18.sh |
| 초음파 | 토픽 `xycar_ultrasonic` Int32MultiArray | src/study PACKAGE 관례 + app_ultra_viewer |
| **표준 차선인식 레퍼런스** (app_hough_drive) | ROI row 300~380, 기준행 L_ROW 40 | app_hough_drive.py:25-28 |
| 〃 에지/허프 | Canny 60/75, HoughLinesP(1, π/180, 50, 50, 20), 기울기 필터 \|m\|>0.2 | :109,114,137 |
| 〃 **차선 폭 상수** | 한쪽 소실 시 반대쪽 ±**380 px** 로 복원 (ROI 행 기준 차선폭) | :310-313 |
| 〃 제어 | P게인 1.0 (midpoint−320), Fix_Speed 12 | :51,374 |
| 〃 소실 대응 | 이전 x_left/x_right 유지(prev_*) | :279-303 |

### F~G. 노하우 / 우리 대비
- 이 레포의 가치는 **"기준선"**: 표준 hough 앱의 380 px 차선폭·ROI 300~380·Canny 60/75는 실코스에서 자이트론이 검증한 초기값이므로, 우리 BEV 차선 인식이 무너졌을 때의 백업 파라미터/백업 알고리즘으로 채택 가능.
- `invalid_range_is_inf: false`(무효=0.0)는 세 팀 공통 함정 포인트 — 우리 라이다 클러스터·상대속도 계산에서 0.0 필터(≥range_min 0.1 m) 확인 필수.
- 경쟁 정보 관점에서는 무해(공개된 표준 코드 백업 수준). 단 7/21에 워크스페이스를 백업했다는 것 자체가 실차 세팅을 시작했다는 신호.
### H. 신뢰도: 표준 구성 정보로서 **상** / 팀 전력 정보로서 **하**(자작 코드 없음).

---

## 종합 — 우리(kmu_race) 대비 액션 아이템

1. **[하드웨어 계약 검증 — 즉시]** 모터 명령이 ROS1 도커+dynamic_bridge를 거친다는 것, angle 스케일 ±100, `ROS_NAMESPACE=xycar`로 인한 상대 토픽 리매핑, 라이다 무효값 0.0 — 이 4가지를 우리 io/motor·perception에서 명시 확인. 특히 토픽은 luna처럼 **절대경로 `/xycar_motor`, `/scan`** 로 고정 권장.
2. **[안전 게이트 이식]** luna의 fail-closed 패턴(센서 stale 0.25~0.3 s → 즉시 [0,0], 정지명령 5회 반복, 경쟁 퍼블리셔 감지, 시작 시 OFF)을 우리 FSM에 저비용으로 추가 가능. 본선 재출발/오작동 리스크를 직접 줄여줌.
3. **[콘 주행 보강]** ① luna의 곡률 적응 lookahead(1.2/1.8/2.6 m)와 곡률-속도 선형보간을 우리 퓨어퍼수트+속도정책에 적용. ② junjun의 2단 라이다 반발(전방 0.4 m→넓은 쪽 ±30°, 측면 0.25 m→±10°)을 최후 안전레이어로 추가. ③ 클러스터 파라미터(gap·지름 0.35 m, 2~8 pt)로 우리 값 교차검증.
4. **[전략 판단]** luna는 속도(모방학습 트랙주행)에 올인, 미션 로직 부재. junjun은 라바콘 특화, 본체 코드 미공개. dltldhs는 초기 단계. → 8/13 시점 공개 정보 기준, **미션 완주형 FSM을 이미 갖춘 우리 구조가 유리**하며, 남은 기간은 신규 알고리즘보다 위 1·2번 하드닝에 쓰는 편이 기대값 높음.
