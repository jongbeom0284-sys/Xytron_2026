# 이전 코드 보관소

여기 있는 코드는 **빌드되지 않는다.** ROS workspace(`real/ros2_ws/src`) 밖에 있고
어떤 launch 에서도 참조하지 않는다. 그냥 두면 실차 동작에 아무 영향이 없다.

지운 게 아니라 옮긴 이유는 두 가지다. 되살릴 값이 있는 코드가 섞여 있고,
지금 선택이 왜 그렇게 됐는지에 대한 근거가 필요하기 때문이다.

## 무엇이 왜 여기 있나

### lane-detectors/ — 이전 차선 검출기 3종

지금은 TwinMixing 하나만 쓴다. 셋 다 실제로 돌아갔던 코드다.

| 폴더 | 방식 | 내려온 이유 |
|---|---|---|
| `white-left/` | 흰 좌측 경계를 ridge score → 방향 인식 DP → 밝은 줄무늬 쌍 RANSAC 으로 추적 | TwinMixing 으로 대체. 640×480 실측 캘리브가 들어 있어 **폴백 후보로 보존** |
| `fast-lane/` | LAB 정규화 후 흰/노랑 마스크 → 좌우 슬라이딩 윈도우 | 픽셀 출력만 냈고 제어 입력으로 쓰인 적이 없다 |
| `clrernet/` | CLRerNet 딥러닝 차선 + 고정 픽셀 기준 PID | 기준 사진(`pixel_reference.json`)을 끝내 만들지 못했다. `reference_ready: false` 인 채로 실차 검증 0회 |

`clrernet/` 안에는 upstream 소스, 가중치 위치 안내, 개발 단계 문서
(`docs/LANE_DRIVE_STAGE_SPEC.md`), 카메라 기준 사진 정책(`calibration/`),
Jetson 이관 스크립트가 함께 들어 있다. 가중치 `.pth` 자체는 `.gitignore` 대상이라
저장소에 없다. 원본은 `~/clrernet_eval` 에 있다.

### vehicle-nodes/ — 이전 차량 노드

| 파일 | 무엇이었나 | 지금은 |
|---|---|---|
| `Lane_follow.py` | HSV 노란 단일선 슬라이딩 윈도우 + PID. `/xycar_motor` 직접 발행 | 점선 외삽(GHOST/PREDICT) 로직이 `yellow_center.py` 로 이관됐다 |
| `Lane_follow_white.py.bak` | 위의 흰선 버전 | 참고용 |
| `LidarAvoidance.py` | 라이다 단독 회피. `/xycar_motor` 직접 발행 | 각도 보정(+pi, 좌측 양수)이 `obstacle_lidar.py` 로 이관됐다 |
| `figure_eight.py` | 8자 주행 시험 패턴 | 필요하면 되살린다 |

`Lane_follow.py` 와 `LidarAvoidance.py` 는 `/xycar_motor` 를 직접 발행한다.
현재 구조에서는 인식 계층이 모터를 건드리지 않으므로 그대로 되살리면 안 된다.
대시보드의 경쟁 퍼블리셔 검사에 걸려 기록 주행 시작이 거부된다.

### simulation/ — Unity 시뮬레이터

ROS-TCP Endpoint 기반. 실차와 메시지·토픽 규약이 달라서 두 workspace 를 한 터미널에서
동시에 `source` 하면 안 된다.

`track_drive/track_drive.py`(2,367줄)에는 신호등 게이트, 콘 모드, 추월 상태기계,
헤드라이트 검출 같은 미션 로직이 있다. 실차 쪽에는 아직 대응물이 없다.
**삭제 대상이 아니라 나중에 이식할 원본이다.**

### evidence/lane-comparison/ — 왜 TwinMixing 인가

같은 프레임 2장에 차선 알고리즘 16종을 돌린 결과 이미지와 비교 페이지다.
HoughLinesP, LSD, RANSAC, fast_lane, CLRerNet, CLRNet, UFLDv2, TwinLiteNet,
TwinMixing, YOLOP(PyTorch/ONNX 320·640·1280), YOLOPv2.

로컬에서 `lane_gallery.html` 을 브라우저로 열면 그대로 볼 수 있다.

### ipm/ — 미사용 IPM 패키지

ros-sports 의 Inverse Perspective Mapping 패키지. 미터 단위 경로를 검토할 때
가져왔지만 현재 방식은 픽셀만 쓴다. 패키지 README 스스로 미사용을 명시했다.

## 되살리는 법

1. 해당 폴더를 `real/ros2_ws/src/kobot_lane_drive/` 아래 원래 위치로 되돌린다.
2. `kobot_lane_drive/setup.py` 의 `entry_points` 와 `data_files` 에 다시 등록한다.
3. `colcon build --symlink-install` 후 시험을 돌린다.
4. 인식 노드라면 `/xycar_motor` publisher 가 없는지 먼저 확인한다.
   모터 출력은 승인 잠금 bridge 한 곳에만 둔다.

Git 이력이 보존돼 있으므로 `git log --follow <파일>` 로 원래 맥락을 볼 수 있다.
