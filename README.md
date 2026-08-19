# xycar-kmu-2026 — 국민대 자율주행 경진대회 2026

Jetson Orin NX 차량 `kobot` 의 주행 코드 전부를 **이 저장소 하나**에 둔다.
스택이 둘 있고 서로 다른 폴더에 산다. 둘 다 같은 차에서 돈다.

```text
xycar-kmu-2026/
├── real/            ★ kobot 스택 — 실차에서 실제로 도는 코드 (빌드 대상)
├── legacy/            kobot 스택의 이전 시도 보관소 (빌드 안 됨)
├── docs/              라바콘 자율주행 기술 보고서
├── xycar_ws/        ★ kmu_race 스택 — 미션 FSM·라바콘·신호등 (별도 workspace)
├── 실차런북.md        현장 운용 런북. 세팅·출발 절차·날짜별 변경분
├── 참고레포분석/      타 팀 공개 레포 전수 분석
├── 백업/              설정 백업
├── deploy.sh · 아침점검.command · 비상정지.command · 경쟁3팀추적.sh
└── *.pdf              대회 규정 · 차량 사용설명서
```

## 두 스택이 각각 무엇인가

| | `real/` (kobot) | `xycar_ws/src/kmu_race` |
|---|---|---|
| 맡는 것 | 차선 인식·추종, 센서 노드, 모터 어댑터, 웹 대시보드 | 미션 상태기계, 라바콘 게이트, 신호등, 출발·정지 규칙 |
| 차선 인식 | TwinMixing 신경망 + 노란 중앙선 우선 | 색 기반 BEV 검출기 |
| 차 위 경로 | `~/2026_Xytron/real/ros2_ws` | `~/xycar_ws` |
| 문서 | [real/README.md](real/README.md) → [real/docs/](real/docs/) | [실차런북.md](실차런북.md) |
| 시험 | 75개 (`kobot_lane_drive` 54 + `kobot_vehicle` 21) | 157개 |

**처음 오면 [real/docs/DRIVING_POLICY.md](real/docs/DRIVING_POLICY.md) 부터 읽는다.**
차가 어디를 보고 어떻게 달리는지가 실측 근거와 함께 적혀 있다.
폴더별 소유물은 [real/docs/FOLDERS.md](real/docs/FOLDERS.md) 한 곳에 있다.

## 주행 규칙 한 줄 요약

노란 중앙선을 중심으로 2차선 전체 폭을 쓰며 달린다. 노란선이 끊기면 1초까지 직전
노란선을 유지하고, 그 뒤에야 도로 중앙으로 내려간다. 한쪽 차선이 막히면 반대편
차선으로 비켜섰다가 추월 후 중앙으로 돌아온다.

## 시작하기 (kobot 스택)

```bash
cd ~/2026_Xytron/real/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7 ROS_NAMESPACE=xycar ROS_LOCALHOST_ONLY=1
ros2 launch kobot_vehicle sensors.launch.py
```

`sensors.launch.py` 는 모터를 실행하지 않는다. 모터는 명시적 승인과
`armed:=true` 없이는 절대 돌지 않는다. 절차 전문은
[real/docs/OPERATIONS.md](real/docs/OPERATIONS.md).

## 저장소 이력

원래 두 저장소였다 — kobot 스택은 `2026_Xytron`, kmu_race 는 `xycar-kmu-2026`.
2026-08-19 에 `2026_Xytron` 의 전체 이력을 이쪽으로 병합했다(`git log` 에 둘 다
남아 있다). **이제부터 모든 푸시는 `gksgpals/xycar-kmu-2026` 으로 간다.**
차(kobot)의 `~/2026_Xytron` 체크아웃도 이 저장소를 추적한다.

> 경고: 대시보드의 모터 HTTP API 에는 인증이 없다. 신뢰된 로컬망이나 Tailscale
> 안에서만 쓰고 인터넷에 직접 공개하지 않는다.
