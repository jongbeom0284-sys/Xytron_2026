# 이 패키지의 출처

| 역할 | Upstream | 포함 방식 |
|---|---|---|
| 주행영역·차선 세그멘테이션 | TwinMixing | `third_party/TwinMixing` (모델 정의 + `pretrained/large.pth`) |

나머지는 전부 프로젝트 자체 코드다.

- `yellow_center.py` — 노란 중앙선 검출과 점선 외삽
- `lane_policy.py` — 차선 폭 측정, 평상시 폭 학습, 주행 라인 상태기계
- `obstacle_lidar.py` — LaserScan → 차량 좌표계 코리도
- `twinmixing_center.py` — 마스크 → 중심 경로, 화면 오버레이
- `geometry.py` — 레이트 리밋 · 클램프 · 저역통과
- `twinmixing_center_node.py` — ROS 인터페이스와 안전 차단
- `twinmixing_center_drive_bridge_node.py` — 승인 잠금 모터 형식 adapter
- `recording_lane_viewer.py` — 녹화 재생 분석 뷰어

CLRerNet, simple-pid, IPM 은 2026-08-19 재편에서 `legacy/lane-detectors/` 로
내려갔다. 전체 이력은 [../../../docs/PROVENANCE.md](../../../docs/PROVENANCE.md).

픽셀을 미터로 환산하거나 BEV·IPM·TF 를 쓰는 코드는 이 패키지에 없다.
카메라와 트랙이 고정이라는 전제 아래 픽셀 기준으로만 판단한다.
