# models — 추론 모델 통일 폴더

모델 파일은 **전부 여기**에 둔다. 코드의 기본 경로가 이 절대경로를 가리킨다.

| 파일 | 용도 | 원 출처 |
|---|---|---|
| `yolopv2_lane_fp16.engine` | 차선 (TensorRT FP16, 이 Orin 전용) | `~/yolotl_offline_eval/models/` |
| `yolov8n.pt` | 차량 인식 (YOLOv8 nano) | `kobot_vehicle/kobot_vehicle/` |

- TorchScript 폴백(`yolopv2.pt` 156MB)은 2026-08-22 제거했다 — 엔진 전용.
  엔진 적재 실패는 조용한 성능 저하가 아니라 즉시 기동 실패로 드러난다.
- 가중치 파일은 git 에 넣지 않는다 (`.gitignore`). 새 장비에서는 위 출처에서 복사한다.
- 엔진은 **장비 종속**이다. 다른 Jetson 에서는 yolopv2.pt 로 다시 구워야 한다.
