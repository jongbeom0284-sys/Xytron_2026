# Open-source provenance

| 역할 | Upstream | 고정 기준 | 라이선스 | 포함 방식 |
|---|---|---|---|---|
| 차선 검출 | https://github.com/hirotomusiker/CLRerNet | `dae038f67da57e292e5293a68c9c1c2922de13c2` | Apache-2.0 | `third_party/CLRerNet` |
| PID 계산 | https://github.com/m-lundberg/simple-pid | v2.0.1 `6e6f4b000cdf7830635674cb89a6e823f5025784` | MIT | `_vendor/simple_pid.py`, 설명 문자열만 축약 |
| 목표 픽셀 구조 | https://github.com/autorope/donkeycar/blob/38dd2b4fa82a504ee1a0e0d4409d81183051a125/donkeycar/parts/line_follower.py | `38dd2b4fa82a504ee1a0e0d4409d81183051a125` | MIT | 설계 참고, 코드 미복사 |
| 두 샘플 행 구조 | https://github.com/waveshareteam/ugv_jetson/blob/18e9a24590e1bfdb39a1bc24aaf8a6af820805d3/cv_ctrl.py | `18e9a24590e1bfdb39a1bc24aaf8a6af820805d3` | AGPL-3.0 | 설계 참고만 하고 코드 미복사 |

CLRerNet의 CUDA NMS 두 소스에는 Jetson PyTorch 2.8 API 호환 패치만 적용했다.
패치는 `patches/torch28_nms.patch`에 보존한다.

프로젝트 자체 코드는 다음 연결부다.

- CLRerNet 결과에서 차량/카메라 중심 바로 왼쪽의 가장 가까운 차선 선택
- 고정된 `near_y_ratio`, `far_y_ratio`에서 x 좌표 선형 보간
- 기준 사진에서 얻은 x 비율을 vendored `simple-pid` setpoint로 설정
- 두 PID 출력 합산과 최대 조향 제한
- 차선/기준값/샘플 행 실패 시 `BLOCKED`, 모터 없는 preview와 overlay
- 승인 잠금 bridge에서 Kobot 기존 모터 입력 형식으로 변환

픽셀을 미터로 변환하거나 BEV, 평행곡선, Stanley 조향을 계산하는 자체 코드는
없다. 이전 IPM/Shapely/PythonRobotics runtime은 현재 패키지에서 제거했다.

평가본 `/home/xytron/clrernet_eval`은 가중치와 런타임 확인 출처일 뿐 최종 개발
위치가 아니다. 최종 위치는
`/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_lane_drive`다.
