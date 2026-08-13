# Open-source provenance

| 역할 | Upstream | 고정 기준 | 라이선스 | 포함 방식 |
|---|---|---|---|---|
| 차선 검출 | https://github.com/hirotomusiker/CLRerNet | `dae038f67da57e292e5293a68c9c1c2922de13c2` | Apache-2.0 | `third_party/CLRerNet` |
| IPM | https://github.com/ros-sports/ipm | Humble `29439254c6c1eaabfaf1e8a42345594ffd55f3b5` | Apache-2.0 | workspace `src/ipm` 전체 |
| 평행 경로 | https://github.com/shapely/shapely | Jetson 설치본 `2.1.2` | BSD-3-Clause | Python 의존성, 소스 미복사 |
| Stanley | https://github.com/AtsushiSakai/PythonRobotics | `1fe4fb980f6a12fe21c3c33d2b4da97a52c9154d` | MIT | 필요한 Stanley/spline/angle 파일과 LICENSE |

CLRerNet의 CUDA NMS 두 소스에는 Jetson PyTorch 2.8 API 호환 패치만 적용했다.
패치는 `patches/torch28_nms.patch`에 보존한다. `ipm`과 PythonRobotics 파일의
알고리즘은 변경하지 않는다.

프로젝트 자체 코드는 ROS 메시지 변환, CLRerNet 결과에서 차량 중앙 왼쪽의 가장
가까운 후보 선택, 공개 라이브러리 호출, 상태 차단, Kobot 기존 모터 입력 형식
변환뿐이다. 자체 BEV, 픽셀-거리 공식, 평행곡선 공식, 조향 공식은 없다.

평가본 `/home/xytron/clrernet_eval`은 가중치와 런타임 확인 출처일 뿐 최종 개발
위치가 아니다. 최종 위치는
`/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_lane_drive`다.
