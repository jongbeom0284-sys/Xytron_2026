# lane_id_perception

AMD 차량 컴퓨터의 차선 인식 실험 코드에서 모터·시뮬레이터 의존성을 제거해 분리한 ROS2 패키지다.

주요 기능:

- 스캔라인 체인으로 좌·우 차선을 분리하고 `L#1`, `R#2` ID 유지
- 한쪽 차선만 보일 때도 좌·우 ID와 이전 차선 폭 기억
- BEV, 마스크, 전방 검출, ID 오버레이 디버그 영상 publish
- 카메라 영상만 처리하며 모터 명령은 publish하지 않음
- 이미지 폴더 기반 오프라인 재생 지원

## NVIDIA workspace로 가져오기

```bash
mkdir -p ~/lane_ws/src
cp -a real/lane_id_perception ~/lane_ws/src/
cd ~/lane_ws
colcon build --symlink-install --packages-select lane_id_perception
source install/setup.bash
ros2 launch lane_id_perception lane_id_preview.launch.py image_topic:=/image_raw
```

카메라 토픽이 다르면 마지막 인자만 실제 토픽으로 바꾼다. 이 launch는 인식 결과만 publish하고 차량을 움직이지 않는다.

## 테스트

```bash
cd real/lane_id_perception
python3 -m pytest -q test/test_lane_id.py
```
