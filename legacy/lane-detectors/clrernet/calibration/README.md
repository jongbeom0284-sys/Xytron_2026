# Kobot camera reference policy

현재 `kobot_lane_drive`는 같은 트랙, 고정 카메라, 고정 해상도/crop 조건에서
기준 사진의 픽셀 비율을 유지한다. 따라서 ROS 체커보드 보정, 카메라 K 행렬,
장착 TF, 미터 거리 환산은 차선 주행의 필수 입력이 아니다.

트랙 직선 구간에서 차량을 차선과 평행하게 맞춘 뒤, 차량의 좌측면이 좌측 차선에서
원하는 간격에 있도록 세우고 저장한 사진을 기준으로 사용한다.
`offline_replay --reference-image ...`가 near/far 두 행의
좌측 차선 x 비율을 `pixel_reference.json`으로 만든다. 사람이 원본과 overlay를
확인한 뒤 두 값만 `config/lane_drive.yaml`에 옮기고 `reference_ready: true`로
바꾼다.

다음 중 하나라도 바뀌면 이전 기준값을 사용하지 않는다.

- 카메라 위치 또는 각도
- 640x480 등 입력 해상도
- resize 전 crop이나 렌즈
- 차량 차체와 카메라의 상대 위치
- 따라갈 트랙 차선 폭 또는 운용 방향

미터 단위 위치가 필요한 다른 기능에서 실제 카메라 보정을 수행할 수는 있지만,
추정 K나 다른 카메라의 YAML을 이 픽셀 추종기에 넣지 않는다.
