# Kobot camera calibration

`kobot_camera.yaml`은 실제 `/dev/videoCAM`을 ROS `camera_calibration`으로 보정한
결과만 둔다. 예제값이나 추정값을 복사하지 않는다.

현재 차량에서 읽은 `/camera_info`는 640x480이지만 `K`, `R`, `P`가 전부 0이고
카메라 TF도 없다. 이 상태에서 `kobot_lane_drive`는 의도적으로
`CALIBRATION_INVALID`를 내고 미터 거리와 조향값을 만들지 않는다.

필수 실측값:

- 640x480 카메라 내부 파라미터 파일 `kobot_camera.yaml`
- `base_link` 기준 카메라 장착 위치 x/y/z (m)
- 카메라 장착 roll/pitch/yaw (rad)

내부 보정은 ROS2 Humble의 공개 `camera_calibration` 패키지를 사용한다. 장착
자세는 수평 바닥에서 실측하고, `track_test.launch.py`의 `camera_*` 인자로 넣는다.
IPM 결과가 바닥 표식의 실측 거리와 일치하는지 확인하기 전에는 `armed:=true`를
사용하지 않는다.
