# Simulation

Unity 시뮬레이터에서만 사용하는 코드와 자산이다.

```text
simulation/
├── ros2_ws/src/
│   ├── ROS-TCP-Endpoint-main-ros2/
│   ├── kookmin9_viewer/
│   ├── track_drive/
│   └── xycar_msgs/
├── assets/
├── legacy/
└── reference/
```

```bash
cd simulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`track_drive`는 `/usb_cam/image_raw/front`, Unity용 `xycar_msgs/XycarMotor`, ROS-TCP endpoint를 전제로 한다. 실제 차량/NVIDIA 작업에는 사용하지 않는다.
