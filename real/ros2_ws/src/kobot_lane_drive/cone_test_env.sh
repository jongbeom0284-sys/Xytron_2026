#!/usr/bin/env bash
# 라바콘 실험 워크스페이스가 메인 kobot_lane_drive에 가려지지 않도록
# ROS overlay 관련 변수만 초기화한 뒤 stable test overlay를 올린다.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset COLCON_CURRENT_PREFIX
unset PYTHONPATH

source /opt/ros/humble/setup.bash
source /home/xytron/Xytron_2026_stable_test/real/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=1
