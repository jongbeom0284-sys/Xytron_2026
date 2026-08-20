#!/bin/bash
# 센서 스택만 정리하고 다시 올린다.
#
# 모터 스택(motor.launch.py · vesc_driver · ackermann_to_vesc · motor_adapter)은
# 이름으로 명시 제외한다. armed 상태를 건드리지 않는다.
#
# 쓰는 법
#   /home/xytron/restart_sensors.sh            카메라 포함 전체
#   /home/xytron/restart_sensors.sh camera:=false   카메라·차선·신호등 빼고
set -u

source /opt/ros/humble/setup.bash
source /home/xytron/2026_Xytron/real/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1

# 차선 노드(YOLOPv2)가 TensorRT 버퍼 할당에 torch 를 쓴다. 추론은 TensorRT 가 한다.
# 폴더 이름이 twinmixing 일 뿐이라 torch 심볼릭 링크를 쓴다.
export KOBOT_TORCH_SITE=/home/xytron/.venvs/torch/lib/python3.10/site-packages
export PYTHONPATH="$KOBOT_TORCH_SITE${PYTHONPATH:+:$PYTHONPATH}"

LOG=/home/xytron/.local/state/kobot/sensors-stack.log
NODES="dashboard_server traffic_perception yolop_drive status_node imu_node
       ultrasonic_node usb_cam xycar_lidar web_video_server rosbridge_websocket"

echo "── 모터 스택 확인 (건드리지 않는다)"
pgrep -af 'motor.launch.py|vesc_driver|ackermann_to_vesc|motor_adapter' \
  | grep -v pgrep | sed 's/^/   지킴  /' || echo "   모터 스택 없음"

echo "── sensors.launch.py 런처 종료"
for pid in $(pgrep -f 'ros2 launch .*sensors\.launch\.py' || true); do
  pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
  [ -n "$pgid" ] && kill -TERM -"$pgid" 2>/dev/null
done
sleep 3

echo "── 남은 센서 노드 정리"
for name in $NODES; do pkill -TERM -f "$name" 2>/dev/null; done
sleep 3
for name in $NODES; do pkill -KILL -f "$name" 2>/dev/null; done
sleep 1

left=$(pgrep -f 'usb_cam|xycar_lidar|web_video_server|rosbridge_websocket|yolop_drive|traffic_perception|dashboard_server' | tr '\n' ' ')
echo "   남은 프로세스: ${left:-없음}"
for port in 8000 8080 9090; do
  if ss -tln 2>/dev/null | grep -q ":$port "; then
    echo "   경고: $port 아직 점유 중"
  else
    echo "   $port 비었음"
  fi
done

echo "── sensors.launch.py 기동  $*"
setsid nohup ros2 launch kobot_vehicle sensors.launch.py "$@" \
  > "$LOG" 2>&1 < /dev/null &
sleep 12

echo "── 결과"
for name in $NODES; do
  printf '   %-20s %s\n' "$name" "$(pgrep -f "$name" | tr '\n' ' ' || echo '없음')"
done
for port in 8000 8080 9090; do
  ss -tln 2>/dev/null | grep -q ":$port " \
    && echo "   $port LISTEN" || echo "   $port 안 열림"
done
echo "   로그: $LOG"
