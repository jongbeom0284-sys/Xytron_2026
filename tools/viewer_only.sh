#!/bin/bash
# 전부 내리고 뷰어에 필요한 것만 올린다.
#   카메라 + 차선(YOLOPv2) + 신호등 + 8000 대시보드 + 8080 영상 + 9090 rosbridge
#   라이다 · IMU · 초음파 · 모터는 올리지 않는다.
#
# 패턴을 스크립트 안에 두는 이유: 명령줄에 두면 pkill -f 가 자기 자신을 죽인다.
set +u

source /opt/ros/humble/setup.bash
source /home/xytron/2026_Xytron/real/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1
export KOBOT_TORCH_SITE=/home/xytron/.venvs/torch/lib/python3.10/site-packages
export PYTHONPATH="$KOBOT_TORCH_SITE${PYTHONPATH:+:$PYTHONPATH}"

LOG=/home/xytron/.local/state/kobot/viewer-stack.log
SELF=$$

PATTERNS=(
  'sensors\.launch\.py' 'motor\.launch\.py'
  'dashboard_server' 'traffic_perception' 'yolop_drive' 'status_node'
  'imu_node' 'ultrasonic_node' 'usb_cam' 'xycar_lidar'
  'web_video_server' 'rosbridge' 'vesc_driver' 'ackermann_to_vesc'
  'motor_adapter'
)

reap() {  # $1 = 시그널
  for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
      [ "$pid" = "$SELF" ] && continue
      [ "$pid" = "$PPID" ] && continue
      kill "-$1" "$pid" 2>/dev/null
    done
  done
}

echo "── 전부 내린다 (모터 포함)"
reap TERM
sleep 4
reap KILL
sleep 2

echo "── 남은 것"
left=''
for pat in "${PATTERNS[@]}"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$pid" = "$SELF" ] && continue
    [ "$pid" = "$PPID" ] && continue
    left="$left $pid"
  done
done
echo "   ${left:-없음}"
for port in 8000 8080 9090; do
  ss -tln 2>/dev/null | grep -q ":$port " && echo "   $port 아직 점유" || echo "   $port 비었음"
done

echo "── 뷰어 스택 기동 (라이다 · IMU · 초음파 제외)"
setsid nohup ros2 launch kobot_vehicle sensors.launch.py \
  lidar:=false imu:=false ultrasonic:=false \
  > "$LOG" 2>&1 < /dev/null &
sleep 15

echo "── 결과"
for name in usb_cam yolop_drive traffic_perception web_video_server \
            rosbridge_websocket dashboard_server; do
  printf '   %-20s %s\n' "$name" "$(pgrep -f "$name" | tr '\n' ' ')"
done
for port in 8000 8080 9090; do
  ss -tln 2>/dev/null | grep -q ":$port " && echo "   $port LISTEN" || echo "   $port 안 열림"
done
echo "   로그: $LOG"
