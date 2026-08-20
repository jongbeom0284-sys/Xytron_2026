#!/bin/bash
# 기록 주행 테스트용 전체 기동.
#
#   1. 돌고 있는 것 전부 정리 (모터 포함)
#   2. 센서 스택   카메라 · 차선(YOLOPv2) · 신호등 · 라이다 · IMU · 초음파
#                  + 8000 대시보드 · 8080 영상 · 9090 rosbridge
#   3. 모터 스택   armed:=true
#   4. 상태 출력
#
# 노드를 새로 띄우므로 고친 코드가 전부 반영된다.
# 그 뒤 대시보드(8000)에서 기록 주행 버튼을 누르면 된다.
#
# 패턴을 스크립트 안에 두는 이유: 명령줄에 두면 pkill -f 가 자기 자신을 죽인다.
set +u

source /opt/ros/humble/setup.bash
source /home/xytron/2026_Xytron/real/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1

# 차선 노드(YOLOPv2)가 TensorRT 버퍼 할당에 torch 를 쓴다. 추론은 TensorRT 가 한다.
# dashboard_server 가 차선 노드를 자식으로 띄우므로 여기서 넣어야 물려받는다.
export KOBOT_TORCH_SITE=/home/xytron/.venvs/torch/lib/python3.10/site-packages
export PYTHONPATH="$KOBOT_TORCH_SITE${PYTHONPATH:+:$PYTHONPATH}"

STATE=/home/xytron/.local/state/kobot
SELF=$$
PATTERNS=(
  'sensors\.launch\.py' 'motor\.launch\.py' 'yolop_arm\.sh'
  'dashboard_server' 'traffic_perception' 'yolop_drive' 'status_node'
  'imu_node' 'ultrasonic_node' 'usb_cam' 'xycar_lidar'
  'web_video_server' 'rosbridge' 'vesc_driver' 'ackermann_to_vesc'
  'motor_adapter'
)

reap() {
  for pat in "${PATTERNS[@]}"; do
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
      [ "$pid" = "$SELF" ] && continue
      [ "$pid" = "$PPID" ] && continue
      kill "-$1" "$pid" 2>/dev/null
    done
  done
}

echo "── 1. 전부 정리"
reap TERM; sleep 4; reap KILL; sleep 2
for port in 8000 8080 9090; do
  ss -tln 2>/dev/null | grep -q ":$port " && echo "   경고: $port 아직 점유" \
                                          || echo "   $port 비었음"
done

echo "── 2. 센서 스택 기동"
setsid nohup ros2 launch kobot_vehicle sensors.launch.py \
  > "$STATE/sensors-stack.log" 2>&1 < /dev/null &
sleep 18

echo "── 3. 모터 스택 기동 (armed)"
setsid nohup ros2 launch kobot_vehicle motor.launch.py armed:=true \
  > "$STATE/motor-stack.log" 2>&1 < /dev/null &
sleep 8

echo "── 4. 상태"
for name in usb_cam yolop_drive traffic_perception web_video_server \
            rosbridge_websocket dashboard_server vesc_driver motor_adapter; do
  printf '   %-20s %s\n' "$name" "$(pgrep -f "$name" | tr '\n' ' ')"
done
for port in 8000 8080 9090; do
  ss -tln 2>/dev/null | grep -q ":$port " && echo "   $port LISTEN" \
                                          || echo "   $port 안 열림"
done

echo "── 5. 토픽"
timeout 12 ros2 topic list 2>/dev/null \
  | grep -E 'traffic_light|kobot_yolop|xycar_motor|image_raw$' | sed 's/^/   /'

echo "── 6. 차선 백엔드"
grep -m1 -iE 'TensorRT 적재|TorchScript' "$STATE/sensors-stack.log" \
  | sed 's/^/   /' || echo "   아직 적재 중"

echo
echo "대시보드 http://10.103.75.200:8000/index.html  (처음 한 번 Ctrl+Shift+R)"
echo "거기서 기록 주행 시작을 누르면 된다."
echo "로그  센서 $STATE/sensors-stack.log   모터 $STATE/motor-stack.log"
