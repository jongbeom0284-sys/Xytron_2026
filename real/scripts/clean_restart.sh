#!/bin/bash
# 완전 정지 → DDS 찌꺼기 삭제 → 재기동.
# 순서가 중요하다: 돌아가는 중에 /dev/shm/fastrtps_* 를 지우면 SHM 락 오류가
# 나고 복구가 더 어려워진다 (0823 실측). 반드시 멈춘 뒤에만 지운다.
set +u
echo "1) 주행 스택 정지"
echo xytron | sudo -S systemctl stop kobot-race.service 2>/dev/null
sleep 3

echo "2) 모터 스택 정지"
for p in $(ps -eo pid= -o args= | awk '/motor\.launch\.py|vesc_driver_node|ackermann_to_vesc|motor_adapter/ && !/awk/ {print $1}'); do
  kill "$p" 2>/dev/null
done
sleep 3

echo "3) 남은 ROS 프로세스 확인"
left=$(ps -eo pid= -o args= | awk '/race_last\/lib|usb_cam_node_exe|web_video_server|rosbridge_websocket|xycar_lidar_node/ && !/awk/ {print $1}')
if [ -n "$left" ]; then
  echo "   남은 것 정리: $left"
  for p in $left; do kill -9 "$p" 2>/dev/null; done
  sleep 2
fi

echo "4) DDS 공유메모리 정리"
n=$(ls /dev/shm/ 2>/dev/null | grep -c fastrtps_ || true)
rm -f /dev/shm/fastrtps_* 2>/dev/null
echo "   ${n}개 삭제"

echo "5) ROS 데몬 재시작"
. /opt/ros/humble/setup.sh
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=1
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1

echo "6) 주행 스택 시작"
echo xytron | sudo -S systemctl start kobot-race.service 2>/dev/null
sleep 14

echo "7) 모터 스택 시작"
. /home/xytron/kobot_env.sh >/dev/null 2>&1
setsid nohup ros2 launch kobot_vehicle motor.launch.py armed:=true \
  > /home/xytron/motor_launch.log 2>&1 < /dev/null &
sleep 10

echo "8) 확인"
curl -s -o /dev/null -w "   대시보드 %{http_code}\n" http://127.0.0.1:8000/
n=$(timeout 15 curl -s --max-time 12 "http://127.0.0.1:8080/snapshot?topic=/kobot_yolop/debug_image" 2>/dev/null | wc -c)
echo "   차선 영상 스냅샷 ${n}바이트"
