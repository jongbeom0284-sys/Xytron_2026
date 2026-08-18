#!/bin/bash
# ⏺ 자율주행 비상정지 버튼 v2 — ENTER 를 누르는 순간 새 ssh 로 발사한다.
#    (v1 은 미리 연결해 두는 방식이라 차 재부팅/네트워크 순단 뒤 죽은 채로
#     화면에 남아 있었다 — 0817 실사고. v2 는 누를 때마다 새로 연결하므로
#     차가 살아만 있으면 무조건 발사된다. 발사 실패 시 빨간 경고로 알린다.)
CAR="xytron@100.95.40.26"; KEY="$HOME/.ssh/kmu_car"
clear
echo "╔══════════════════════════════════════════════╗"
echo "║        ⏺  자율주행 비상정지 버튼  ⏺          ║"
echo "║                                              ║"
echo "║   위험하다 싶으면  ⏎ ENTER  를 누르세요     ║"
echo "║   (누르는 순간 발사 — 몇 번이고 다시 가능)   ║"
echo "╚══════════════════════════════════════════════╝"
while true; do
  read -p "  [ENTER = 즉시 정지] " _
  echo "  🚀 발사 중..."
  # 1) 최우선: race_node 즉사 (짧은 타임아웃, 재시도 2회)
  ok=0
  for i in 1 2; do
    if ssh -o ConnectTimeout=3 -o BatchMode=yes -i "$KEY" "$CAR" \
        'pkill -9 -f "kmu_race[ ]race"; pkill -9 -f "kmu_race/lib/kmu_race/rac[e]"; source ~/kobot_env.sh; source ~/xycar_ws/install/setup.bash; timeout 4 ros2 topic pub --times 10 -r 25 /xycar_motor std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0]}" >/dev/null 2>&1; echo OK' 2>/dev/null | grep -q OK; then
      ok=1; break
    fi
  done
  if [ $ok -eq 1 ]; then
    echo "  🛑 정지 완료 (race_node 종료 + 정지 10발). 다시 대기합니다."
  else
    echo ""
    echo "  ❌❌❌ 차에 연결 실패 — 네트워크/전원 확인! 차를 손으로 들어올리세요 ❌❌❌"
    echo ""
  fi
done
