#!/bin/bash
# ☀️ 아침 원터치 점검 — 차 켜고 이거 더블클릭하면 배포·빌드·테스트·검증 끝.
#    (8/17 밤 작업분을 차에 올리고, 실차에서 전부 통과하는지 확인한다)
set -u
CAR="xytron@100.95.40.26"; KEY="$HOME/.ssh/kmu_car"
PROJ="/Users/hanhyemin/Desktop/국민대자율주행경진대회"
SSH="ssh -o ConnectTimeout=8 -o BatchMode=yes -i $KEY $CAR"
ok(){ printf "  ✅ %s\n" "$1"; }
bad(){ printf "  ❌ %s\n" "$1"; FAIL=1; }
FAIL=0

clear
echo "════════════════════════════════════════════════"
echo "  ☀️  아침 점검 — 8/17 밤 작업분 배포 및 검증"
echo "════════════════════════════════════════════════"

echo; echo "① 차량 연결"
if $SSH "echo up" >/dev/null 2>&1; then
  ok "kobot 연결 ($($SSH 'uptime | cut -d, -f1'))"
else
  bad "연결 실패 — 젯슨 전원과 Tailscale 확인"; exit 1
fi

echo; echo "② 코드 배포"
if bash "$PROJ/deploy.sh" >/dev/null 2>&1; then ok "deploy.sh"; else bad "배포 실패"; fi

echo; echo "③ 차에서 빌드"
if $SSH "source ~/kobot_env.sh && cd ~/xycar_ws && colcon build --packages-select kmu_race --symlink-install" >/dev/null 2>&1; then
  ok "colcon build"
else
  bad "빌드 실패 — 아래 명령으로 원인 확인"
  echo "     ssh -i $KEY $CAR 'source ~/kobot_env.sh && cd ~/xycar_ws && colcon build --packages-select kmu_race'"
fi

echo; echo "④ 차에서 테스트 (137개)"
T=$($SSH "source ~/kobot_env.sh && source ~/xycar_ws/install/setup.bash && cd ~/xycar_ws/src/kmu_race && python3 -m pytest test/ -q 2>&1 | tail -1")
echo "     $T"
case "$T" in *failed*) bad "테스트 실패";; *passed*) ok "테스트 통과";; *) bad "테스트 결과 불명";; esac

echo; echo "⑤ 차선 시뮬레이터 (코너링 회귀)"
S=$($SSH "source ~/kobot_env.sh && source ~/xycar_ws/install/setup.bash && cd ~/xycar_ws/src/kmu_race && timeout 900 python3 -m kmu_race.tools.lanesim --speeds 0.6,1.6 2>&1 | tail -1")
echo "     $S"
case "$S" in *"전 시나리오 통과"*) ok "코너링 시뮬";; *) bad "시뮬 실패 — 코너링 회귀 발생";; esac

echo; echo "⑥ 센서"
for t in "/image_raw 카메라" "/scan 라이다" "/imu IMU"; do
  set -- $t
  R=$($SSH "source ~/kobot_env.sh && source ~/xycar_ws/install/setup.bash && timeout 8 ros2 topic hz $1 2>/dev/null | grep -m1 average")
  if [ -n "$R" ]; then ok "$2 ${R#*: }"; else bad "$2 무신호"; fi
done

echo; echo "⑦ 모터"
if $SSH "ls /dev/ttyMOTOR" >/dev/null 2>&1; then
  ok "ttyMOTOR (전원 ON)"
  if $SSH "pgrep -f '[v]esc_driver_node'" >/dev/null 2>&1; then
    ok "VESC 무장됨"
  else
    echo "  ⚠️  미무장 — 지금 무장합니다"
    $SSH "~/arm_motor.sh" 2>&1 | grep -E "텔레메트리|명령 체인|완료" | sed 's/^/     /'
  fi
else
  bad "ttyMOTOR 없음 — 모터 배터리 스위치를 켜세요"
fi

echo; echo "⑧ 노출 제어 (신호등 판독용, 밤에 신설)"
if $SSH "which v4l2-ctl" >/dev/null 2>&1; then
  ok "v4l2-ctl 사용 가능 — 대기 중 자동으로 노출을 낮춥니다"
else
  echo "  ⚠️  v4l2-ctl 없음 — 노출 제어는 건너뛰고 위치 워처만 사용"
  echo "     설치하려면: sudo apt install v4l-utils"
fi

echo; echo "════════════════════════════════════════════════"
if [ "$FAIL" = "0" ]; then
  echo "  ✅ 전부 통과 — 주행 준비 완료"
  echo
  echo "  다음 순서:"
  echo "   1) 비상정지.command 더블클릭 (ENTER = 즉시 정지)"
  echo "   2) 클로드에게 '자율 시작' → 신호등 빨강 → 초록"
else
  echo "  ❌ 실패 항목이 있습니다 — 클로드에게 이 화면을 알려주세요"
fi
echo "════════════════════════════════════════════════"
echo; read -p "  창을 닫으려면 ENTER " _
