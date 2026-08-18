#!/usr/bin/env bash
# kmu_race 를 자이카(kobot)로 배포 → 빌드 → 프리플라이트까지 한 번에.
#
#   ./deploy.sh                  # Tailscale kobot 으로 배포
#   ./deploy.sh 192.168.0.51     # 다른 주소를 쓸 때 직접 지정
#   ./deploy.sh --preflight      # 배포 후 프리플라이트까지
#
# 2026-08 하드웨어 이행 이후의 전제 (자세한 근거: 차의 ~/xycar_ws/Jetson_*.md):
#   · 차 = Jetson Orin NX, 호스트명 kobot, Tailscale 100.95.40.26
#   · 자이카 자체 AP(10.42.0.1) 구조는 폐기됨 — KMU_WiFi + Tailscale 만 사용
#   · ROS 환경은 차의 ~/kobot_env.sh 가 원장 (ROS_DOMAIN_ID=7, LOCALHOST_ONLY=1)
#     → 빌드/실행 셸은 반드시 이걸 소싱해야 토픽이 보인다
#   · 모터는 잠금 상태가 기본. 센서만 뜬다 (kobot-sensors.service)
#
# 맥에는 ROS2 가 없으므로 빌드·실행은 전부 차 위에서 일어난다.
# 이 스크립트는 rsync 로 소스만 올리고 차에서 colcon build 를 돌린다.

set -uo pipefail

CAR_USER="${CAR_USER:-xytron}"
CAR_IP_DEFAULT="100.95.40.26"          # Tailscale kobot
SSH_KEY="${SSH_KEY:-$HOME/.ssh/kmu_car}"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/xycar_ws/src/kmu_race"
REMOTE_WS="${REMOTE_WS:-~/xycar_ws}"

CAR_IP=""
DO_PREFLIGHT=false
for a in "$@"; do
  case "$a" in
    --preflight) DO_PREFLIGHT=true ;;
    *) CAR_IP="$a" ;;
  esac
done
[ -z "$CAR_IP" ] && CAR_IP="$CAR_IP_DEFAULT"

SSH=(ssh -i "$SSH_KEY" -o ConnectTimeout=8 "$CAR_USER@$CAR_IP")

echo "════════════════════════════════════════════"
echo " 대상 : $CAR_USER@$CAR_IP:$REMOTE_WS  (Tailscale kobot)"
echo " 소스 : $PKG_DIR"
echo "════════════════════════════════════════════"

# ── 1. 연결 확인 ─────────────────────────────────────────────────
echo
echo "[1/3] 연결 확인 + 소스 전송"
"${SSH[@]}" "mkdir -p $REMOTE_WS/src" || {
  cat >&2 <<EOS
❌ SSH 실패: $CAR_USER@$CAR_IP

  1) 차 전원과 Tailscale 상태 확인:  tailscale status | grep kobot
  2) 키 등록 여부 확인:  ssh -i $SSH_KEY $CAR_USER@$CAR_IP
  3) 학교망 직통 주소로 시도:  ./deploy.sh 10.103.75.200
EOS
  exit 1
}

rsync -az --delete \
  -e "ssh -i $SSH_KEY" \
  --exclude '__pycache__' --exclude '.pytest_cache' --exclude '*.pyc' \
  "$PKG_DIR" "$CAR_USER@$CAR_IP:$REMOTE_WS/src/" || { echo "❌ 전송 실패"; exit 1; }
echo "  ✅ 전송 완료"

# ── 2. 빌드 ──────────────────────────────────────────────────────
# kobot_env.sh 를 소싱해야 install 체인(kobot_vehicle 등)과 도메인이 맞는다.
echo
echo "[2/3] 차에서 빌드"
"${SSH[@]}" bash -lc "'
  source ~/kobot_env.sh
  cd $REMOTE_WS
  colcon build --symlink-install --packages-select kmu_race 2>&1 | tail -15
'" || { echo "❌ 빌드 실패"; exit 1; }
echo "  ✅ 빌드 완료"

# ── 3. 프리플라이트 ──────────────────────────────────────────────
if $DO_PREFLIGHT; then
  echo
  echo "[3/3] 프리플라이트 (차는 움직이지 않음)"
  echo "  차를 출발 위치(신호등 보이게)에 두고, 정면 1m 앞에 상자를 놓으면 실측까지 나온다"
  "${SSH[@]}" -t bash -lc "'
    source ~/kobot_env.sh
    source $REMOTE_WS/install/setup.bash
    ros2 run kmu_race preflight --duration 15
  '"
else
  cat <<EOS

[3/3] 다음 단계 — 차에 접속해서:

  ssh -i $SSH_KEY $CAR_USER@$CAR_IP
  source ~/kobot_env.sh && source ~/xycar_ws/install/setup.bash

  # ① 인터페이스 점검 (바퀴 안 움직임)
  ros2 run kmu_race preflight --duration 15

  # ② 모터 무장 (바퀴 띄우고, 팀 승인 후에만!)
  ros2 launch kobot_vehicle motor.launch.py armed:=true

  # ③ 수동주행 3바퀴 녹화 — 조종은 대시보드 http://kobot:8000 WASD
  #    (raw 는 4GB+ 라 compressed 로 찍는다. /imu 빼먹으면 요 튜닝 불가)
  ros2 bag record /image_raw/compressed /scan /imu -o run_\$(date +%m%d_%H%M)

  # ④ 맥으로 bag 가져오기 (맥에서)
  scp -i $SSH_KEY -r $CAR_USER@$CAR_IP:~/run_* ~/kmu_bags/
EOS
fi
