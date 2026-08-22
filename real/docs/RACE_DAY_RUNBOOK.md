# 본선 런북 — 재부팅 후 상태와 대시보드 없는 실주행

- 작성: 2026-08-22, 함종하 팀 (전 항목 실측 근거)
- 사본: 저장소 `real/docs/RACE_DAY_RUNBOOK.md` ↔ 차량 `~/RACE_DAY_RUNBOOK.md`

---

## 1. 컴퓨터(Jetson)가 꺼졌다 켜져도 유지되는가

| 항목 | 재부팅 후 | 근거 |
|---|---|---|
| **VESC 설정 전부** (FOC 전환, 실측 R/L/자속, 속도 PID kp 0.004, 전류 45/-20A, 22500 ERPM, 기동 lock/ramp) | **유지 ✅** | VESC 자체 플래시에 저장. 차 배터리를 뽑아도 남는다 |
| 코드·빌드·git 커밋 (race_last 포함) | 유지 ✅ | 디스크 |
| 가속 한계 4.0 m/s² | **유지 ✅** | `motor.launch.py` 기본값이 4.0 (install 이 소스로 심링크된 것 확인). 다음 launch 부터 자동 적용 |
| `kobot-sensors.service` (구 스택 + 대시보드) | **부팅 시 자동 기동 ⚠️** | `systemctl is-enabled` = enabled. 부팅 후 약 30초면 카메라·yolop·신호등·차량인식·대시보드가 저절로 떠 있다 |
| `ros2 param set` 으로 바꾼 런타임 값 (armed 등) | **소실 ❌** | 노드 재시작 시 초기화. 영구화하려면 소스/yaml 수정 + 커밋 |
| race_last / 모터 스택 | 자동 시작 안 됨 — 수동 | systemd 유닛 없음 (의도된 안전 설계) |

**재부팅 후 VESC 가 그대로인지 30초 확인법** (모터 스택 내려간 상태에서):
```bash
flatpak run com.vesc_project.VescTool --offscreen --vescPort /dev/ttyACM1 --getMcConf ~/vesc_readback/boot_check.xml && grep -oE "<(motor_type|s_pid_kp|l_in_current_max)>[^<]*</" ~/vesc_readback/boot_check.xml
# 기대: motor_type 2 / kp 0.004 / 45
# 주의: /tmp 는 flatpak 샌드박스라 밖에서 안 보인다 — 반드시 홈 경로로
```

---

## 2. 부팅 직후 실제로 벌어지는 일

전원 ON → 약 30초 → **구 스택이 저절로 떠 있다** (sensors.launch.py: 카메라 + yolop_drive + 신호등 + 차량인식 + 대시보드 8000). 모터 스택은 안 떠 있다.

- **디버깅할 때**: 이 상태 그대로 대시보드(`http://kobot:8000`) 쓰면 된다.
- **본선(race_last)으로 갈 때**: 구 스택을 먼저 내려야 한다 (§3의 1번). 안 내리면 카메라 장치·노드 이름이 충돌한다.

---

## 3. 본선 절차 — 대시보드 없이 (터미널만)

> 모든 터미널에서 첫 줄은 항상 `source ~/kobot_env.sh`

```
0. (안전) 차를 스탠드에 → 첫 기동 확인은 바퀴 띄우고

1. 구 스택 내리기 (부팅 시 자동으로 떠 있으므로)
   sudo systemctl stop kobot-sensors.service

2. [터미널 A] 인식 + 상황판 — 이 화면을 심판에게 보인다
   source ~/kobot_env.sh && ros2 launch race_last race.launch.py
   → "RACE 부팅" 전 항목 완료 확인
   → 이후 신호/차량(좌·우)/콘 상황판이 계속 갱신된다

3. [터미널 B] 모터
   source ~/kobot_env.sh && ros2 launch kobot_vehicle motor.launch.py armed:=true

4. 출발 (터미널 B 또는 새 창)
   ~/race_go.sh          # 기본 2.0 m/s (숫자로 덮어쓰기 가능)        # 원하는 상한 속도 m/s
   → 이후는 노드가 알아서: 빨간불 정지, 직진 신호, +1초 뒤 콘 인식,
     콘 구간 진입/이탈, 차량 회피

5. 정지 / 비상
   ~/race_halt.sh          # 무장 해제 + 모터 0 명령 3발. 몇 번 눌러도 안전
   (물리 비상: 배터리 커넥터 분리)
```

### race_go.sh 가 하는 일 (대시보드의 '기록 주행 시작'을 대체)
`enabled` 는 토픽, `armed` 는 파라미터라 **둘 다** 걸어야 움직인다
(`yolop_drive_node.py:671` — enabled 없으면 속도가 0). 대시보드가 하던
이 두 가지를 스크립트가 걸고, `/kobot_yolop/status` 로 실제 반영을 재확인한다
(`pub --once` 는 디스커버리 전이면 유실된다 — 0821 cone_once 실측 교훈).

### 디버깅으로 돌아가려면
```bash
~/race_halt.sh
# race_last(터미널 A)와 모터(터미널 B) Ctrl+C
sudo systemctl start kobot-sensors.service     # 구 스택 + 대시보드 복귀
```

---

## 4. 본선 중 금지 목록

| 금지 | 이유 |
|---|---|
| race_last 주행 중 대시보드 병행 사용 | 대시보드 '차선주행' 버튼은 **구 kobot_yolop_drive 를 하나 더 띄운다** — 모터 이중 발행 |
| `~/vesc_readback/detect_foc.py.done` 재실행 | detection 을 다시 돌리면 전류 한계를 99/-60 으로 또 덮어쓴다 (0822 실제 발생) |
| `ros2 run kobot_vehicle yolop_drive`, `sensors.launch.py.friend_20260821` 실행 | 갈라진 구버전 계보가 떠서 모터를 잡는다 |
| 주행 중 `ros2 topic pub /xycar_motor ...` 수동 발행 | race_halt.sh 를 통해서만 |

---

## 5. 미해결 — 배터리 (본선 전 반드시)

2026-08-22 실측: 부하 7A 에 전압 9.2→**6.1V 폭락** (내부저항 환산 ~400mΩ,
정상의 5~10배) → 7.5V 컷오프 → DRV 폴트(빨간불) → 차가 기어감.
**"모터가 느리다"의 최종 원인.** VESC·코드 문제 아님 (보호 정상 동작).

- [ ] 다른 팩으로 교체 시험 → 전압 버티면 이 팩 폐기 (죽은 셀)
- [ ] 같은 증상이면 커넥터/배선 저항 (주행 직후 커넥터 발열 확인)
- [ ] 본선용 팩 확보 + 완충, 예비 1개
