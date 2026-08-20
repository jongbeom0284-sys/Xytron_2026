# 라바콘 활성 구간 전용 기록

이 도구의 목적은 `ros2 launch` 시작 시간이 주행 시간에 포함되던 문제를 없애고,
차량이 실제로 enable 된 구간 전체를 하나의 rosbag과 JSONL 로그로 남기는 것이다.

## 실행 전에 사람이 준비할 것

- 센서 스택과 모터 스택은 기존의 검증된 런치로 먼저 실행한다.
- 차량을 출발점에 놓고 진행 방향과 비상 정지 공간을 확인한다.
- 대시보드·YOLOP·이전 주행 노드처럼 `/xycar_motor` 또는
  `/kobot_lane_drive/record_drive/enabled`를 발행하는 프로세스는 팀의 기존 종료
  절차로 먼저 내린다.
- 이 기록 도구는 충돌 프로세스를 **자동 종료하지 않는다**. 충돌 publisher가
  남아 있으면 이름을 출력하고 출발 전에 중단한다.
- 출발 후에도 enable/모터/status/preview publisher의 노드명과 DDS endpoint GID를
  반복 확인한다. recorder+planner+bridge 이외의 endpoint가 새로 생기거나 기존
  endpoint가 교체되면 즉시 `enable=false`로 전환하고 테스트를 실패 처리한다.
- bridge가 enable 토픽을 받는 subscriber GID도 준비 단계에서 고정한다. rosbag이
  나중에 같은 토픽 subscriber를 추가하는 것은 허용하지만, bridge 자신의 endpoint가
  사라지거나 교체되면 실패 처리한다.
- 하네스 launch는 브리지의 enable heartbeat lease를 `0.30초`로 켠다. 하네스가
  `SIGKILL`되거나 SSH가 끊겨 cleanup 자체가 실행되지 않아도 true heartbeat가
  끊긴 뒤 브리지가 0을 발행한다. 일반 launch의 기본값은 `0.0`이라 기존
  대시보드의 명시적 true/false 방식은 바뀌지 않는다.

## 15초 주행 기록

Jetson의 테스트 소스 디렉터리에서 다음 한 줄을 실행한다.

```bash
bash ./scripts/cone_active_run.sh --duration 15 --speed 0.55
```

스크립트가 `READY`를 출력해도 차량은 `enable=false` 상태다. Enter를 누른 뒤 기본
3초 카운트다운이 끝나면 정확히 15초 동안만 `enable=true`가 되고, 끝나는 즉시
같은 ROS publisher가 `false`를 연속 발행한다. 어느 때든 `Ctrl-C`를 누르면 같은
정지 절차를 먼저 수행한다.

자동 시작이 꼭 필요할 때만 다음처럼 사용한다.

```bash
bash ./scripts/cone_active_run.sh \
  --duration 15 \
  --speed 0.55 \
  --countdown 3 \
  --no-prompt
```

`timeout 15s ros2 launch ...`는 사용하지 않는다. 그 방식은 DDS discovery와 노드
시작 시간이 15초에 포함되어 실제 활성 주행이 몇 초였는지 알 수 없다.

## 실행 순서 보장

도구 내부 순서는 다음과 같다.

1. 기존 command publisher 충돌 확인 — 발견 시 아무것도 종료하지 않고 중단
2. transient-local enable publisher 생성 및 `false` 연속 발행
3. `armed:=true auto_enable:=false`로 라바콘 planner/bridge만 실행
4. launch 시작 중에도 recorder/planner/bridge 외 endpoint를 거부하며, 허용
   endpoint가 처음 보이는 순간 DDS GID를 고정
5. 두 노드, 최신 status/preview, bridge-enable 연결과 기본 연속 `TRACKING`
   0.5초를 확인한 후 recorder enable 1개·planner enable 1개·bridge motor 1개·
   planner status 1개·planner preview 1개의 DDS GID 허용목록 고정
6. rosbag 시작 후 여섯 토픽 모두에 같은 rosbag 노드의 subscriber endpoint가
   새로 생겼는지 graph handshake로 확인하고 출력 파일 생성도 확인
7. Enter·카운트다운 후 `true` heartbeat 발행과 동시에 별도 활성 타이머 시작
8. 활성 직전과 활성 중 command endpoint 허용목록을 계속 재검증
9. 지정 시간이 끝나면 `false`를 발행하고 실제 `/xycar_motor`의 `[*, 0]` 메시지를
   제한 시간 안에 확인한 뒤에만 자체 launch와 bag 종료

센서, VESC driver, motor adapter, dashboard 서비스에 대해 `pkill`, `killall`,
restart를 수행하는 코드는 없다. 종료 신호는 이 도구가 직접 만든 라바콘 launch와
rosbag 프로세스 그룹에만 보낸다. ROS CLI 부모가 먼저 죽어도 기록해 둔 자체 PGID에
`SIGINT → SIGTERM → SIGKILL` 순서로 정리하며, 정리 도중 다시 `Ctrl-C`가 들어와도
false/0 확인과 두 자체 그룹 정리를 건너뛰지 않는다.

정지 확인 제한 시간은 기본 2초다. 필요하면 다음처럼 바꿀 수 있다.

```bash
bash ./scripts/cone_active_run.sh \
  --duration 15 \
  --speed 0.55 \
  --stop-timeout 2.5
```

제한 시간 안에 실제 0 메시지가 수신되지 않으면 결과는 실패이며 종료 코드는 0이
아니다. 이 경우에도 자체 프로세스 그룹 정리는 계속 수행한다.

## 결과 파일

기본 저장 위치는 `~/cone_runs/cone-active-YYYYmmdd-HHMMSS/`다.

| 파일 | 내용 |
|---|---|
| `bag/` | `/scan`, status, preview, enable, `/xycar_motor`, `/sensors/core` 전체 |
| `events.jsonl` | false/ready/bag/true/false/첫 nonzero/정지 0의 정확한 시각 |
| `status.jsonl` | planner 상태 JSON과 수신 wall/monotonic/ROS 시각 |
| `preview.jsonl` | 조향·속도·목표점 프리뷰 |
| `motor.jsonl` | bridge가 실제 `/xycar_motor`에 발행한 값 |
| `launch.log` | 해당 테스트 라바콘 launch 로그 |
| `bag.log` | rosbag recorder 로그 |
| `summary.json` | 실제 enable 시간, 첫 구동 지연, 정지 지연, 마지막 상태 요약 |

`summary.json`에서 우선 확인할 값은 다음 세 개다.

- `active_enable_duration_sec`: 시작 준비와 무관한 실제 enable 구간
- `first_nonzero_motor_after_enable_sec`: enable 후 첫 실제 구동 명령까지 지연
- `zero_after_stop_delay_sec`: false 전환 후 첫 0 명령까지 지연
- `motor_zero_confirmed_before_child_shutdown`: 자식 종료 전에 실제 0을 확인했는지
- `bag_graph_subscription_handshake`: rosbag이 여섯 토픽 모두와 graph 연결됐는지

라바콘이 아직 안 보이는 상태도 기록만 해야 한다면
`--allow-not-tracking`을 추가할 수 있다. 실차 주행에서는 기본값을 유지해
`found=true`가 된 뒤에만 출발하는 편이 데이터 구간을 명확히 만든다.
기본 모드는 단일 `TRACKING` 프레임으로 출발하지 않고, status 간격이 0.25초를
넘지 않는 연속 `TRACKING + found=true + motor_output=false` 0.5초를 요구한다.
중간에 `HOLD`, `found=false`, `motor_output=true`가 한 번이라도 나오거나 status가
끊기면 0.5초 측정을 처음부터 다시 한다.

현재 하네스는 `/sensors/core`를 bag에 포함하지만 VESC `fault_code`의 의미를 해석해
주행을 자동 중단하지는 않는다. VESC fault 기반 fail-close는 별도 검증 후 추가할
범위이며, 이번 하네스가 보장하는 것은 command endpoint 단독 점유, 활성 구간 기록,
그리고 false 이후 실제 모터 0 명령 확인이다.
