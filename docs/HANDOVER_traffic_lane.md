# 신호등 인식 + 신호 연동 주행 — 인수인계

**작성 2026-08-21 · 대회 2026-08-25(화) 국민대 자율주행 경진대회 본선**

이 문서가 담당하는 것은 **트랙 전반부**다. 신호등을 읽고, 그 신호에 따라 정지·출발하고,
삼거리에서 직진/좌회전을 가르는 부분이다. 후반부 구간 코드와 합쳐야 한다.

합칠 때 읽어야 할 곳은 **§7 토픽 계약** 과 **§9 충돌 지점** 두 곳이다.

---

## 1. 한 줄 요약

```
카메라 → [신호등 노드] → /traffic_light/state → [차선 노드] → /xycar_motor
              색도+원형성                          정지·직진 트리거
```

신호등은 **YOLO를 쓰지 않는다.** 정규화 색도와 원형성만으로 판정하며 numpy+OpenCV로
약 24ms(≈15Hz)다. 차선은 기존 YOLOPv2 TensorRT 그대로이고, 신호에 따라 노란선 전파
방식만 바꾼다.

---

## 2. 건드린 파일 (전부 `real/ros2_ws/src/` 아래)

| 파일 | 줄수 | 무엇을 했나 |
|---|---|---|
| `kobot_vehicle/kobot_vehicle/Traffic_Perception.py` | 333 | **전면 재작성.** YOLO 제거, 색도+원형성 방식 |
| `kobot_yolop_drive/kobot_yolop_drive/lane_core.py` | 477 | `grow_vertical` · `reclaim_white` 추가, `_split_blobs` 에 직진 가지 |
| `kobot_yolop_drive/kobot_yolop_drive/yolop_drive_node.py` | 746 | `/traffic_light/state` 구독, 확정·잠금 상태기계, 정지·조향억제 |
| `kobot_vehicle/launch/sensors.launch.py` | 104 | `traffic_perception` 노드 추가 (카메라 조건부) |
| `kobot_vehicle/launch/traffic.launch.py` | 17 | 단독 실행용. ultralytics venv 주입 제거 |
| `kobot_vehicle/dashboard/app.js` | 1123 | 차선 화면에 신호등 오버레이 + 신호 상태 카드 |
| `kobot_vehicle/dashboard/drive.css` | 381 | 위 카드 스타일 |
| `kobot_vehicle/dashboard/index.html` | — | 캐시 무력화 `?v=20260821a` |

백업은 전부 같은 폴더에 `*.bak_20260821` 로 있다.

> **함정 — `kobot_yolop_drive` 는 install 이 실제 복사본이다.**
> `kobot_vehicle` 은 egg-link(develop) 라 `src/` 수정이 즉시 반영되지만,
> `kobot_yolop_drive` 는 아니다. `src/` 만 고치면 **노드는 옛 코드를 계속 돈다.**
> 이걸 몰라서 한참 헤맸다. 반드시 둘 다 고치거나 colcon build 하라.
> ```
> install/kobot_yolop_drive/lib/python3.10/site-packages/kobot_yolop_drive/
> ```

---

## 3. 신호등 인식 원리

### 3.1 신호등 사양

실내 가로형 4구. 왼쪽부터 `[빨강] [노랑] [좌회전←] [초록]`.
램프 간격 = 램프 반지름의 **3.6배** (거리와 무관한 무차원 값).

| 상태 | 켜지는 램프 |
|---|---|
| 빨간불 | ① |
| 노란불 | ② |
| 초록불 | ④ |
| 좌회전 | **③ + ④ 동시** |

**초록불과 좌회전은 색이 완전히 같다.** 화살표가 초록원 왼쪽에 하나 더 켜지는지로만 갈린다.

### 3.2 왜 밝기·채도로 찾으면 안 되는가

실측 (60장, `/home/xytron/signal_shots/`):

| 대상 | Hue | 채도 S | 밝기 V | **정규화 청색 nb** |
|---|---|---|---|---|
| 빨강 LED | 12 | **76** | 229 | **0.280** |
| 주황 의자 | 16 | **186** | 195 | **0.137** |
| 천장 형광등 | — | 8 | 236 | 0.333 |

**의자가 빨강 LED보다 채도가 2.4배 높다.** 밝기·채도로 찾는 방법은 전부 의자에 진다.
LED는 과노출 블룸으로 흰빛이 섞여 **채도가 오히려 낮고 청색이 남는다**. 의자는 진짜
주황 확산면이라 청색이 거의 없다. 그래서 `nb`(정규화 청색)로 자르면 뒤집힌다.

### 3.3 판정 절차 (`find_lamps` → `decide`)

```
1. 3x3 가우시안 후 정규화 색도 nr,ng,nb = B,G,R / (B+G+R), 밝기 V = max(B,G,R)
2. 점등 마스크
     V > MIN_VALUE(150)          충분히 밝다
   & nb > MIN_BLUE(0.175)        LED 블룸의 청색. 주황 확산면 배제
   & 색편차 > MIN_SPREAD(0.045)  무채색 형광등 배제
   & ROI 안                      아래 3.4
3. 연결요소 → 원형 조건
     면적 >= MIN_AREA(6)
     가로세로비 <= MAX_ASPECT(1.8)     램프는 원이다
     채움률 >= MIN_FILL(0.33)          면적/외접사각형
   ※ 둘 다 무차원이라 거리가 변해도 그대로다
4. 덩어리 색 분류 (classify)
     nb <= 0.175                       → 램프 아님
     ng - nr > CYAN_MARGIN(0.05) & ng > 0.36  → cyan (초록·화살표)
     nr > 0.385 & ng >= 0.355          → amber (노랑)
     nr > 0.36 & ng < 0.355            → red
5. 상태 결정 (우선순위 cyan > amber > red)
     cyan 있음 → 초록원(가장 큰 것) 기준
        왼쪽에 같은 cyan 덩어리가 또 있으면      → left
        없으면 arrow_beside() 로 화살표 자리 직접 확인
           있으면 left, 없으면 green
     amber → yellow
     red   → red
     없음  → none
```

**`ng - nr` 로 청록을 판정하는 이유:** 절대 문턱값(`ng>0.385 and nr<0.30`)을 쓰면
거리가 멀어져 화살표가 흐려질 때 두 조건이 동시에 아슬아슬하게 탈락한다. 실측에서
먼 거리 화살표가 `ng 0.383 / nr 0.304` 로 둘 다 놓쳤다. 녹−적 **차이**는 흐려져도
남는다 (초록원 0.156, 화살표 0.079, 노랑 −0.053, 빨강 −0.069, 의자 −0.173).

**`arrow_beside()` 가 필요한 이유:** 화살표는 `←` 모양이라 **원이 아니다.** 원형 조건에
걸려 절반쯤 탈락한다. 그래서 덩어리로 못 찾으면 초록원 면적에서 반지름을 역산하고
`반지름 × 3.6` 만큼 왼쪽 자리를 직접 본다. 자리를 알고 보므로 밝기 문턱을 140으로 낮춘다.
좌표를 알려주는 게 아니라 **검출된 램프에서 재는 것**이라 거리 무관하다.

### 3.4 ROI

```python
ROI_TOP    = 0.373   # 179px
ROI_BOTTOM = 0.444   # 213px
ROI_LEFT   = 0.302   # 193px
ROI_RIGHT  = 0.528   # 338px
```

**자르지 않고 마스크만 끈다.** 좌표가 원본 그대로 유지되므로 발행되는 박스 좌표를
그대로 화면에 겹칠 수 있다.

근거: 신호등은 머리 위에 달려 있어 화면에서 지평선 아래로 내려오지 않는다. 라바콘·의자는
바닥에 있어 항상 지평선 아래다. 그 경계로 자른다.

> **이 값은 2026-08-21 04:30 주행 프레임에서 맞춘 것이다.**
> 8월 20일 촬영 60장(램프가 x 301~373, y 156~194에 있던)과는 거의 안 겹친다.
> 카메라 각도나 신호등 배치가 바뀌면 **반드시 다시 잡아야 한다.**
> 조절 도구: `http://<ip>:8001/combined.html` 우측 하단 ROI 슬라이더.

---

## 4. 차선 노란색 오염 억제

### 4.1 문제

삼거리에서 **좌회전 도로의 노란 중앙선이 직진 도로의 좌측 흰선과 마스크상 붙어 있다.**
기존 `_split_blobs` 는 "덩이 안에 노란 씨앗이 있으면 덩이 전체가 노란선" 이라, 흰선까지
통째로 노랑이 되고 차가 좌회전 쪽으로 빨려간다.

실측 (프레임 `000370.jpg`): 노란 화소가 **231 → 3353** 으로 14배 폭증하고 목표 x가
**327 → 78** 로 한 프레임에 249px 튄다.

### 4.2 이 오염은 쓸모가 있다

**트리거가 꺼져 있으면 이 오염 덕분에 차가 좌회전을 따라간다.** 그래서 없애면 안 된다.
직진일 때만 억제한다.

### 4.3 억제 방법 (직진일 때만, 2단계)

```
1단계  grow_vertical()   씨앗에서 세로 방향으로만 노랑을 넓힌다
       커널 21(세로) x 3(가로) → 세로 10px 당 가로 1px, 수직에서 약 6도까지만
       노란 중앙선은 세로로 이어져 끝까지 번지고, 가로로 붙은 흰선은 안 넘어간다

2단계  reclaim_white()   흰선에서 한 번 더 퍼뜨려 남은 오염을 되덮는다
       씨앗을 뺀 영역에서 연결요소를 나누고, 흰선 화소가 든 덩어리는 통째로 흰선
       씨앗을 지나야 닿는 곳은 못 넘어오므로 진짜 중앙선은 남는다
       연결요소 한 번이라 비용 거의 없음
```

실측 효과 (프레임 `000370.jpg`):

| | 노랑 화소 | 좌측 오염 |
|---|---|---|
| 트리거 끔 (기존) | 3353 | 화면 왼쪽 끝까지 |
| 1단계만 | 2005 | 대각선 조각 남음 |
| **1+2단계** | **856** | **없음** |

합성 마스크 검증: 진짜 노란선 1870화소 **100% 보존**, 오염 536화소 **전부 제거**.

### 4.4 기존 동작 보존 보증

```python
if cfg.straight_only:
    ...새 경로...
else:
    yellow_mask[box] |= blob      # 기존 코드 그대로
```

`straight_only` 기본값이 `False` 이고 `else` 가지는 원본 그대로다. 옛 코드와 새 코드를
같은 입력으로 돌려 **노랑 화소 4634 = 4634 로 동일**함을 확인했다 (무작위 마스크 5회 포함).
트리거가 안 켜지면 한 픽셀도 안 달라진다.

---

## 5. 신호 → 주행 연동 (상태 기계)

`yolop_drive_node.py` 의 `_on_traffic()` + `_on_control()` 에 있다.

```python
STOP_RELEASE_SEC   = 1.0    # red 를 이 시간 넘게 못 보면 정지 해제
RED_CONFIRM        = 3      # 빨간불 확정 연속 장수  (약 0.2초 @15Hz)
GO_CONFIRM         = 7      # 초록·좌회전 확정 연속 장수 (약 0.47초)
GO_LOCK_SEC        = 10.0   # 확정 뒤 신호등 인식을 통째로 쉬는 시간
STRAIGHT_HOLD_SEC  = 3.0    # 직진 확정 직후 조향 억제 시간
STRAIGHT_HOLD_GAIN = 0.15   # 그동안 조향에 곱하는 값
```

### 동작

| 입력 | 조건 | 결과 |
|---|---|---|
| `red` | 3장 연속 | `_traffic_stop = True` → **속도 0** (조향은 유지) |
| `green` | 7장 연속 | `_straight = True`, **3초 조향 x0.15**, 10초 잠금 |
| `left` | 7장 연속 | `_straight = False` (오염 유지 → 좌측 추종), 10초 잠금 |
| `yellow` / `none` | — | 아무것도 안 바꿈. 하던 대로 간다 |
| — | 잠금 중 | **모든 신호 무시.** 빨간불도 안 읽는다 |
| — | red 1초 이상 없음 | 정지 자동 해제 |

**왜 잠금이 필요한가:** 이전에는 한 프레임짜리 오검출이 트리거를 그대로 뒤집었다.
실측 로그에서 `좌회전 → 초록불 → 빨간불` 이 2초 안에 전부 일어났다.

**왜 잠금 중 빨간불까지 끄는가:** 확정한 판단을 흔들 것이 없으면 그 사이 오검출에
완전히 면역이다. 신호등을 지나 삼거리까지 가는 구간이 여기에 해당한다.

**왜 조향 억제가 필요한가:** 초록불(직진)인데도 노란색 전파가 완전히는 안 잡힐 때
좌회전 차선으로 끌려간다. 3초간 조향을 15%로 줄여 사실상 직진만 하게 한다.
`0`으로 두면 완전 직진.

### 정지 수단

`_publish_motor(steer, speed)` 의 `speed` 를 0으로 떨구는 것뿐이다.
**브레이크가 아니라 스로틀 0** 이다. `/commands/motor/brake` 토픽은 이 경로에서 안 쓴다.

---

## 6. 실행

```bash
# 전체 (센서 + 차선 + 신호등 + 대시보드 + 모터 armed)
ssh kobot "/home/xytron/drive_test.sh"

# 센서만 (모터 제외)
ssh kobot "/home/xytron/restart_sensors.sh"

# 뷰어용 최소 (라이다·IMU·초음파 제외)
ssh kobot "/home/xytron/viewer_only.sh"
```

그 뒤 **http://\<ip\>:8000/index.html** 에서 기록 주행 시작.

### 환경 함정

1. **차선 노드는 `import torch` 가 필요하다.** 추론은 TensorRT가 하지만 torch가 GPU
   버퍼 할당(`torch.empty(device='cuda')`)과 CUDA 스트림을 담당한다. pycuda를 안 쓰려고
   일부러 그렇게 설계돼 있다 (`trt_backend.py` 주석 참고).
   ```bash
   export PYTHONPATH=/home/xytron/.venvs/torch/lib/python3.10/site-packages:$PYTHONPATH
   ```
   `twinmixing` 은 **폴더 이름일 뿐 알고리즘과 무관하다.** 헷갈려서
   `/home/xytron/.venvs/torch` 심볼릭 링크를 만들어 뒀다.

2. **`dashboard_server` 가 차선 노드를 자식으로 띄운다.** 부모에 위 PYTHONPATH가 없으면
   대시보드의 차선 재시작이 `ModuleNotFoundError: torch` 로 실패한다.

3. **기록 주행 버튼은 노드를 새로 안 띄운다.** `yolop_arm.sh` 가 `ros2 param set` 으로
   arm 만 한다. **코드를 고쳤으면 노드를 재시작해야 반영된다.**

4. **`pkill -f` 를 명령줄에 직접 쓰지 마라.** 패턴이 자기 자신의 명령줄과 매칭돼서
   셸이 죽는다. 위 스크립트들은 패턴을 파일 안에 둬서 피했다.

---

## 7. 토픽 계약 (합칠 때 여기만 보면 된다)

### 발행 — `traffic_perception_node`

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/traffic_light/state` | `std_msgs/String` | JSON. 아래 |
| `/traffic_light/debug_img` | `sensor_msgs/Image` | 원본 + 박스 + ROI |

```json
{
  "state": "red|yellow|green|left|none",   // 이 프레임 단독 판정
  "stable": "...",                          // 3프레임 연속 확인값
  "ko": "빨간불",
  "roi":   {"x":0.302,"y":0.373,"w":0.225,"h":0.071},   // 0~1 정규화
  "boxes": [{"x":..,"y":..,"w":..,"h":..,"c":"red|amber|cyan","area":..,"nb":..}],
  "lamps": 1, "ms": 24.1, "frames": 1234, "w": 640, "h": 480, "stamp": ...
}
```

좌표는 전부 **0~1 정규화**다. 해상도가 달라도 그대로 쓸 수 있다.

### 구독 — `traffic_perception_node`

`/image_raw/compressed` 와 `/image_raw` **둘 다** 구독한다. 어느 쪽이 와도 처리한다.

### 차선 노드가 쓰는 것

`/traffic_light/state` 의 **`state`** 필드만 본다 (`stable` 은 안 쓴다).
확정·잠금은 차선 노드 쪽에서 한다. 신호등 노드는 순수하게 프레임 단독 판정만 한다.

**후반부 코드에서 신호를 쓰려면 `/traffic_light/state` 를 구독하면 된다.**
신호등 노드는 건드릴 필요 없다.

---

## 8. 분석 도구 (포트 8001)

```bash
ssh kobot "setsid nohup python3 -m http.server 8001 --bind 0.0.0.0 \
  --directory /home/xytron/lane_eval/replay > /tmp/v8001.log 2>&1 < /dev/null &"
```

| 페이지 | 용도 |
|---|---|
| `/combined.html` | **주력.** 주행 기록 재생. 한 장에 신호등+차선 오버레이, 이 장의 판정과 유지 중인 상태, ROI 슬라이더 |
| `/signal_all.html` | 촬영본에 대한 방법별 비교표 (행=방법, 열=신호색) |
| `/roi.html` | 정지 사진 2장으로 ROI 조절 |
| `/drive.html` | 3분할 재생 (원본 / 트리거끔 / 트리거켬) |

### 재생 자료 만들기

```bash
# 기록 폴더 이름은 /home/xytron/recordings/camera/ 아래
ssh kobot "/tmp/run_comb.sh 20260821_042119_910"
```
`build_combined.py` 가 프레임마다
① 신호등 노드를 그대로 돌려 판정
② 차선 노드의 확정·잠금 규칙을 재현해 그 시점 유지 상태 계산
③ 그 상태로 차선을 돌려 오버레이
를 만든다. **그림이 실제 주행에서 일어났을 판단과 같다.**

### 기록 위치

```
/home/xytron/recordings/camera/<타임스탬프>/
    000000.jpg ...            프레임
    timestamps.csv            filename, ros_time_ns
    drive_telemetry.csv       wall_time, lane_state, control_mode, speed_mps, ...
    session.json              speed_mps, target_fps
```

**주의:** 기록은 약 6fps 로 저장되는데 신호등 노드는 약 15Hz 로 돈다. **기록에 없는
프레임이 절반 이상이다.** 실차가 본 신호가 기록에 없을 수 있다.

---

## 9. 합칠 때 충돌할 지점

| 지점 | 내용 |
|---|---|
| `lane_core.py` `_split_blobs` | 후반부 코드도 여기를 고쳤다면 충돌한다. 우리 변경은 `if cfg.straight_only:` 가지 하나 추가가 전부다. `else` 는 원본 그대로 |
| `yolop_drive_node.py` `_on_control` | `speed` 계산에 `_traffic_stop`, `steer` 에 `STRAIGHT_HOLD_GAIN` 이 끼어든다 |
| `sensors.launch.py` | `traffic_perception` 노드 한 줄 추가 |
| `dashboard/app.js` | `laneCard()`, `viewLabels`, `recoveryLabels` 수정 + `renderSignalCards`/`drawSignalOverlay` 추가 |
| 모터 발행권 | `dashboard_server.ensure_exclusive_motor_publisher()` 가 있다. `/xycar_motor` 를 두 노드가 동시에 쓰면 안 된다 |

**우리 쪽 변경은 전부 "기본값 꺼짐 + 기존 경로 보존" 으로 넣었다.** 신호등 노드를 안 띄우면
`/traffic_light/state` 가 안 오고, `_traffic_stop`/`_straight` 가 계속 `False` 라
**기존 주행과 완전히 동일하게 동작한다.** 합치다 문제가 생기면 신호등 노드만 빼면 된다.

---

## 10. 알려진 한계

1. **ROI가 카메라 자세에 민감하다.** 8/20 촬영본과 8/21 주행본의 램프 위치가 거의 안 겹친다.
   현장에서 다시 잡아야 한다. `combined.html` 슬라이더로 30초면 된다.

2. **난간 위 라바콘이 신호등 높이에 있으면 빨간불로 잡힌다.** 색으로는 구분 불가
   (콘 `nr 0.404` vs 빨강램프 `nr 0.396`). 지금은 ROI로 피하고 있다.
   근본 해결은 링 대비(밝은 원 + 어두운 베젤) 조건 추가인데 아직 안 넣었다.

3. **`GO_LOCK_SEC = 10초` 동안 빨간불도 무시한다.** 잠금 중 신호가 빨강으로 바뀌면 못 본다.
   삼거리 통과 시간이 10초를 넘으면 위험하다.

4. **`_straight` 는 다음 `left` 확정까지 계속 물려 있다.** 초록불로 한 바퀴 돌면 다음 바퀴
   신호등을 다시 볼 때까지 켜져 있다. 트랙의 다른 곳에서 오염을 이용해 도는 구간이 있다면
   그 구간이 망가진다. **후반부 구간에 그런 곳이 있는지 확인 필요.**

5. **기록 fps(6) < 노드 fps(15).** 기록만으로는 실차가 본 신호를 다 알 수 없다.
   정밀 분석이 필요하면 프레임별 판정을 로그에 남기게 고쳐야 한다.

6. **8/21 04:48에 보드가 한 번 재부팅됐다.** 원인 미확인.

---

## 11. 실측 데이터

```
/home/xytron/signal_shots/{red,yellow,green,left,off}/   각 12장 (2026-08-20 23:54)
/home/xytron/signal_shots/left_far/                       6장 (2026-08-21 주행 중, 램프 8px)
/home/xytron/recordings/camera/<타임스탬프>/              주행 기록
```

검증 스크립트: `/tmp/verify_tp.py` (ROS 없이 노드 알고리즘만 돌린다)

```bash
ssh kobot "/home/xytron/clrernet_eval/venv/bin/python3 /tmp/verify_tp.py"
```

8/20 촬영본 기준 최종 성적은 **점등 4상태 48/48**이었다 (그때 ROI 기준).
현재 ROI로는 그 데이터가 범위 밖이라 0/48 이 나온다 — §3.4 참고.

---

## 12. 분석용 파이썬 환경

```
분석용  /home/xytron/clrernet_eval/venv/bin/python3
        numpy 1.26.4 · cv2 4.11.0 · torch 2.8.0 · ultralytics 8.4.121
        rclpy 없음 → 노드를 불러오려면 ROS 모듈을 껍데기로 채워야 한다
        (build_combined.py 의 load_traffic() 참고)

노드용  /usr/bin/python3 + ROS humble
        numpy 1.21.5 · cv2 4.5.4 · rclpy · cv_bridge
        torch 는 PYTHONPATH 로 /home/xytron/.venvs/torch 에서 온다
```

**분석 venv에 pip 설치할 때 numpy가 2.x로 올라가지 않는지 반드시 확인하라.**
이전에 `pip install onnx` 로 numpy가 올라가 torch가 깨진 적이 있다.
