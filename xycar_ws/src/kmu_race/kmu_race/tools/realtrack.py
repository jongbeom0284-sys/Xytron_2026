"""대회 배치도(규정 p.6)를 lanesim Track 으로 옮긴 것 — 반시계 둥근 사각형 + 우측 슬라럼.

구간 대응:
  상단 직선  ① 신호등 출발 → ② 차선주행       좌상 코너 ③ 라바콘 구간이 걸침
  좌측 직선  ④ 고정장애물                      하단 직선 ⑤ 방해차량 추월
  우측       슬라럼 (배치도의 물결 구간)        우상 코너 → ⑦ 결승선

폐합 조건: 좌측직선 = 슬라럼 전진길이,  하단직선 = 상단직선 + 진입직선.
★ 치수는 추정값이다 — 규정 PDF 에 전체 치수가 없다. 실차런북의 "speed_scale 0.4
  에서 랩당 25~35초" 로 랩 길이를 잡고 배치도 비율에 맞췄다. 실측이 나오면
  TOP / ENTRY / R_CORNER / 슬라럼 상수만 바꾸면 된다.
"""
import math
from kmu_race.tools.lanesim import Track, Segment

TOP      = 3.6                    # 상단 직선 [m]
ENTRY    = 1.2                    # 우상 코너 → 결승선 [m]
R_CORNER = 1.05                   # 모서리 반경 (차 최소회전반경 1.01m 바로 위)
R_SLALOM = 1.05                   # 슬라럼 반경 (차 최소회전반경 1.01m 이상)
SLALOM_A = math.radians(35)       # 굽이 각
SLALOM_N = 2                      # 굽이 수

def _quarter(r): return Segment("arc", math.pi/2*r, r)

def _slalom_unit(r, a):
    """(+a, -2a, +a) — 순 방향·순 횡이탈이 모두 0 이라 직선을 그대로 대체한다."""
    return [Segment("arc",a*r,r), Segment("arc",2*a*r,-r), Segment("arc",a*r,r)]

def _slalom_forward(r, a, n):
    """슬라럼이 진행방향으로 실제로 나아가는 길이 [m]."""
    t = Track("_", _slalom_unit(r,a)*n)
    c = t.centerline()
    return float(math.hypot(c[-1,0]-c[0,0], c[-1,1]-c[0,1]))

LEFT   = _slalom_forward(R_SLALOM, SLALOM_A, SLALOM_N)   # 폐합 조건
BOTTOM = TOP + ENTRY                                     # 폐합 조건

def make_real_track() -> Track:
    return Track("대회 트랙 (추정)", [
        Segment("straight", TOP),     _quarter(R_CORNER),
        Segment("straight", LEFT),    _quarter(R_CORNER),
        Segment("straight", BOTTOM),  _quarter(R_CORNER),
        *_slalom_unit(R_SLALOM, SLALOM_A)*SLALOM_N, _quarter(R_CORNER),
        Segment("straight", ENTRY),
    ])

if __name__=="__main__":
    t=make_real_track(); c=t.centerline()
    L=sum(s.length for s in t.segments)
    err=math.hypot(c[-1,0]-c[0,0], c[-1,1]-c[0,1])
    print(f"폐합오차 {err:.3f} m   (좌측={LEFT:.2f}  하단={BOTTOM:.2f})")
    print(f"랩 길이 {L:.2f} m   바운딩 박스 {c[:,0].max()-c[:,0].min():.2f} × {c[:,1].max()-c[:,1].min():.2f} m")
    print(f"끝 heading {math.degrees(c[-1,2])%360:.1f}도   차선폭 {t.lane_width_m} m")
