"""카메라 노출 제어 — 신호등 대기 중에만 노출을 낮춘다.

왜 (0817 실측):
  usb_cam 자동노출은 실내 바닥에 맞춰 노출을 올리는데, 그 결과 점등 램프가
  **채도 12 의 흰 점**으로 날아간다. 8~10m 출발선에서는 색도, 하우징의 어두운
  테두리도 남지 않아 어떤 알고리즘도 빨강/초록을 가를 수 없다.

  노출을 낮추면 상황이 뒤집힌다: 바닥·벽은 어두워지고 **발광체인 램프만**
  밝게 남는다. 즉 '점등 램프 찾기'가 가장 쉬운 문제가 되고, 하우징도 검게
  살아나 위치(몇 번째 램프인가) 판정이 가능해진다.

  자이트론 순정 워크스페이스가 같은 방식을 제공한다 (app_cam_exposure):
      v4l2-ctl -d /dev/videoCAM -c auto_exposure=1            # 1 = 수동
      v4l2-ctl -d /dev/videoCAM -c exposure_time_absolute=N   # 1..5000
  타 참가팀은 아무도 노출을 건드리지 않는다 (전수조사 확인) — ROI 를 좁히고
  임계를 올리는 방식으로 버틴다. 우리는 문제의 원인을 직접 없앤다.

안전 규약:
  · **주행 중에는 절대 건드리지 않는다.** 차선 흰색 임계(V 243~255)가 노출에
    직결되므로, 노출을 바꾼 채 달리면 차선을 잃는다.
  · WAIT_GREEN 진입 시 낮추고, 출발이 확정되면 즉시 자동으로 되돌린다.
    출발 직진 홀드(1.2s) 동안 카메라가 안정화될 시간이 확보된다.
  · v4l2-ctl 이 없거나 실패해도 예외를 올리지 않는다 — 노출 제어는 '있으면
    좋은' 보조 수단이고, 실패 시 기존 경로(위치 기반 워처)가 그대로 돈다.
"""

from __future__ import annotations

import shutil
import subprocess
from typing import Optional


DEFAULT_DEVICE = "/dev/videoCAM"
# UVC 규약: auto_exposure 1 = 수동(Manual Mode), 3 = 자동(Aperture Priority)
_MANUAL = "1"
_AUTO = "3"


class CameraExposure:
    """v4l2-ctl 래퍼. 실패해도 조용히 넘어간다 (best effort)."""

    def __init__(self, device: str = DEFAULT_DEVICE, timeout_s: float = 1.5) -> None:
        self.device = device
        self.timeout_s = timeout_s
        self.available = shutil.which("v4l2-ctl") is not None
        self.last_error: str = ""
        self._state: Optional[str] = None    # "manual" | "auto" | None

    # ── 내부 ──────────────────────────────────────────────────────
    def _run(self, *ctrls: str) -> bool:
        if not self.available:
            self.last_error = "v4l2-ctl 없음"
            return False
        cmd = ["v4l2-ctl", "-d", self.device]
        for c in ctrls:
            cmd += ["-c", c]
        try:
            r = subprocess.run(
                cmd, capture_output=True, timeout=self.timeout_s, text=True
            )
        except Exception as exc:                       # noqa: BLE001
            self.last_error = f"{type(exc).__name__}: {exc}"
            return False
        if r.returncode != 0:
            self.last_error = (r.stderr or r.stdout or "").strip()[:120]
            return False
        self.last_error = ""
        return True

    # ── 공개 ──────────────────────────────────────────────────────
    def set_manual(self, exposure: int) -> bool:
        """수동 노출로 전환. exposure 는 1~5000 (작을수록 어둡다)."""
        exposure = max(1, min(5000, int(exposure)))
        # 순서 중요: 먼저 수동 모드로 바꿔야 노출값이 먹는다.
        ok = self._run(f"auto_exposure={_MANUAL}")
        ok = self._run(f"exposure_time_absolute={exposure}") and ok
        if ok:
            self._state = "manual"
        return ok

    def set_auto(self) -> bool:
        """자동 노출로 복귀."""
        ok = self._run(f"auto_exposure={_AUTO}")
        if ok:
            self._state = "auto"
        return ok

    def set_drive(self, drive_exposure: int) -> bool:
        """주행용 노출로 전환. 0 이면 자동 노출로 되돌린다.

        왜 '자동 복귀'가 기본이 아닌가 (0818 bag 실측):
            자동노출은 세션 내내 화면 평균 V 를 **109~214 로 흔든다**
            (V≥243 통과 픽셀 비율은 0.000~0.385, 385배). 밝기 분포가
            프레임마다 바뀌면 어떤 임계값도 매 프레임 다른 문제를 푼다.
            노출을 하나로 고정해야 그때부터 아래 단계가 정지 상태가 된다.

        ⚠️ 값은 **트랙에서 실측해서 넣어야 한다** — 조명이 바뀌면 달라진다.
            `ros2 run kmu_race exposure_scan` 이 후보를 쓸어 보고 차선
            검출률이 가장 높은 값을 알려 준다. 실측 전에는 0(자동) 으로
            두는 것이 안전하다: 지어낸 상수로 주행하느니 자동이 낫다.
        """
        if int(drive_exposure) <= 0:
            return self.set_auto()
        return self.set_manual(int(drive_exposure))

    def enforce_drive(self, drive_exposure: int) -> bool:
        """주행 노출이 풀렸으면 재적용 (드라이버 자가 복귀 대응).

        0818 실측대로 수동 노출은 스스로 풀린다. 주행 중에 풀리면 화면이
        확 밝아지면서 차선 대비가 무너지므로, 호출부가 주기적으로 이걸
        불러 줘야 한다. drive_exposure=0(자동) 이면 강제할 것이 없다.
        """
        if int(drive_exposure) <= 0:
            return False
        return self.enforce_manual(int(drive_exposure))

    def read_back(self) -> Optional[tuple]:
        """실제 카메라 상태 (mode, exposure) 를 읽는다. 실패 시 None.

        0818 실측: set_manual(60) 이 성공해도 **대기 중 스스로 풀린다**
        (run4 115s / run5 80s 후 밝기 127→192 점프 — 드라이버/펌웨어가
        컨트롤을 재설정하는 것으로 추정). '한 번 설정'을 믿을 수 없으므로
        호출부가 이걸로 주기 검증하고 어긋나면 재적용해야 한다.
        """
        if not self.available:
            return None
        try:
            r = subprocess.run(
                ["v4l2-ctl", "-d", self.device, "--get-ctrl",
                 "auto_exposure,exposure_time_absolute"],
                capture_output=True, timeout=self.timeout_s, text=True,
            )
        except Exception as exc:                       # noqa: BLE001
            self.last_error = f"{type(exc).__name__}: {exc}"
            return None
        if r.returncode != 0:
            self.last_error = (r.stderr or r.stdout or "").strip()[:120]
            return None
        mode = value = None
        for line in r.stdout.splitlines():
            if "auto_exposure" in line:
                mode = line.split(":")[1].strip().split()[0]
            elif "exposure_time_absolute" in line:
                value = int(line.split(":")[1].strip().split()[0])
        if mode is None:
            return None
        return ("manual" if mode == _MANUAL else "auto", value)

    def enforce_manual(self, exposure: int) -> bool:
        """read-back 으로 확인해 어긋나 있으면 재적용. 맞으면 no-op.

        반환값: 재적용을 **했으면** True (호출부 로그용).
        """
        rb = self.read_back()
        if rb is not None and rb[0] == "manual" and rb[1] == int(exposure):
            return False
        self.set_manual(exposure)
        return True

    @property
    def state(self) -> Optional[str]:
        return self._state


__all__ = ["CameraExposure", "DEFAULT_DEVICE"]
