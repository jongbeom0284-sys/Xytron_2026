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
        """자동 노출로 복귀 — 주행 전에 반드시 호출된다."""
        ok = self._run(f"auto_exposure={_AUTO}")
        if ok:
            self._state = "auto"
        return ok

    @property
    def state(self) -> Optional[str]:
        return self._state


__all__ = ["CameraExposure", "DEFAULT_DEVICE"]
