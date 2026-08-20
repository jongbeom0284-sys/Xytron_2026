"""Pure state machine for an opt-in drive-enable heartbeat lease."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class EnableHeartbeatLease:
    """Require fresh ``true`` heartbeats when ``timeout_sec`` is positive.

    ``timeout_sec <= 0`` preserves the historical dashboard behavior: one true
    remains enabled until an explicit false arrives.  A positive timeout is a
    fail-closed lease intended for automated test harnesses.  Once it expires,
    only a newly received true can enable the bridge again.
    """

    timeout_sec: float = 0.0
    enabled: bool = False
    last_true_at: float | None = None
    expired: bool = False

    def __post_init__(self) -> None:
        self.timeout_sec = float(self.timeout_sec)
        if not math.isfinite(self.timeout_sec) or self.timeout_sec < 0.0:
            raise ValueError("enable heartbeat timeout must be finite and >= 0")

    def update(self, enabled: bool, now: float) -> None:
        self.enabled = bool(enabled)
        self.last_true_at = float(now) if self.enabled else None
        self.expired = False

    def active(self, now: float) -> bool:
        if not self.enabled:
            return False
        timeout = self.timeout_sec
        if timeout <= 0.0:
            return True
        if self.last_true_at is None or float(now) - self.last_true_at > timeout:
            self.enabled = False
            self.last_true_at = None
            self.expired = True
            return False
        return True
