import pytest

from kobot_lane_drive.enable_lease import EnableHeartbeatLease
from kobot_lane_drive.drive_command import make_legacy_command


def test_zero_timeout_preserves_legacy_latched_enable():
    lease = EnableHeartbeatLease(timeout_sec=0.0)
    lease.update(True, now=1.0)

    assert lease.active(now=10_000.0)
    assert not lease.expired

    lease.update(False, now=10_001.0)
    assert not lease.active(now=10_001.0)


def test_positive_heartbeat_lease_expires_fail_closed_and_needs_fresh_true():
    lease = EnableHeartbeatLease(timeout_sec=0.30)
    lease.update(True, now=1.0)

    assert lease.active(now=1.29)
    assert not lease.active(now=1.31)
    assert lease.expired
    assert not lease.active(now=5.0)

    # Expiry does not self-recover.  Only a newly received true re-arms it.
    lease.update(True, now=5.0)
    assert lease.active(now=5.29)
    assert not lease.expired


def test_repeated_true_is_a_fresh_heartbeat():
    lease = EnableHeartbeatLease(timeout_sec=0.30)
    lease.update(True, now=1.0)
    lease.update(True, now=1.25)

    assert lease.active(now=1.50)
    assert not lease.active(now=1.56)


def test_expired_lease_maps_to_an_explicit_motor_zero():
    lease = EnableHeartbeatLease(timeout_sec=0.30)
    lease.update(True, now=1.0)
    run_enabled = lease.active(now=1.31)

    command = make_legacy_command(
        steering_rad=0.2,
        speed_mps=0.55,
        angle_scale=0.0068,
        speed_scale=0.08,
        ready=True and run_enabled,
        fresh=True and run_enabled,
    )
    assert command == [0.0, 0.0]


@pytest.mark.parametrize("invalid", [-1.0, float("nan"), float("inf")])
def test_invalid_timeout_is_rejected(invalid):
    with pytest.raises(ValueError):
        EnableHeartbeatLease(timeout_sec=invalid)


@pytest.mark.parametrize("explicit", [False, 0, None])
def test_explicit_false_never_counts_as_a_heartbeat(explicit):
    lease = EnableHeartbeatLease(timeout_sec=0.30)
    lease.update(True, now=1.0)
    lease.update(explicit, now=1.1)
    assert not lease.active(now=1.1)
    assert lease.last_true_at is None
