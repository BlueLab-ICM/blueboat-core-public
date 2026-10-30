"""Tests for the gateway safety contract in `core_gateway.safety`.

The clock is injected: `now` is a parameter, never `time.time()`.
"""

import pytest
from core_gateway.safety import AUTHORIZED_SOURCE, ROVER_MODE, AutonomyLease, is_guided

T0 = 1_000.0  # arbitrary epoch; only differences matter
TIMEOUT_S = 2.0
GUIDED = ROVER_MODE['GUIDED']
HOLD = ROVER_MODE['HOLD']
MANUAL = ROVER_MODE['MANUAL']
NOT_GUIDED = [None, 99, *sorted(set(ROVER_MODE.values()) - {GUIDED})]


def _held_lease(now: float = T0) -> AutonomyLease:
    """Return a lease acquired by the brain at `now`, with the vehicle in GUIDED."""
    lease = AutonomyLease(timeout_s=TIMEOUT_S)
    assert lease.on_command(AUTHORIZED_SOURCE, now, GUIDED) is True
    assert lease.active is True
    return lease


def test_a_fresh_lease_is_not_held() -> None:
    lease = AutonomyLease()
    assert lease.active is False
    assert lease.allows_send(T0, GUIDED) is False


def test_the_default_timeout_is_the_two_second_watchdog() -> None:
    assert AutonomyLease().timeout_s == TIMEOUT_S


@pytest.mark.parametrize('source', ['', 'core_gateway', 'payload_ctd', 'CORE_AUTONOMY'])
def test_an_unauthorized_source_is_rejected(source: str) -> None:
    """The source check is exact: no empty string, no neighbouring node, no case folding."""
    lease = AutonomyLease(timeout_s=TIMEOUT_S)
    assert lease.on_command(source, T0, GUIDED) is False
    assert lease.active is False


def test_an_unauthorized_command_does_not_renew_the_lease() -> None:
    lease = _held_lease(T0)
    assert lease.on_command('payload_ctd', T0 + 1.9, GUIDED) is False
    assert lease.tick(T0 + 2.1) is False  # the deadline still counts from T0


def test_the_lease_is_not_acquired_outside_guided() -> None:
    lease = AutonomyLease(timeout_s=TIMEOUT_S)
    assert lease.on_command(AUTHORIZED_SOURCE, T0, HOLD) is True
    assert lease.active is False


def test_the_lease_is_acquired_once_the_operator_selects_guided() -> None:
    lease = AutonomyLease(timeout_s=TIMEOUT_S)
    lease.on_command(AUTHORIZED_SOURCE, T0, HOLD)
    lease.on_command(AUTHORIZED_SOURCE, T0 + 0.5, GUIDED)
    assert lease.active is True


def test_an_authorized_command_renews_the_lease_outside_guided() -> None:
    """Renewal is a keepalive, so a held lease survives a manual stretch."""
    lease = _held_lease(T0)
    assert lease.on_command(AUTHORIZED_SOURCE, T0 + 1.5, MANUAL) is True
    assert lease.tick(T0 + 3.0) is True
    assert lease.active is True


def test_sends_are_dropped_outside_guided() -> None:
    """ArduRover auto-enters GUIDED on a position target, which would override the operator."""
    lease = _held_lease(T0)
    assert lease.allows_send(T0, HOLD) is False
    assert lease.allows_send(T0, MANUAL) is False
    assert lease.allows_send(T0, GUIDED) is True


def test_the_lease_survives_just_short_of_the_timeout() -> None:
    lease = _held_lease(T0)
    assert lease.tick(T0 + 1.9) is True
    assert lease.active is True
    assert lease.allows_send(T0 + 1.9, GUIDED) is True


def test_the_timeout_itself_does_not_revoke_the_lease() -> None:
    """The check is `elapsed > timeout_s`, so exactly on the budget is still held."""
    lease = _held_lease(T0)
    assert lease.tick(T0 + TIMEOUT_S) is True
    assert lease.active is True


def test_the_lease_is_revoked_past_the_timeout() -> None:
    lease = _held_lease(T0)
    assert lease.tick(T0 + 2.1) is False
    assert lease.active is False
    assert lease.allows_send(T0 + 2.1, GUIDED) is False


def test_the_failsafe_is_reported_once_not_on_every_tick() -> None:
    lease = _held_lease(T0)
    assert lease.tick(T0 + 2.1) is False
    assert lease.tick(T0 + 3.1) is True
    assert lease.active is False


def test_a_stale_lease_is_refused_before_the_watchdog_ticks() -> None:
    """The watchdog runs at 1 Hz, so the lease can still be flagged active after the timeout."""
    lease = _held_lease(T0)
    assert lease.active is True
    assert lease.allows_send(T0 + 2.1, GUIDED) is False


def test_a_revoked_lease_is_reacquired_in_guided() -> None:
    lease = _held_lease(T0)
    lease.tick(T0 + 2.1)
    assert lease.on_command(AUTHORIZED_SOURCE, T0 + 5.0, GUIDED) is True
    assert lease.active is True
    assert lease.allows_send(T0 + 5.0, GUIDED) is True


def test_a_revoked_lease_is_not_reacquired_outside_guided() -> None:
    """A brain that comes back after the operator took over does not resume."""
    lease = _held_lease(T0)
    lease.tick(T0 + 2.1)
    assert lease.on_command(AUTHORIZED_SOURCE, T0 + 5.0, MANUAL) is True
    assert lease.active is False


def test_guided_is_ardurover_mode_fifteen() -> None:
    """Pinned so a table edit cannot move the gated mode quietly."""
    assert GUIDED == 15
    assert is_guided(GUIDED) is True


@pytest.mark.parametrize('mode_num', NOT_GUIDED)
def test_every_other_mode_blocks_sending(mode_num: int | None) -> None:
    """None is no HEARTBEAT yet; 99 is a mode the table does not list."""
    assert is_guided(mode_num) is False
    assert _held_lease(T0).allows_send(T0, mode_num) is False
