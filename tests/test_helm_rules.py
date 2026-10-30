"""Tests of the single command path."""

import math

import pytest
from core_helm.collision import CollisionConfig, CollisionFilter, Peer
from core_helm.model import HOLD, KEEPALIVE, POSITION, VELOCITY, Fence, Limits, Vehicle
from core_helm.rules import (
    AVOIDING,
    COLLISION,
    ENGAGED,
    HOLDING,
    IDLE,
    NO_GPS,
    OUTSIDE_FENCE,
    STALE,
    Config,
    Helm,
    Request,
)

HOME = (41.0, 2.0)
FENCE = Fence(HOME[0], HOME[1], 50.0, 5.0)
EAST_30M = 2.0 + 30.0 / (111_195.0 * math.cos(math.radians(41.0)))
EAST_500M = 2.0 + 500.0 / (111_195.0 * math.cos(math.radians(41.0)))


def vehicle(
    lat: float | None = HOME[0],
    lon: float | None = HOME[1],
    guided: bool = True,
    gps_age_s: float = 0.1,
) -> Vehicle:
    return Vehicle(lat, lon, 0.0, 0.0, guided, gps_age_s)


def engaged_helm(avoidance: CollisionFilter | None = None) -> Helm:
    helm = Helm(Config(), Limits(2.0, 1.0), avoidance)
    ok, message = helm.engage(FENCE, None, vehicle(), 0.0)
    assert ok, message
    helm.on_grant('rl', 0.0)
    return helm


def go(lat: float, lon: float, t: float = 0.0, source: str = 'rl') -> Request:
    return Request(source, POSITION, lat, lon, 0.0, 0.0, t)


def test_idle_sends_keepalives_only() -> None:
    helm = Helm(Config(), Limits(), None)
    assert helm.tick(0.0, vehicle()).mode == KEEPALIVE
    assert helm.state == IDLE


@pytest.mark.parametrize(
    ('state', 'why'),
    [
        (vehicle(guided=False), 'GUIDED'),
        (vehicle(gps_age_s=9.0), 'GPS'),
        (vehicle(lat=None, lon=None), 'GPS'),
        (vehicle(lon=EAST_500M), 'outside'),
    ],
)
def test_engage_refused(state: Vehicle, why: str) -> None:
    ok, message = Helm(Config(), Limits(), None).engage(FENCE, None, state, 0.0)
    assert not ok
    assert why in message


def test_granted_source_reaches_the_vehicle() -> None:
    helm = engaged_helm()
    helm.on_setpoint(go(HOME[0], EAST_30M))
    output = helm.tick(0.1, vehicle())
    assert (output.mode, output.lon) == (POSITION, EAST_30M)
    assert helm.state == ENGAGED
    assert helm.active_source == 'rl'


def test_ungranted_source_is_ignored() -> None:
    helm = engaged_helm()
    helm.on_setpoint(go(HOME[0], EAST_30M, source='someone_else'))
    output = helm.tick(0.1, vehicle())
    assert helm.override == STALE
    assert (output.mode, output.lon) == (POSITION, HOME[1])


def test_stale_command_holds_then_ends() -> None:
    helm = engaged_helm()
    helm.on_setpoint(go(HOME[0], EAST_30M))
    helm.tick(0.1, vehicle())
    helm.on_grant('rl', 5.0)
    output = helm.tick(5.0, vehicle(lon=2.0001))
    assert helm.state == HOLDING
    assert (output.mode, output.lon) == (POSITION, 2.0001)
    helm.on_grant('rl', 31.0)
    output = helm.tick(31.0, vehicle())
    assert helm.state == IDLE
    assert output.mode == VELOCITY
    assert 'no command' in helm.reason


def test_silent_executive_holds_even_if_the_skill_keeps_publishing() -> None:
    helm = engaged_helm()
    helm.on_setpoint(go(HOME[0], EAST_30M, t=3.0))
    helm.tick(3.0, vehicle())
    assert helm.override == STALE


def test_target_outside_the_fence_is_refused_one_by_one() -> None:
    helm = engaged_helm()
    helm.on_setpoint(go(HOME[0], EAST_500M))
    output = helm.tick(0.1, vehicle())
    assert helm.override == OUTSIDE_FENCE
    assert output.lon == HOME[1]
    helm.on_setpoint(go(HOME[0], EAST_30M, t=0.2))
    assert helm.tick(0.2, vehicle()).lon == EAST_30M


def test_leaving_the_fence_stops_for_several_ticks() -> None:
    helm = engaged_helm()
    helm.on_setpoint(go(HOME[0], EAST_30M))
    output = helm.tick(0.1, vehicle(lon=EAST_500M))
    assert output.mode == VELOCITY
    assert (output.speed_mps, output.yaw_rate_rps) == (0.0, 0.0)
    assert helm.state == IDLE
    assert 'fence' in helm.reason
    flushes = [helm.tick(0.2 + i * 0.1, vehicle()).mode for i in range(11)]
    assert flushes == [VELOCITY] * 10 + [KEEPALIVE]


def test_operator_leaving_guided_ends_the_engagement() -> None:
    helm = engaged_helm()
    helm.tick(0.1, vehicle(guided=False))
    assert helm.state == IDLE
    assert 'GUIDED' in helm.reason


def test_no_gps_stops_the_thrusters_and_recovers() -> None:
    helm = engaged_helm()
    output = helm.tick(0.1, vehicle(gps_age_s=5.0))
    assert output.mode == VELOCITY
    assert helm.override == NO_GPS
    helm.on_grant('rl', 0.2)
    helm.on_setpoint(go(HOME[0], EAST_30M, t=0.2))
    assert helm.tick(0.2, vehicle()).mode == POSITION


def test_velocity_is_clamped() -> None:
    helm = engaged_helm()
    helm.on_setpoint(Request('rl', VELOCITY, 0.0, 0.0, 9.0, -9.0, 0.0))
    output = helm.tick(0.1, vehicle())
    assert (output.speed_mps, output.yaw_rate_rps) == (2.0, -1.0)


def test_nan_velocity_holds() -> None:
    helm = engaged_helm()
    helm.on_setpoint(Request('rl', VELOCITY, 0.0, 0.0, math.nan, 0.0, 0.0))
    assert helm.tick(0.1, vehicle()).mode == POSITION
    assert helm.state == HOLDING


def test_hold_request_is_a_position_inside_the_fence() -> None:
    helm = engaged_helm()
    helm.on_setpoint(Request('rl', HOLD, HOME[0], EAST_30M, 0.0, 0.0, 0.0))
    assert helm.tick(0.1, vehicle()).lon == EAST_30M


def test_collision_overrides_the_granted_source() -> None:
    helm = engaged_helm(CollisionFilter(CollisionConfig()))
    north_20m = HOME[0] + 20.0 / 111_195.0
    helm.on_setpoint(go(north_20m + 0.0001, HOME[1]))
    peer = Peer('other', north_20m, HOME[1], math.pi, 1.0, 0.1)
    output = helm.tick(0.1, vehicle(), [peer])
    assert helm.override == COLLISION
    assert helm.state == AVOIDING
    assert output.mode == VELOCITY
