"""Tests of the collision filter, with a simple kinematic simulation of two vehicles."""

import math

import pytest
from core_common.geodesy import haversine_m
from core_helm.collision import CollisionConfig, CollisionFilter, Peer
from core_helm.model import POSITION, Fence, Output, Vehicle

LAT, LON = 41.0, 2.0
M_PER_DEG = 111_195.0
FENCE = Fence(LAT, LON, 500.0, 5.0)


def offset(north: float, east: float) -> tuple[float, float]:
    return (
        LAT + north / M_PER_DEG,
        LON + east / (M_PER_DEG * math.cos(math.radians(LAT))),
    )


class Boat:
    def __init__(
        self, north: float, east: float, heading: float, goal: tuple[float, float]
    ) -> None:
        self.north, self.east, self.heading = north, east, heading
        self.speed = 1.0
        self.goal = goal
        self.filter = CollisionFilter(CollisionConfig())
        self.overrides = 0

    def vehicle(self) -> Vehicle:
        lat, lon = offset(self.north, self.east)
        return Vehicle(lat, lon, self.heading, self.speed, True, 0.1)

    def peer(self, name: str, age_s: float = 0.0) -> Peer:
        lat, lon = offset(self.north, self.east)
        return Peer(name, lat, lon, self.heading, self.speed, age_s)

    def step(self, t: float, other: 'Boat', dt: float) -> None:
        lat, lon = offset(*self.goal)
        output = Output(POSITION, lat, lon)
        steer = self.filter.check(t, self.vehicle(), [other.peer('other')], output, FENCE)
        if steer is None:
            course = math.atan2(self.goal[1] - self.east, self.goal[0] - self.north)
            err = math.atan2(math.sin(course - self.heading), math.cos(course - self.heading))
            speed = 1.0
        else:
            self.overrides += 1
            err, speed = steer.heading_error_rad, steer.speed_mps
        self.heading += max(-0.5, min(0.5, 0.8 * err)) * dt
        self.speed = speed
        self.north += speed * math.cos(self.heading) * dt
        self.east += speed * math.sin(self.heading) * dt


def distance(a: Boat, b: Boat) -> float:
    return math.hypot(a.north - b.north, a.east - b.east)


def simulate(a: Boat, b: Boat, seconds: float = 200.0, dt: float = 0.1) -> float:
    closest = distance(a, b)
    for i in range(int(seconds / dt)):
        a.step(i * dt, b, dt)
        b.step(i * dt, a, dt)
        closest = min(closest, distance(a, b))
    return closest


def test_head_on_vehicles_stay_apart() -> None:
    a = Boat(-60.0, 0.0, 0.0, (60.0, 0.0))
    b = Boat(60.0, 0.0, math.pi, (-60.0, 0.0))
    assert simulate(a, b) >= 8.0
    assert a.overrides + b.overrides > 0


def test_crossing_vehicles_stay_apart() -> None:
    a = Boat(-60.0, 0.0, 0.0, (60.0, 0.0))
    b = Boat(0.0, -60.0, math.pi / 2.0, (0.0, 60.0))
    assert simulate(a, b) >= 8.0


def test_both_vehicles_reach_their_goals_after_avoiding() -> None:
    a = Boat(-60.0, 0.0, 0.0, (60.0, 0.0))
    b = Boat(60.0, 0.0, math.pi, (-60.0, 0.0))
    simulate(a, b)
    assert a.north > 45.0
    assert b.north < -45.0


def test_clear_course_is_left_alone() -> None:
    a = Boat(0.0, 0.0, 0.0, (100.0, 0.0))
    b = Boat(0.0, 200.0, 0.0, (100.0, 200.0))
    simulate(a, b, seconds=20.0)
    assert a.overrides == 0


def test_lost_peer_is_ignored_after_the_timeout() -> None:
    a = Boat(-30.0, 0.0, 0.0, (60.0, 0.0))
    lat, lon = offset(*a.goal)
    steer = a.filter.check(
        0.0,
        a.vehicle(),
        [Peer('other', *offset(30.0, 0.0), math.pi, 1.0, 11.0)],
        Output(POSITION, lat, lon),
        FENCE,
    )
    assert steer is None


def test_old_report_widens_the_margin() -> None:
    a = Boat(-30.0, 0.0, 0.0, (60.0, 0.0))
    lat, lon = offset(*a.goal)
    # Abeam at 14 m: clear when fresh, inside the widened radius when 8 s old.
    fresh = Peer('other', *offset(-30.0, 14.0), 0.0, 1.0, 0.0)
    stale = Peer('other', *offset(-38.0, 14.0), 0.0, 1.0, 8.0)
    filter_fresh = CollisionFilter(CollisionConfig())
    filter_stale = CollisionFilter(CollisionConfig())
    output = Output(POSITION, lat, lon)
    assert filter_fresh.check(0.0, a.vehicle(), [fresh], output, FENCE) is None
    assert filter_stale.check(0.0, a.vehicle(), [stale], output, FENCE) is not None


def test_override_is_held_until_the_way_has_been_clear() -> None:
    a = Boat(-20.0, 0.0, 0.0, (60.0, 0.0))
    near = Peer('other', *offset(0.0, 0.0), math.pi, 1.0, 0.0)
    far = Peer('other', *offset(0.0, 300.0), math.pi, 1.0, 0.0)
    lat, lon = offset(*a.goal)
    output = Output(POSITION, lat, lon)
    assert a.filter.check(0.0, a.vehicle(), [near], output, FENCE) is not None
    assert a.filter.check(1.0, a.vehicle(), [far], output, FENCE) is not None
    assert a.filter.check(4.0, a.vehicle(), [far], output, FENCE) is None


def test_avoidance_never_steers_out_of_the_fence() -> None:
    fence = Fence(LAT, LON, 25.0, 5.0)
    a = Boat(20.0, 0.0, 0.0, (20.0, 0.0))
    lat, lon = offset(*a.goal)
    peer = Peer('other', *offset(20.0, 12.0), -math.pi / 2.0, 1.0, 0.0)
    steer = a.filter.check(0.0, a.vehicle(), [peer], Output(POSITION, lat, lon), fence)
    if steer is not None and steer.speed_mps > 0.0:
        course = a.heading + steer.heading_error_rad
        north = 20.0 + steer.speed_mps * math.cos(course) * 5.0
        east = steer.speed_mps * math.sin(course) * 5.0
        assert haversine_m(*offset(north, east), LAT, LON) <= 25.0 + 1e-6


@pytest.mark.parametrize('heading', [0.0, 1.0, 3.0, -2.0])
def test_no_peers_means_no_override(heading: float) -> None:
    a = Boat(0.0, 0.0, heading, (50.0, 50.0))
    lat, lon = offset(*a.goal)
    assert a.filter.check(0.0, a.vehicle(), [], Output(POSITION, lat, lon), FENCE) is None
