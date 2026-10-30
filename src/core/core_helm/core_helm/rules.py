"""Helm rules: which one command reaches the vehicle.

Every motion command in the system passes through :class:`Helm`. Each tick takes the
vehicle state and the latest requests and returns exactly one :class:`Output`. Free of ROS
so the tests run without it.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from core_helm.collision import CollisionFilter, Peer
from core_helm.model import HOLD, KEEPALIVE, POSITION, VELOCITY, Fence, Limits, Output, Vehicle

IDLE, ENGAGED, HOLDING, AVOIDING = 'idle', 'engaged', 'holding', 'avoiding'

# Overrides: the rule that replaced the granted source's request this tick.
STALE, OUTSIDE_FENCE, BAD_REQUEST, NO_GPS, COLLISION = (
    'stale_command',
    'target_outside_fence',
    'bad_request',
    'no_gps',
    'collision',
)


@dataclass(frozen=True)
class Config:
    """Helm timing. All times in seconds."""

    setpoint_timeout_s: float = 1.0
    grant_timeout_s: float = 1.0
    stale_stop_s: float = 30.0
    gps_timeout_s: float = 2.0
    # Ticks of zero velocity after an engagement ends: the autopilot keeps its last velocity
    # target until replaced, and one message can be lost.
    flush_ticks: int = 10
    cruise_speed_mps: float = 1.0
    yaw_gain_per_s: float = 0.8


@dataclass(frozen=True)
class Request:
    """The latest setpoint of one source."""

    source: str
    mode: int
    lat: float
    lon: float
    speed_mps: float
    yaw_rate_rps: float
    received: float


STOP = Output(VELOCITY)
QUIET = Output(KEEPALIVE)


class Helm:
    """Arbitrate requests into one command: engagement, staleness, fence, avoidance, limits."""

    def __init__(self, config: Config, limits: Limits, avoidance: CollisionFilter | None) -> None:
        self.config = config
        self.default_limits = limits
        self.limits = limits
        self.avoidance = avoidance
        self.fence: Fence | None = None
        self.engaged = False
        self.grant = ''
        self.override = ''
        self.active_source = ''
        self.reason = ''
        self._grant_time: float | None = None
        self._request: Request | None = None
        self._stale_since = 0.0
        self._hold_point: tuple[float, float] | None = None
        self._flush_left = 0
        self._gps_lost_since: float | None = None

    @property
    def state(self) -> str:
        """Return the state reported on /helm/status."""
        if not self.engaged:
            return IDLE
        if self.override == COLLISION:
            return AVOIDING
        return HOLDING if self.override else ENGAGED

    def engage(
        self, fence: Fence, limits: Limits | None, vehicle: Vehicle, now: float
    ) -> tuple[bool, str]:
        """Start relaying the granted source inside ``fence``."""
        if self.engaged:
            return False, 'already engaged'
        if not fence.valid():
            return False, 'the fence is not a position and a positive radius'
        if not vehicle.guided:
            return False, 'the autopilot is not in GUIDED'
        if vehicle.lat is None or vehicle.gps_age_s > self.config.gps_timeout_s:
            return False, 'no fresh GPS fix'
        if fence.breached(vehicle.lat, vehicle.lon):
            return False, 'the vehicle is outside the fence'
        self.fence = fence
        self.limits = limits or self.default_limits
        self.engaged = True
        self.grant = ''
        self.override = ''
        self.reason = ''
        self._grant_time = None
        self._request = None
        self._stale_since = now
        self._hold_point = None
        self._flush_left = 0
        self._gps_lost_since = None
        return True, 'engaged'

    def disengage(self, reason: str) -> None:
        """End the engagement and stop the vehicle."""
        if not self.engaged:
            return
        self.engaged = False
        self.reason = reason
        self.override = ''
        self.active_source = ''
        self.limits = self.default_limits
        self._flush_left = self.config.flush_ticks

    def on_grant(self, source: str, now: float) -> None:
        """Record who may drive. A grant that stops arriving makes the helm hold."""
        self.grant = source
        self._grant_time = now

    def on_setpoint(self, request: Request) -> None:
        """Keep the latest request of the granted source; others are dropped."""
        if request.source and request.source == self.grant:
            self._request = request

    def tick(self, now: float, vehicle: Vehicle, peers: Sequence[Peer] = ()) -> Output:
        """Return the command of this tick."""
        self.override = ''
        if not self.engaged:
            if self._flush_left > 0:
                self._flush_left -= 1
                return STOP
            return QUIET
        if not vehicle.guided:
            self.disengage('the autopilot left GUIDED')
            return STOP
        if vehicle.lat is None or vehicle.gps_age_s > self.config.gps_timeout_s:
            return self._without_position(now)
        self._gps_lost_since = None
        assert self.fence is not None
        if self.fence.breached(vehicle.lat, vehicle.lon):
            self.disengage('the vehicle left the fence')
            return STOP
        output = self._follow_grant(now, vehicle)
        if self.engaged and self.avoidance is not None and peers:
            output = self._avoid(now, vehicle, peers, output)
        return output

    def _without_position(self, now: float) -> Output:
        # Without a position neither the fence nor a hold can work, so stop the thrusters.
        self.override = NO_GPS
        self._gps_lost_since = self._gps_lost_since or now
        if now - self._gps_lost_since > self.config.stale_stop_s:
            self.disengage('no GPS fix')
        return STOP

    def _follow_grant(self, now: float, vehicle: Vehicle) -> Output:
        request = self._fresh_request(now)
        if request is None:
            if now - self._stale_since > self.config.stale_stop_s:
                self.disengage(f'no command for {self.config.stale_stop_s:.0f} s')
                return STOP
            return self._hold(vehicle, STALE)
        self._stale_since = now
        self.active_source = request.source
        if request.mode in (POSITION, HOLD):
            return self._position(request, vehicle)
        if request.mode == VELOCITY:
            return self._velocity(request, vehicle)
        return self._hold(vehicle, BAD_REQUEST)

    def _fresh_request(self, now: float) -> Request | None:
        cfg = self.config
        if self._grant_time is None or now - self._grant_time > cfg.grant_timeout_s:
            return None
        request = self._request
        if request is None or request.source != self.grant:
            return None
        return request if now - request.received <= cfg.setpoint_timeout_s else None

    def _position(self, request: Request, vehicle: Vehicle) -> Output:
        assert self.fence is not None
        if not (math.isfinite(request.lat) and math.isfinite(request.lon)):
            return self._hold(vehicle, BAD_REQUEST)
        if not self.fence.contains(request.lat, request.lon):
            return self._hold(vehicle, OUTSIDE_FENCE)
        self._hold_point = None
        return Output(POSITION, request.lat, request.lon)

    def _velocity(self, request: Request, vehicle: Vehicle) -> Output:
        speed, rate = request.speed_mps, request.yaw_rate_rps
        if not (math.isfinite(speed) and math.isfinite(rate)):
            return self._hold(vehicle, BAD_REQUEST)
        self._hold_point = None
        limits = self.limits
        return Output(
            VELOCITY,
            speed_mps=max(-limits.max_speed_mps, min(limits.max_speed_mps, speed)),
            yaw_rate_rps=max(-limits.max_yaw_rate_rps, min(limits.max_yaw_rate_rps, rate)),
        )

    def _hold(self, vehicle: Vehicle, why: str) -> Output:
        self.override = why
        if self._hold_point is None:
            assert vehicle.lat is not None and vehicle.lon is not None
            self._hold_point = (vehicle.lat, vehicle.lon)
        return Output(POSITION, *self._hold_point)

    def _avoid(self, now: float, vehicle: Vehicle, peers: Sequence[Peer], output: Output) -> Output:
        assert self.avoidance is not None and self.fence is not None
        steer = self.avoidance.check(now, vehicle, peers, output, self.fence)
        if steer is None:
            return output
        self.override = COLLISION
        err = steer.heading_error_rad
        rate = self.config.yaw_gain_per_s * err
        limits = self.limits
        return Output(
            VELOCITY,
            speed_mps=min(steer.speed_mps, limits.max_speed_mps),
            yaw_rate_rps=max(-limits.max_yaw_rate_rps, min(limits.max_yaw_rate_rps, rate)),
        )
