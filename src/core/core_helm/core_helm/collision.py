"""Collision filter: replace a command that would bring two vehicles too close.

Closest-point-of-approach check over a horizon. When a conflict exists the filter picks the
velocity nearest the wanted one that clears every peer, preferring a turn to starboard, so
two vehicles that meet head-on turn away from each other. Free of ROS.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from core_common.geodesy import gps_to_local_ned, wrap_pi

from core_helm.model import POSITION, VELOCITY, Fence, Output, Vehicle

# Tried in this order; positive is clockwise, so starboard comes first.
_HEADING_OFFSETS_DEG = (0, 30, 60, 90, 120, -30, -60, -90, 150, 180)
_SPEED_FACTORS = (1.0, 0.5)
# A fallback candidate is scored by the separation it gives this far ahead.
_SCORE_AHEAD_S = 5.0
# Candidates must stay inside the fence for this long.
_FENCE_LOOKAHEAD_S = 5.0


@dataclass(frozen=True)
class Peer:
    """Another vehicle, as last heard."""

    vehicle: str
    lat: float
    lon: float
    heading_rad: float
    speed_mps: float
    age_s: float


@dataclass(frozen=True)
class Steer:
    """The filter's replacement: drive at this speed, turn by this much."""

    speed_mps: float
    heading_error_rad: float


@dataclass(frozen=True)
class CollisionConfig:
    """Filter settings. Distances in metres, times in seconds."""

    safety_radius_m: float = 10.0
    horizon_s: float = 30.0
    clear_s: float = 3.0
    peer_timeout_s: float = 10.0
    cruise_speed_mps: float = 1.0
    max_radius_growth_m: float = 10.0


def _closest_approach(px: float, py: float, vx: float, vy: float, horizon: float) -> float:
    """Return the smallest separation over the horizon for relative position and velocity."""
    speed_sq = vx * vx + vy * vy
    t = 0.0 if speed_sq < 1e-9 else max(0.0, min(horizon, -(px * vx + py * vy) / speed_sq))
    return math.hypot(px + vx * t, py + vy * t)


class CollisionFilter:
    """Keep vehicles apart. Holds an override until the way has been clear for ``clear_s``."""

    def __init__(self, config: CollisionConfig) -> None:
        self.config = config
        self._last_conflict: float | None = None

    def check(
        self,
        now: float,
        vehicle: Vehicle,
        peers: Sequence[Peer],
        output: Output,
        fence: Fence,
    ) -> Steer | None:
        """Return a replacement for ``output``, or None when it is safe."""
        cfg = self.config
        tracks = self._tracks(vehicle, peers)
        heading = vehicle.heading_rad
        if not tracks or heading is None:
            self._last_conflict = None
            return None
        speed, course = self._wanted(vehicle, output)
        if self._separation(tracks, speed, course, cfg.horizon_s) >= 0.0:
            if self._last_conflict is None or now - self._last_conflict >= cfg.clear_s:
                self._last_conflict = None
                return None
        else:
            self._last_conflict = now
        best = self._choose(tracks, vehicle, speed or cfg.cruise_speed_mps, course, fence)
        return Steer(best[0], wrap_pi(best[1] - heading))

    def _tracks(
        self, vehicle: Vehicle, peers: Sequence[Peer]
    ) -> list[tuple[float, float, float, float, float]]:
        """Return (north, east, vn, ve, radius) of each peer heard recently, around the vehicle."""
        cfg = self.config
        if vehicle.lat is None or vehicle.lon is None:
            return []
        tracks = []
        for peer in peers:
            if peer.age_s > cfg.peer_timeout_s:
                continue
            north, east = gps_to_local_ned(peer.lat, peer.lon, vehicle.lat, vehicle.lon)
            vn = peer.speed_mps * math.cos(peer.heading_rad)
            ve = peer.speed_mps * math.sin(peer.heading_rad)
            # Dead-reckon an old report and widen the radius by what it could have done.
            growth = min(cfg.max_radius_growth_m, peer.speed_mps * peer.age_s)
            tracks.append(
                (
                    north + vn * peer.age_s,
                    east + ve * peer.age_s,
                    vn,
                    ve,
                    cfg.safety_radius_m + growth,
                )
            )
        return tracks

    def _wanted(self, vehicle: Vehicle, output: Output) -> tuple[float, float]:
        """Return the speed and course ``output`` asks for."""
        assert vehicle.heading_rad is not None and vehicle.lat is not None
        if output.mode == POSITION and vehicle.lon is not None:
            north, east = gps_to_local_ned(output.lat, output.lon, vehicle.lat, vehicle.lon)
            if math.hypot(north, east) < 1.0:
                return 0.0, vehicle.heading_rad
            return self.config.cruise_speed_mps, math.atan2(east, north)
        if output.mode == VELOCITY:
            return max(0.0, output.speed_mps), vehicle.heading_rad
        return 0.0, vehicle.heading_rad

    def _separation(
        self,
        tracks: Sequence[tuple[float, float, float, float, float]],
        speed: float,
        course: float,
        horizon: float,
    ) -> float:
        """Return the worst margin over the safety radii; negative means a conflict."""
        vn = speed * math.cos(course)
        ve = speed * math.sin(course)
        return min(
            _closest_approach(north, east, pvn - vn, pve - ve, horizon) - radius
            for north, east, pvn, pve, radius in tracks
        )

    def _choose(
        self,
        tracks: Sequence[tuple[float, float, float, float, float]],
        vehicle: Vehicle,
        speed: float,
        course: float,
        fence: Fence,
    ) -> tuple[float, float]:
        """Return the (speed, course) nearest the wanted one that clears every peer."""
        horizon = self.config.horizon_s
        fallback: tuple[float, float] = (0.0, course)
        fallback_score = self._separation(tracks, 0.0, course, _SCORE_AHEAD_S)
        for factor in _SPEED_FACTORS:
            for offset in _HEADING_OFFSETS_DEG:
                candidate_speed = speed * factor
                candidate = wrap_pi(course + math.radians(offset))
                if not self._stays_inside(vehicle, candidate_speed, candidate, fence):
                    continue
                if self._separation(tracks, candidate_speed, candidate, horizon) >= 0.0:
                    return candidate_speed, candidate
                score = self._separation(tracks, candidate_speed, candidate, _SCORE_AHEAD_S)
                if score > fallback_score:
                    fallback, fallback_score = (candidate_speed, candidate), score
        return fallback

    def _stays_inside(self, vehicle: Vehicle, speed: float, course: float, fence: Fence) -> bool:
        assert vehicle.lat is not None and vehicle.lon is not None
        north = speed * math.cos(course) * _FENCE_LOOKAHEAD_S
        east = speed * math.sin(course) * _FENCE_LOOKAHEAD_S
        lat = vehicle.lat + math.degrees(north / 6_371_000.0)
        lon = vehicle.lon + math.degrees(east / (6_371_000.0 * math.cos(math.radians(vehicle.lat))))
        return fence.contains(lat, lon)
