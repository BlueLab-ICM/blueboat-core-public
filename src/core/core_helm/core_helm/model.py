"""Value types of the helm. Free of ROS."""

from __future__ import annotations

import math
from dataclasses import dataclass

from core_common.geodesy import haversine_m

KEEPALIVE, POSITION, VELOCITY, HOLD = 0, 1, 2, 3


@dataclass(frozen=True)
class Fence:
    """A disk the vehicle may not leave. Past ``radius_m + abort_margin_m`` the helm stops."""

    lat: float
    lon: float
    radius_m: float
    abort_margin_m: float = 5.0

    def valid(self) -> bool:
        """Return whether the fence is usable."""
        return math.isfinite(self.lat) and math.isfinite(self.lon) and self.radius_m > 0.0

    def contains(self, lat: float, lon: float) -> bool:
        """Return whether a target lies inside the nominal radius."""
        return haversine_m(lat, lon, self.lat, self.lon) <= self.radius_m

    def breached(self, lat: float, lon: float) -> bool:
        """Return whether the vehicle is past the abort radius."""
        return haversine_m(lat, lon, self.lat, self.lon) > self.radius_m + self.abort_margin_m


@dataclass(frozen=True)
class Limits:
    """Hard limits on any velocity command."""

    max_speed_mps: float = 2.5
    max_yaw_rate_rps: float = 2.0


@dataclass(frozen=True)
class Vehicle:
    """What the helm knows about the vehicle this tick."""

    lat: float | None
    lon: float | None
    heading_rad: float | None
    speed_mps: float
    guided: bool
    gps_age_s: float


@dataclass(frozen=True)
class Output:
    """The one command of this tick."""

    mode: int
    lat: float = 0.0
    lon: float = 0.0
    speed_mps: float = 0.0
    yaw_rate_rps: float = 0.0
