"""Positions, fences and polygons used by plans and behaviours. Free of ROS."""

from __future__ import annotations

import math
from dataclasses import dataclass

from core_common.geodesy import EARTH_RADIUS_M, gps_to_local_ned, haversine_m


@dataclass(frozen=True)
class LatLon:
    """A WGS-84 position in degrees."""

    lat: float
    lon: float

    def distance_m(self, other: LatLon) -> float:
        """Return the great-circle distance to ``other`` in metres."""
        return haversine_m(self.lat, self.lon, other.lat, other.lon)

    def to_list(self) -> list[float]:
        """Return ``[lat, lon]``, the form used on the wire."""
        return [self.lat, self.lon]


@dataclass(frozen=True)
class Geofence:
    """Hard circular limit the helm enforces; leaving it ends the engagement."""

    center: LatLon
    radius_m: float

    def contains(self, p: LatLon) -> bool:
        """Return whether ``p`` lies inside the circle."""
        return self.center.distance_m(p) <= self.radius_m


@dataclass(frozen=True)
class Polygon:
    """Usable water drawn by the operator; leaving it aborts the current trial."""

    vertices: tuple[LatLon, ...]

    def contains(self, p: LatLon) -> bool:
        """Return whether ``p`` lies inside, by ray casting in a local metric frame.

        Points exactly on an edge may fall either way; the plan check keeps a
        margin from the edge for anything the vehicle must reach.
        """
        origin = self.vertices[0]
        pts = [gps_to_local_ned(v.lat, v.lon, origin.lat, origin.lon) for v in self.vertices]
        y, x = gps_to_local_ned(p.lat, p.lon, origin.lat, origin.lon)
        inside = False
        j = len(pts) - 1
        for i in range(len(pts)):
            yi, xi = pts[i]
            yj, xj = pts[j]
            if (yi > y) != (yj > y):
                x_cross = xi + (y - yi) * (xj - xi) / (yj - yi)
                if x < x_cross:
                    inside = not inside
            j = i
        return inside

    def edge_distance_m(self, p: LatLon) -> float:
        """Return the distance from ``p`` to the nearest edge, in metres."""
        origin = self.vertices[0]
        pts = [gps_to_local_ned(v.lat, v.lon, origin.lat, origin.lon) for v in self.vertices]
        py, px = gps_to_local_ned(p.lat, p.lon, origin.lat, origin.lon)
        best = math.inf
        for i in range(len(pts)):
            ay, ax = pts[i]
            by, bx = pts[(i + 1) % len(pts)]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
            best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
        return best


def offset(origin: LatLon, north_m: float, east_m: float) -> LatLon:
    """Return ``origin`` moved by metres north and east."""
    lat = origin.lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = origin.lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin.lat))))
    return LatLon(lat, lon)


def bearing_rad(src: LatLon, dst: LatLon) -> float:
    """Return the bearing from ``src`` to ``dst`` (NED, clockwise from north) in radians."""
    north, east = gps_to_local_ned(dst.lat, dst.lon, src.lat, src.lon)
    return math.atan2(east, north)
