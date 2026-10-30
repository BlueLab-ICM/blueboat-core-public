"""Geodetic helpers, free of ROS so they run anywhere, including on a laptop reading a log."""

import math

EARTH_RADIUS_M = 6_371_000.0


def wrap_pi(angle: float) -> float:
    """Normalize any angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return the great-circle distance in metres between two WGS-84 coordinates."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def gps_to_local_ned(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Project a GPS fix to local metres, as (north, east).

    Flat-earth: accurate to well under a metre over a few hundred metres.
    """
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east = math.radians(lon - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return north, east
