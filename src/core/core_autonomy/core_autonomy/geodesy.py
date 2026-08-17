"""
Geodetic helpers.

Kept in their own module, free of ROS imports, so they can be tested and reused
without a ROS 2 environment — including from a laptop analysing a mission log.
Every geofence decision in the stack rests on :func:`haversine_m`, so it is
worth being able to exercise it anywhere.
"""

import math

EARTH_RADIUS_M = 6_371_000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two WGS-84 coordinates.

    Used for every range check. The great-circle form rather than a flat
    approximation costs almost nothing here and keeps the result correct
    across the antimeridian and at any latitude.
    """
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
    """Flat-earth projection of a GPS fix to local metres, as (north, east).

    Accurate to well under a metre over the few-hundred-metre areas a BlueBoat
    works in, and far cheaper than a full geodetic projection. Used only for
    visualisation — range checks use :func:`haversine_m`.

    The cos(latitude) term on the east component is not optional: a degree of
    longitude shrinks towards the poles, and omitting it stretches every drawn
    geometry east-west by about 25% at the latitude of Barcelona.
    """
    north = math.radians(lat - origin_lat) * EARTH_RADIUS_M
    east = (
        math.radians(lon - origin_lon)
        * EARTH_RADIUS_M
        * math.cos(math.radians(origin_lat))
    )
    return north, east
