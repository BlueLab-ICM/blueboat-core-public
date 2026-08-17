"""
Tests for the geodetic helpers the geofence depends on.

`core_autonomy.geodesy` is deliberately free of ROS imports, so these run on a
plain laptop as well as inside the brain container — which is the point of
keeping the geometry in its own module.
"""

import importlib.util
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
GEODESY = REPO_ROOT / 'src/core/core_autonomy/core_autonomy/geodesy.py'


def _load_geodesy():
    spec = importlib.util.spec_from_file_location('geodesy', GEODESY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


brain = _load_geodesy()

# Port of Barcelona test site.
ORIGIN_LAT, ORIGIN_LON = 41.362628, 2.186193


# ---------------------------------------------------------------------------
# haversine_m — every geofence decision rests on this
# ---------------------------------------------------------------------------

def test_identical_points_are_zero_apart():
    assert brain.haversine_m(ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, ORIGIN_LON) == 0.0


def test_distance_is_symmetric():
    forward = brain.haversine_m(ORIGIN_LAT, ORIGIN_LON, 41.3630, 2.1870)
    backward = brain.haversine_m(41.3630, 2.1870, ORIGIN_LAT, ORIGIN_LON)
    assert forward == pytest.approx(backward)


def test_matches_a_known_short_distance():
    """One arcsecond of latitude is about 30.9 m anywhere on the sphere."""
    one_arcsecond = 1.0 / 3600.0
    distance = brain.haversine_m(
        ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT + one_arcsecond, ORIGIN_LON)
    assert distance == pytest.approx(30.9, abs=0.2)


def test_longitude_degrees_are_shorter_than_latitude_degrees():
    """The cos(latitude) convergence must be applied.

    Omitting it makes an east-west geofence roughly 25% too large at the
    latitude of Barcelona — a fence that does not fence.
    """
    north = brain.haversine_m(ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT + 0.001, ORIGIN_LON)
    east = brain.haversine_m(ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, ORIGIN_LON + 0.001)
    assert east == pytest.approx(north * math.cos(math.radians(ORIGIN_LAT)), rel=1e-3)


def test_antimeridian_does_not_invert_the_fence():
    """Points either side of 180 deg are close, not a circumference apart.

    Naive coordinate subtraction reports about 40 000 km here. No BlueBoat has
    crossed the antimeridian yet, but a geofence that inverts there would be a
    silent trap for whoever does.
    """
    assert brain.haversine_m(0.0, 179.999, 0.0, -179.999) < 250.0


# ---------------------------------------------------------------------------
# gps_to_local_ned — used for the Foxglove/RViz markers
# ---------------------------------------------------------------------------

def test_origin_projects_to_zero():
    north, east = brain.gps_to_local_ned(
        ORIGIN_LAT, ORIGIN_LON, ORIGIN_LAT, ORIGIN_LON)
    assert north == pytest.approx(0.0)
    assert east == pytest.approx(0.0)


def test_signs_follow_the_ned_convention():
    """North is +north and east is +east — a sign slip mirrors every plot."""
    north, east = brain.gps_to_local_ned(
        ORIGIN_LAT + 0.001, ORIGIN_LON + 0.001, ORIGIN_LAT, ORIGIN_LON)
    assert north > 0
    assert east > 0


def test_projection_agrees_with_the_range_check():
    """The marker geometry must not disagree with the fence it draws.

    The visualisation projects to flat metres while the geofence uses the
    great-circle distance. If the two drifted apart, the drawn circle would
    stop matching the boundary actually being enforced.
    """
    target_lat, target_lon = ORIGIN_LAT + 0.0005, ORIGIN_LON + 0.0007
    north, east = brain.gps_to_local_ned(
        target_lat, target_lon, ORIGIN_LAT, ORIGIN_LON)
    projected = math.hypot(north, east)
    great_circle = brain.haversine_m(
        ORIGIN_LAT, ORIGIN_LON, target_lat, target_lon)
    assert projected == pytest.approx(great_circle, rel=1e-4)
