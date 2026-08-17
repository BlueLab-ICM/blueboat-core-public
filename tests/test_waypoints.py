"""
Tests for the survey pattern generator and the shipped waypoint files.

These need only pytest and the standard library — no ROS — so they run on a
laptop as well as inside the brain container.
"""

import importlib.util
import json
import math
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR = REPO_ROOT / 'scripts' / 'generate-waypoints.py'
WAYPOINT_DIR = REPO_ROOT / 'deployment' / 'waypoints'


def _load_generator():
    """Import the generator despite its hyphenated, non-importable filename."""
    spec = importlib.util.spec_from_file_location('generate_waypoints', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gen = _load_generator()


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

def test_north_offset_does_not_move_east():
    lat, lon = gen.local_to_gps(100.0, 0.0, 41.362628, 2.186193)
    assert lat > 41.362628
    assert lon == pytest.approx(2.186193, abs=1e-12)


def test_longitude_convergence_is_applied():
    """East offsets need the cos(latitude) correction.

    Dropping it stretches every pattern east-west — by about 25% at the
    latitude of Barcelona — which quietly pushes the outer waypoints past a
    geofence sized from the intended dimensions.
    """
    origin_lat = 41.362628
    lat_north, _ = gen.local_to_gps(100.0, 0.0, origin_lat, 2.186193)
    _, lon_east = gen.local_to_gps(0.0, 100.0, origin_lat, 2.186193)
    assert (lon_east - 2.186193) == pytest.approx(
        (lat_north - origin_lat) / math.cos(math.radians(origin_lat)), rel=1e-9)


# ---------------------------------------------------------------------------
# Patterns
# ---------------------------------------------------------------------------

def test_lawnmower_stays_within_requested_extent():
    for north, east in gen.lawnmower(width_m=80.0, height_m=60.0, spacing_m=12.0):
        assert abs(east) <= 40.0 + 1e-6
        assert abs(north) <= 30.0 + 1e-6


def test_lawnmower_alternates_leg_direction():
    """Consecutive tracks must run in opposite directions.

    Without alternation the pattern becomes separate passes, each needing a
    transit back to its start — roughly double the distance travelled.
    """
    points = gen.lawnmower(width_m=48.0, height_m=60.0, spacing_m=12.0)
    legs = [points[i:i + 2] for i in range(0, len(points), 2)]
    assert len(legs) >= 3
    for first, second in zip(legs, legs[1:]):
        assert (first[1][0] > first[0][0]) != (second[1][0] > second[0][0])


def test_lawnmower_track_spacing_matches_request():
    points = gen.lawnmower(width_m=48.0, height_m=40.0, spacing_m=12.0)
    eastings = sorted({round(east, 6) for _, east in points})
    assert all((b - a) == pytest.approx(12.0) for a, b in zip(eastings, eastings[1:]))


def test_spiral_never_exceeds_outer_radius():
    """The outer radius is what the geofence gets sized against."""
    points = gen.spiral(radius_m=50.0, arm_spacing_m=10.0,
                        point_spacing_m=12.0, start_radius_m=7.0)
    assert points
    assert all(math.hypot(n, e) <= 50.0 + 1e-6 for n, e in points)


def test_spiral_radius_increases_monotonically():
    points = gen.spiral(radius_m=50.0, arm_spacing_m=10.0,
                        point_spacing_m=12.0, start_radius_m=7.0)
    radii = [math.hypot(n, e) for n, e in points]
    assert all(b >= a for a, b in zip(radii, radii[1:]))


def test_spiral_starts_at_requested_radius():
    """Starting at zero demands a turn tighter than the hull can hold."""
    points = gen.spiral(radius_m=50.0, arm_spacing_m=10.0,
                        point_spacing_m=12.0, start_radius_m=7.0)
    assert math.hypot(*points[0]) == pytest.approx(7.0, abs=0.5)


# ---------------------------------------------------------------------------
# The shipped files — the contract the brain reads back
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('path', sorted(WAYPOINT_DIR.glob('*.json')),
                         ids=lambda p: p.name)
def test_bundled_waypoint_file_is_loadable(path):
    document = json.loads(path.read_text())
    waypoints = document['waypoints']
    assert waypoints, f'{path.name} contains no waypoints'
    for waypoint in waypoints:
        # These two keys are the entire contract with the brain's loader.
        assert -90.0 <= waypoint['latitude'] <= 90.0
        assert -180.0 <= waypoint['longitude'] <= 180.0


@pytest.mark.parametrize('path', sorted(WAYPOINT_DIR.glob('*.json')),
                         ids=lambda p: p.name)
def test_bundled_pattern_fits_a_sensible_geofence(path):
    """Report each pattern's extent, and keep it inside the default fence.

    The brain refuses to start a mission whose waypoints fall outside
    geofence_radius_m, so a bundled example that does not fit the 100 m default
    would fail the moment someone tried it.
    """
    document = json.loads(path.read_text())
    centre = document['metadata']['center']
    furthest = max(
        math.hypot(
            math.radians(w['latitude'] - centre['lat']) * 6_371_000.0,
            math.radians(w['longitude'] - centre['lon']) * 6_371_000.0
            * math.cos(math.radians(centre['lat'])))
        for w in document['waypoints'])
    assert furthest < 100.0, (
        f'{path.name}: furthest waypoint is {furthest:.1f} m from the centre, '
        'outside the default geofence_radius_m of 100 m')
