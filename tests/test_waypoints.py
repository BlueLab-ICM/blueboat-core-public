"""Tests for the survey pattern generator and the shipped waypoint files.

These need only pytest and the standard library — no ROS — so they run on a
laptop as well as inside a container.
"""

import importlib.util
import json
import math
import re
from itertools import pairwise
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATOR = REPO_ROOT / 'scripts' / 'dev' / 'generate-waypoints.py'
EXPERIMENT_DIR = REPO_ROOT / 'experiments'
WAYPOINT_DIR = EXPERIMENT_DIR / 'waypoints'


def _load_generator() -> ModuleType:
    """Import the generator despite its hyphenated, non-importable filename."""
    spec = importlib.util.spec_from_file_location('generate_waypoints', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gen = _load_generator()


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------


def test_north_offset_does_not_move_east() -> None:
    lat, lon = gen.local_to_gps(100.0, 0.0, 41.362628, 2.186193)
    assert lat > 41.362628
    assert lon == pytest.approx(2.186193, abs=1e-12)


def test_longitude_convergence_is_applied() -> None:
    """East offsets need the cos(latitude) correction.

    Dropping it stretches every pattern east-west — by about 25% at the
    latitude of Barcelona — which quietly pushes the outer waypoints past a
    geofence sized from the intended dimensions.
    """
    origin_lat = 41.362628
    lat_north, _ = gen.local_to_gps(100.0, 0.0, origin_lat, 2.186193)
    _, lon_east = gen.local_to_gps(0.0, 100.0, origin_lat, 2.186193)
    assert (lon_east - 2.186193) == pytest.approx(
        (lat_north - origin_lat) / math.cos(math.radians(origin_lat)), rel=1e-9
    )


# ---------------------------------------------------------------------------
# Patterns
# ---------------------------------------------------------------------------


def test_lawnmower_stays_within_requested_extent() -> None:
    for north, east in gen.lawnmower(width_m=80.0, height_m=60.0, spacing_m=12.0):
        assert abs(east) <= 40.0 + 1e-6
        assert abs(north) <= 30.0 + 1e-6


def test_lawnmower_alternates_leg_direction() -> None:
    """Consecutive tracks must run in opposite directions.

    Without alternation the pattern becomes separate passes, each needing a
    transit back to its start — roughly double the distance travelled.
    """
    points = gen.lawnmower(width_m=48.0, height_m=60.0, spacing_m=12.0)
    legs = [points[i : i + 2] for i in range(0, len(points), 2)]
    assert len(legs) >= 3
    for first, second in pairwise(legs):
        assert (first[1][0] > first[0][0]) != (second[1][0] > second[0][0])


def test_lawnmower_track_spacing_matches_request() -> None:
    points = gen.lawnmower(width_m=48.0, height_m=40.0, spacing_m=12.0)
    eastings = sorted({round(east, 6) for _, east in points})
    assert all((b - a) == pytest.approx(12.0) for a, b in pairwise(eastings))


def test_spiral_never_exceeds_outer_radius() -> None:
    """The outer radius is what the geofence gets sized against."""
    points = gen.spiral(radius_m=50.0, arm_spacing_m=10.0, point_spacing_m=12.0, start_radius_m=7.0)
    assert points
    assert all(math.hypot(n, e) <= 50.0 + 1e-6 for n, e in points)


def test_spiral_radius_increases_monotonically() -> None:
    points = gen.spiral(radius_m=50.0, arm_spacing_m=10.0, point_spacing_m=12.0, start_radius_m=7.0)
    radii = [math.hypot(n, e) for n, e in points]
    assert all(b >= a for a, b in pairwise(radii))


def test_spiral_starts_at_requested_radius() -> None:
    """Starting at zero demands a turn tighter than the hull can hold."""
    points = gen.spiral(radius_m=50.0, arm_spacing_m=10.0, point_spacing_m=12.0, start_radius_m=7.0)
    assert math.hypot(*points[0]) == pytest.approx(7.0, abs=0.5)


def _centre_of(document: dict) -> dict:
    """Pattern centre, tolerating both metadata shapes in circulation.

    The survey reads only `waypoints[].latitude/longitude` — `metadata` is not
    part of its contract, so generators have drifted. scripts/dev/generate-waypoints.py
    writes `metadata.center`; the web tool that produced baseline.json writes
    `metadata.parameters.centerLat/centerLon`. Falling back to the centroid keeps
    this test useful for a file written by neither.
    """
    metadata = document.get('metadata', {})
    if 'center' in metadata:
        return metadata['center']
    parameters = metadata.get('parameters', {})
    if 'centerLat' in parameters:
        return {'lat': parameters['centerLat'], 'lon': parameters['centerLon']}
    waypoints = document['waypoints']
    return {
        'lat': sum(w['latitude'] for w in waypoints) / len(waypoints),
        'lon': sum(w['longitude'] for w in waypoints) / len(waypoints),
    }


# ---------------------------------------------------------------------------
# The shipped files — the contract the survey reads back
# ---------------------------------------------------------------------------


@pytest.mark.parametrize('path', sorted(WAYPOINT_DIR.glob('*.json')), ids=lambda p: p.name)
def test_bundled_waypoint_file_is_loadable(path: Path) -> None:
    document = json.loads(path.read_text())
    waypoints = document['waypoints']
    assert waypoints, f'{path.name} contains no waypoints'
    for waypoint in waypoints:
        # These two keys are the entire contract with the survey loader.
        assert -90.0 <= waypoint['latitude'] <= 90.0
        assert -180.0 <= waypoint['longitude'] <= 180.0


@pytest.mark.parametrize('path', sorted(WAYPOINT_DIR.glob('*.json')), ids=lambda p: p.name)
def test_bundled_pattern_fits_a_sensible_geofence(path: Path) -> None:
    """Report each pattern's extent, and keep it inside the default fence.

    A plan is refused whose waypoints fall outside
    geofence_radius_m, so a bundled example that does not fit the 100 m default
    would fail the moment someone tried it.
    """
    document = json.loads(path.read_text())
    centre = _centre_of(document)
    furthest = max(
        math.hypot(
            math.radians(w['latitude'] - centre['lat']) * 6_371_000.0,
            math.radians(w['longitude'] - centre['lon'])
            * 6_371_000.0
            * math.cos(math.radians(centre['lat'])),
        )
        for w in document['waypoints']
    )
    assert furthest < 100.0, (
        f'{path.name}: furthest waypoint is {furthest:.1f} m from the centre, '
        'outside the default geofence_radius_m of 100 m'
    )


def _waypoint_files_named_by_experiments() -> list[tuple[str, str]]:
    # A regex, not yaml: CI installs only pytest.
    pattern = re.compile(r'^\s*waypoint_file:\s*["\']?([^"\'\s#]+)', re.MULTILINE)
    return [
        (yaml_path.name, match)
        for yaml_path in sorted(EXPERIMENT_DIR.glob('*.yaml'))
        for match in pattern.findall(yaml_path.read_text())
    ]


@pytest.mark.parametrize(
    ('experiment', 'waypoint_file'),
    _waypoint_files_named_by_experiments(),
    ids=lambda v: v if v.endswith('.yaml') else '',
)
def test_experiment_waypoint_file_exists(experiment: str, waypoint_file: str) -> None:
    """A plan is refused whose waypoint_file is missing."""
    assert not Path(waypoint_file).is_absolute(), (
        f'{experiment}: use a path relative to experiments/'
    )
    assert (EXPERIMENT_DIR / waypoint_file).is_file(), (
        f'{experiment}: waypoint_file {waypoint_file!r} is not in experiments/'
    )
