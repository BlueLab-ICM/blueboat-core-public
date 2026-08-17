#!/usr/bin/env python3
"""
Generate a waypoint file for a survey pattern.

    ./scripts/generate-waypoints.py lawnmower --lat 41.362628 --lon 2.186193 \
        --width 80 --height 60 --spacing 12 -o deployment/waypoints/survey.json

    ./scripts/generate-waypoints.py spiral --lat 41.362628 --lon 2.186193 \
        --radius 50 --arm-spacing 10 -o deployment/waypoints/spiral.json

Two patterns cover most small-vessel survey work:

* ``lawnmower`` — parallel tracks with alternating direction. The standard
  choice for uniform coverage of a rectangle: bathymetry, photo mosaics, and
  anything where even spacing matters more than reaching a specific point.

* ``spiral`` — an Archimedean spiral outward from a centre. Better when a
  target is believed to be near a known position and search effort should be
  concentrated there first, since the boat covers the centre before the edges.

The output is what the brain's ``waypoint_file`` parameter expects. Only
``latitude``/``longitude`` are read back; the local metric coordinates and the
metadata block are written for plotting and provenance.

Requires only the Python standard library.
"""

import argparse
import json
import math
from datetime import datetime, timezone

EARTH_RADIUS_M = 6_371_000.0


def local_to_gps(
    north_m: float,
    east_m: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Flat-earth offset in metres to WGS-84 degrees.

    The cos(latitude) term is what keeps an east offset the right size: a
    degree of longitude shrinks towards the poles, and omitting it stretches
    every pattern east-west — by about 25% at the latitude of Barcelona.
    """
    lat = origin_lat + math.degrees(north_m / EARTH_RADIUS_M)
    lon = origin_lon + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return lat, lon


def lawnmower(width_m: float, height_m: float, spacing_m: float) -> list[tuple[float, float]]:
    """Parallel north-south tracks, centred on the origin, in (north, east)."""
    points: list[tuple[float, float]] = []
    half_width = width_m / 2.0
    half_height = height_m / 2.0

    east = -half_width
    heading_north = True
    while east <= half_width + 1e-9:
        # Alternating the leg direction is what makes this one continuous
        # track rather than a set of separate passes, each needing a transit
        # back to its start — roughly half the distance travelled.
        leg = [(-half_height, east), (half_height, east)]
        points.extend(leg if heading_north else list(reversed(leg)))
        heading_north = not heading_north
        east += spacing_m
    return points


def spiral(
    radius_m: float,
    arm_spacing_m: float,
    point_spacing_m: float,
    start_radius_m: float,
) -> list[tuple[float, float]]:
    """Archimedean spiral outward from the origin, in (north, east).

    Points are placed at roughly constant arc length rather than constant
    angle, so the boat holds a steady speed instead of crawling through the
    tight inner turns and sprinting along the outer ones.
    """
    points: list[tuple[float, float]] = []
    # r = a·theta, where one full turn (2π) advances r by the arm spacing.
    a = arm_spacing_m / (2.0 * math.pi)
    theta = start_radius_m / a if a > 0 else 0.0

    while True:
        r = a * theta
        if r > radius_m:
            break
        points.append((r * math.cos(theta), r * math.sin(theta)))
        # Arc length is approximately r·dtheta for r >> a, so this keeps the
        # spacing between consecutive points near constant.
        theta += point_spacing_m / max(r, arm_spacing_m / 4.0)
    return points


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Generate a BlueBoat survey waypoint file.')
    parser.add_argument('pattern', choices=['lawnmower', 'spiral'])
    parser.add_argument('--lat', type=float, required=True,
                        help='Centre latitude in decimal degrees.')
    parser.add_argument('--lon', type=float, required=True,
                        help='Centre longitude in decimal degrees.')
    parser.add_argument('-o', '--output', required=True, help='Output JSON path.')

    parser.add_argument('--width', type=float, default=80.0,
                        help='lawnmower: east-west extent in metres.')
    parser.add_argument('--height', type=float, default=60.0,
                        help='lawnmower: north-south extent in metres.')
    parser.add_argument('--spacing', type=float, default=12.0,
                        help='lawnmower: distance between tracks. Set it from '
                             'your sensor swath, with 10-20%% overlap.')

    parser.add_argument('--radius', type=float, default=50.0,
                        help='spiral: outer radius in metres.')
    parser.add_argument('--arm-spacing', type=float, default=10.0,
                        help='spiral: radial distance between successive turns.')
    parser.add_argument('--point-spacing', type=float, default=12.0,
                        help='spiral: approximate distance between waypoints.')
    parser.add_argument('--start-radius', type=float, default=7.0,
                        help='spiral: radius of the first waypoint. Starting at '
                             'zero demands a turn tighter than the hull can hold.')

    args = parser.parse_args()

    if args.pattern == 'lawnmower':
        local_points = lawnmower(args.width, args.height, args.spacing)
        parameters = {'width_m': args.width, 'height_m': args.height,
                      'spacing_m': args.spacing}
    else:
        local_points = spiral(args.radius, args.arm_spacing,
                              args.point_spacing, args.start_radius)
        parameters = {'radius_m': args.radius, 'arm_spacing_m': args.arm_spacing,
                      'point_spacing_m': args.point_spacing,
                      'start_radius_m': args.start_radius}

    waypoints = []
    for index, (north, east) in enumerate(local_points, start=1):
        lat, lon = local_to_gps(north, east, args.lat, args.lon)
        waypoints.append({
            'index': index,
            'latitude': round(lat, 8),
            'longitude': round(lon, 8),
            'north_m': round(north, 3),
            'east_m': round(east, 3),
        })

    document = {
        'metadata': {
            'type': args.pattern,
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'center': {'lat': args.lat, 'lon': args.lon},
            'parameters': parameters,
        },
        'waypoints': waypoints,
    }

    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(document, handle, indent=2)
        handle.write('\n')

    extent = max(math.hypot(n, e) for n, e in local_points) if local_points else 0.0
    print(f'Wrote {len(waypoints)} waypoints to {args.output}')
    print(f'Furthest point is {extent:.1f} m from the centre — set the brain\'s '
          f'geofence_radius_m above this, or the mission will be refused.')


if __name__ == '__main__':
    main()
