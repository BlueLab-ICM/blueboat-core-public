#!/usr/bin/env python3
"""
Set the ArduPilot flight mode on the simulated vehicle.

    ./scripts/sim-set-mode.py GUIDED
    ./scripts/sim-set-mode.py MANUAL

This stands in for the RC transmitter or ground station. The backseat is
architecturally forbidden from setting the flight mode — that authority stays
with the operator — so something has to play the operator's part in
simulation, and this is it. It connects on the SITL operator link (TCP 5762),
never on the backseat link the gateway uses.

Against a real vehicle, use the RC transmitter or QGroundControl instead.

Requires pymavlink:  pip install pymavlink
"""

import argparse
import sys

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit('pymavlink is not installed. Install it with: pip install pymavlink')

# ArduRover mode name to custom_mode number (MAV_TYPE_GROUND_ROVER).
# Explicit integers are used for the same reason as in gateway_node.py:
# pymavlink's set_mode_apm silently no-ops when its vehicle-type lookup fails.
ROVER_MODES = {
    'MANUAL': 0,
    'ACRO': 1,
    'STEERING': 3,
    'HOLD': 4,
    'LOITER': 5,
    'FOLLOW': 6,
    'SIMPLE': 7,
    'AUTO': 10,
    'RTL': 11,
    'SMART_RTL': 12,
    'GUIDED': 15,
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('mode', choices=sorted(ROVER_MODES), help='Flight mode to set.')
    parser.add_argument('--endpoint', default='tcp:127.0.0.1:5762',
                        help='SITL operator link (default: %(default)s). SITL serial0 is\n'
                             'redirected to the gateway over UDP, so the operator\n'
                             'link is SERIAL1 on TCP 5762, not the usual 5760.')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Seconds to wait for the mode change (default: %(default)s).')
    args = parser.parse_args()

    print(f'Connecting to {args.endpoint} ...')
    link = mavutil.mavlink_connection(args.endpoint)
    heartbeat = link.wait_heartbeat(timeout=args.timeout)
    if heartbeat is None:
        sys.exit(f'No heartbeat from {args.endpoint}. Is the SITL container running?')
    print(f'Connected to system {link.target_system}, component {link.target_component}.')

    target = ROVER_MODES[args.mode]
    link.mav.set_mode_send(
        link.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        target)

    # Confirm from telemetry rather than trusting the send: ArduPilot rejects
    # a mode it cannot enter (GUIDED before EKF convergence, for one) and says
    # so only by leaving custom_mode unchanged.
    deadline = args.timeout
    while deadline > 0:
        message = link.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        deadline -= 1.0
        if message is None:
            continue
        if int(message.custom_mode) == target:
            print(f'Mode is now {args.mode}.')
            return

    sys.exit(
        f'Mode did not change to {args.mode} within {args.timeout}s. '
        'ArduPilot may still be waiting for EKF/GPS convergence — '
        'give the simulator ~30 s after start and try again.')


if __name__ == '__main__':
    main()
