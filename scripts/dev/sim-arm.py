#!/usr/bin/env python3
"""Arm or disarm the simulated vehicle.

    ./scripts/dev/sim-arm.py
    ./scripts/dev/sim-arm.py --disarm

Stands in for the operator, like sim-set-mode.py: the gateway never arms.
Connects on the SITL operator link (TCP 5762), never on the backseat link.

Against a real vehicle, use the RC transmitter or QGroundControl instead.

Requires pymavlink:  pip install pymavlink
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit('pymavlink is not installed. Install it with: pip install pymavlink')

# The operator link also carries heartbeats from component 0; only the autopilot's count.
AUTOPILOT = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1


def is_armed(heartbeat: mavutil.mavlink.MAVLink_heartbeat_message) -> bool:
    """Read the armed state from a HEARTBEAT."""
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def main() -> None:
    """Request arming until the HEARTBEAT shows the new state, or time out."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    parser.add_argument('--disarm', action='store_true', help='Disarm instead of arm.')
    parser.add_argument(
        '--endpoint',
        default='tcp:127.0.0.1:5762',
        help='SITL operator link (default: %(default)s).',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=60.0,
        help='Seconds to keep retrying (default: %(default)s).',
    )
    parser.add_argument(
        '--retry-period',
        type=float,
        default=1.0,
        help='Seconds between requests (default: %(default)s).',
    )
    args = parser.parse_args()
    arm = not args.disarm
    verb = 'armed' if arm else 'disarmed'

    print(f'Connecting to {args.endpoint} ...')
    link = mavutil.mavlink_connection(args.endpoint)
    if link.wait_heartbeat(timeout=args.timeout) is None:
        sys.exit(f'No heartbeat from {args.endpoint}. Is the SITL container running?')
    print(f'Connected to system {link.target_system}.')

    # Prearm checks fail for a while after boot; ArduPilot gives the reason
    # only in STATUSTEXT, so print each distinct one while retrying.
    seen_texts: set[str] = set()
    last_text = ''
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        link.mav.command_long_send(
            link.target_system,
            AUTOPILOT,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        period_end = time.monotonic() + args.retry_period
        while time.monotonic() < period_end:
            message = link.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=0.2)
            if message is None:
                continue
            if message.get_type() == 'STATUSTEXT':
                last_text = message.text
                if last_text not in seen_texts:
                    seen_texts.add(last_text)
                    print(f'  {last_text}')
            elif (
                message.get_srcSystem() == link.target_system
                and message.get_srcComponent() == AUTOPILOT
                and is_armed(message) == arm
            ):
                print(f'Vehicle is {verb}.')
                return

    sys.exit(f'Vehicle not {verb} within {args.timeout}s. Last message: {last_text or "none"}')


if __name__ == '__main__':
    main()
