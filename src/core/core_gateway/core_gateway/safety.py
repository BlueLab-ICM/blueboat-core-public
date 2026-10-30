"""Gateway command rules: authorized source, GUIDED gate and autonomy lease.

Free of ROS and pymavlink so the tests run without either.
"""

# ArduRover custom_mode numbers.
ROVER_MODE = {
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
ROVER_MODE_NAME_BY_NUM = {value: key for key, value in ROVER_MODE.items()}

AUTHORIZED_SOURCE = 'core_helm'


def is_guided(mode_num: int | None) -> bool:
    """Check whether the autopilot is in GUIDED; None (no HEARTBEAT yet) is not."""
    return mode_num == ROVER_MODE['GUIDED']


class AutonomyLease:
    """Track whether the helm may drive.

    Acquired by an authorized command in GUIDED, renewed by every later
    authorized command, revoked after more than ``timeout_s`` of silence.
    Timestamps are seconds from one monotonic clock.
    """

    def __init__(self, timeout_s: float = 2.0) -> None:
        self.timeout_s = timeout_s
        self.active = False
        self.last_renewed = 0.0  # read only while active

    def on_command(self, source: str, now: float, mode_num: int | None) -> bool:
        """Renew or acquire the lease; return False for an unauthorized source.

        A rejected source does not renew the lease, so no other node can hold
        the failsafe open.
        """
        if source != AUTHORIZED_SOURCE:
            return False
        self.last_renewed = now
        if not self.active and is_guided(mode_num):
            self.active = True
        return True

    def allows_send(self, now: float, mode_num: int | None) -> bool:
        """Check for a fresh lease and GUIDED before a command reaches the autopilot.

        Staleness is checked here as well as in ``tick``, because ``tick`` runs at 1 Hz.
        """
        return self.active and not self._is_stale(now) and is_guided(mode_num)

    def tick(self, now: float) -> bool:
        """Revoke a stale lease; return False only on the tick that revokes it."""
        if self.active and self._is_stale(now):
            self.active = False
            return False
        return True

    def _is_stale(self, now: float) -> bool:
        return (now - self.last_renewed) > self.timeout_s
