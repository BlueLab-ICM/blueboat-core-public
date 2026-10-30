"""Collect the autopilot's parameter list into one snapshot. Free of ROS."""

# The list is finished when nothing has arrived for this long, even with some missing.
IDLE_DONE_S = 10.0


class ParamCollector:
    """Hold PARAM_VALUE messages until the list is complete or goes quiet."""

    def __init__(self) -> None:
        self.values: dict[str, float] = {}
        self.expected = 0
        self.published = False
        self._last = 0.0

    def add(self, name: str, value: float, count: int, now: float) -> None:
        """Take one PARAM_VALUE."""
        if name:
            self.values[name] = value
            self.expected = max(self.expected, count)
            self._last = now

    @property
    def missing(self) -> int:
        """Return how many expected parameters have not arrived."""
        return max(0, self.expected - len(self.values))

    def ready(self, now: float) -> bool:
        """Return whether the snapshot should go out now."""
        if self.published or self.expected <= 0:
            return False
        return self.missing == 0 or now - self._last > IDLE_DONE_S

    def snapshot(self) -> dict[str, object]:
        """Return the snapshot as a JSON-ready dict."""
        return {'expected': self.expected, 'values': self.values}
