"""Pure parts of mission recording: synced JSONL, health tracking, clock pairs. Free of ROS."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

OK, WARN, ERROR, STALE = 0, 1, 2, 3
LEVEL_NAMES = {OK: 'ok', WARN: 'warn', ERROR: 'error', STALE: 'stale'}
SCHEMA_VERSION = 2


def mounted(path: Path) -> bool:
    """Return whether ``path`` or an ancestor below / is a mountpoint.

    Without a mount a file lands in the container's own filesystem and vanishes with it.
    """
    p = path
    while p != p.parent:
        if os.path.ismount(p):
            return True
        p = p.parent
    return False


class SyncedLog:
    """Append JSON lines. ``sync`` every line, or every ``sync_every`` lines."""

    def __init__(self, path: Path, sync_every: int = 1) -> None:
        self.path = path
        self.sync_every = max(1, sync_every)
        self._unsynced = 0
        self._file: TextIO | None = open(path, 'a', buffering=1, encoding='utf-8')  # noqa: SIM115

    def write(self, record: dict[str, Any]) -> None:
        """Append one record."""
        if self._file is None:
            return
        self._file.write(json.dumps(record) + '\n')
        self._unsynced += 1
        if self._unsynced >= self.sync_every:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._unsynced = 0

    def close(self) -> None:
        """Sync and close."""
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._file.close()
            self._file = None


class Repeats:
    """Let a repeated log line through once per ``period_s``, counting the rest."""

    def __init__(self, period_s: float = 30.0) -> None:
        self.period_s = period_s
        self._seen: dict[tuple[str, str], tuple[float, int]] = {}

    def admit(self, source: str, text: str, now: float) -> int | None:
        """Return how many copies were held back if this line should pass, else None."""
        key = (source, re.sub(r'[-0-9.]+', '#', text)[:120])
        last = self._seen.get(key)
        if last is not None and now - last[0] < self.period_s:
            self._seen[key] = (last[0], last[1] + 1)
            return None
        held = last[1] if last is not None else 0
        self._seen[key] = (now, 0)
        return held


@dataclass
class ModuleState:
    """Last report of one module."""

    level: int
    message: str
    heard: float


class HealthBoard:
    """Track every module's health. A module that stops reporting goes stale."""

    def __init__(self, stale_s: float = 5.0) -> None:
        self.stale_s = stale_s
        self.modules: dict[str, ModuleState] = {}

    def report(self, name: str, level: int, message: str, now: float) -> ModuleState | None:
        """Take one report; return the previous state if the level changed."""
        previous = self.modules.get(name)
        self.modules[name] = ModuleState(level, message, now)
        if previous is None or previous.level != level:
            return previous or ModuleState(-1, '', now)
        return None

    def sweep(self, now: float) -> list[str]:
        """Mark silent modules stale; return the names that just went stale."""
        stale = []
        for name, state in self.modules.items():
            if state.level != STALE and now - state.heard > self.stale_s:
                state.level = STALE
                state.message = f'no report for {self.stale_s:.0f} s'
                stale.append(name)
        return stale


class ClockPairs:
    """Compare the backseat clock with the autopilot's GPS time."""

    def __init__(self, period_s: float = 60.0, jump_s: float = 1.0) -> None:
        self.period_s = period_s
        self.jump_s = jump_s
        self.offset_s: float | None = None
        self._logged: tuple[float, float] | None = None

    def update(self, system_s: float, gps_s: float, now: float) -> bool:
        """Take one pair; return whether it should be logged.

        A pair goes in the log once a period, and at once if the offset moved, so every
        logged time in a mission can be corrected to GPS time afterwards.
        """
        self.offset_s = gps_s - system_s
        if self._logged is None:
            self._logged = (now, self.offset_s)
            return True
        last_time, last_offset = self._logged
        if now - last_time >= self.period_s or abs(self.offset_s - last_offset) >= self.jump_s:
            self._logged = (now, self.offset_s)
            return True
        return False
