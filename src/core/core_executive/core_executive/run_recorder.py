"""On-disk record of one executive run: experiment_logs/executive/<stamp>_<plan name>/.

plan.json, events.jsonl, results.json, detections.jsonl, and frames/ with frames.jsonl when
frames are saved. ``t`` is the vehicle clock; plan.json holds its offset to the laptop's UTC.
Events and results are synced to disk, because a power cut loses what is only in the OS cache.
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
from collections.abc import Callable
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO


def _dir_is_mounted(path: Path) -> bool:
    """Return whether `path` or any ancestor below / is a mountpoint."""
    p = path
    while p != p.parent:
        if os.path.ismount(p):
            return True
        p = p.parent
    return False


def mounted_log_root(path: Path, *, require_mount: bool = True) -> Path | None:
    """Return ``path`` if it is writable (and on a bind mount when required).

    Without the mount check a run would write into the container's own
    filesystem and vanish when the container is recreated.
    """
    if require_mount and not _dir_is_mounted(path):
        return None
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / '.executive_write_test'
        probe.touch()
        probe.unlink()
    except OSError:
        return None
    return path


class RunRecorder:
    """Write the files of one run. Not thread-safe except :meth:`save_frame`."""

    def __init__(
        self,
        root: Path,
        plan_name: str,
        clock: Callable[[], float] = time.time,
        mono: Callable[[], float] = time.monotonic,
    ) -> None:
        stamp = datetime.fromtimestamp(clock(), tz=timezone.utc).strftime('%Y%m%d_%H%M%S')
        # The vehicle clock is stale at sea and restarts from the same saved
        # time after a power cycle, so a stamp can repeat: never reuse a folder.
        base = root / 'executive' / f'{stamp}_{plan_name}'
        self.dir = base
        n = 1
        while True:
            try:
                self.dir.mkdir(parents=True, exist_ok=False)
                break
            except FileExistsError:
                n += 1
                self.dir = base.with_name(f'{base.name}_{n}')
        self._clock = clock
        self._mono = mono
        self._events = self._open('events.jsonl')
        self._detections = self._open('detections.jsonl')
        self._frames_index: TextIO | None = None
        self._frame_queue: queue.Queue[tuple[bytes, dict[str, Any]] | None] = queue.Queue(
            maxsize=200
        )
        self._frame_thread: threading.Thread | None = None
        self.frames_saved = 0
        self.frames_dropped = 0
        self.detections_logged = 0

    def _open(self, name: str) -> TextIO:
        # buffering=1: line-buffered, so every record reaches the OS on its newline.
        return open(self.dir / name, 'a', buffering=1, encoding='utf-8')

    def _stamp(self, record: dict[str, Any]) -> dict[str, Any]:
        return {'t': round(self._clock(), 3), 't_mono': round(self._mono(), 3), **record}

    def write_plan(self, plan: dict[str, Any], extra: dict[str, Any]) -> None:
        """Write plan.json: the uploaded plan plus run metadata."""
        doc = {'plan': plan, **extra}
        (self.dir / 'plan.json').write_text(json.dumps(doc, indent=2) + '\n', encoding='utf-8')

    def event(self, kind: str, **fields: object) -> dict[str, Any]:
        """Append one event and return the record written (also after :meth:`close`)."""
        record = self._stamp({'kind': kind, **fields})
        line = json.dumps(record) + '\n'
        if self._events.closed:
            with open(self.dir / 'events.jsonl', 'a', encoding='utf-8') as handle:
                handle.write(line)
                os.fsync(handle.fileno())
        else:
            self._events.write(line)
            self._events.flush()
            os.fsync(self._events.fileno())
        return record

    def detection(self, payload: dict[str, Any], context: dict[str, Any]) -> None:
        """Append one detector message, with the boat state."""
        self._detections.write(json.dumps(self._stamp({**context, 'msg': payload})) + '\n')
        self.detections_logged += 1

    def write_results(self, results: list[dict[str, Any]]) -> None:
        """Replace results.json atomically with the current trial results."""
        tmp = self.dir / 'results.json.tmp'
        with open(tmp, 'w', encoding='utf-8') as handle:
            handle.write(json.dumps(results, indent=2) + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        tmp.replace(self.dir / 'results.json')

    def save_frame(self, jpeg: bytes, meta: dict[str, Any]) -> bool:
        """Queue one JPEG for writing; return False if the queue is full (frame dropped)."""
        if self._frame_thread is None:
            (self.dir / 'frames').mkdir(exist_ok=True)
            self._frames_index = self._open('frames.jsonl')
            self._frame_thread = threading.Thread(target=self._frame_worker, daemon=True)
            self._frame_thread.start()
        try:
            self._frame_queue.put_nowait((jpeg, self._stamp(meta)))
        except queue.Full:
            self.frames_dropped += 1
            return False
        return True

    def _frame_worker(self) -> None:
        while True:
            item = self._frame_queue.get()
            if item is None:
                # The worker owns the index: closing it here cannot race a write.
                if self._frames_index is not None:
                    self._frames_index.close()
                return
            jpeg, meta = item
            name = f'{self.frames_saved:06d}.jpg'
            try:
                (self.dir / 'frames' / name).write_bytes(jpeg)
            except OSError:
                self.frames_dropped += 1
                continue
            self.frames_saved += 1
            if self._frames_index is not None:
                self._frames_index.write(json.dumps({'file': name, **meta}) + '\n')

    def close(self) -> None:
        """Flush the frame queue and close every file.

        Frames still queued are written first; the worker closes the frame
        index itself once the queue is empty.
        """
        if self._frame_thread is not None:
            self._frame_queue.put(None)
            self._frame_thread.join()
        for handle in (self._events, self._detections):
            handle.close()
