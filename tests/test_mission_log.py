"""Tests of the pure parts of mission recording."""

import json
from pathlib import Path

from core_ops.mission_log import (
    ERROR,
    OK,
    STALE,
    WARN,
    ClockPairs,
    HealthBoard,
    Repeats,
    SyncedLog,
)


def test_synced_log_appends_json_lines(tmp_path: Path) -> None:
    log = SyncedLog(tmp_path / 'x.jsonl')
    log.write({'a': 1})
    log.write({'a': 2})
    log.close()
    log.write({'a': 3})
    lines = (tmp_path / 'x.jsonl').read_text().splitlines()
    assert [json.loads(line)['a'] for line in lines] == [1, 2]


def test_synced_log_reaches_the_file_before_close(tmp_path: Path) -> None:
    log = SyncedLog(tmp_path / 'x.jsonl')
    log.write({'a': 1})
    assert (tmp_path / 'x.jsonl').read_text().strip() == '{"a": 1}'
    log.close()


def test_repeats_pass_once_per_period_and_count_the_rest() -> None:
    repeats = Repeats(period_s=30.0)
    assert repeats.admit('cam', 'frame 1 lost', 0.0) == 0
    assert repeats.admit('cam', 'frame 2 lost', 1.0) is None
    assert repeats.admit('cam', 'frame 3 lost', 2.0) is None
    assert repeats.admit('cam', 'frame 4 lost', 31.0) == 2


def test_repeats_keep_sources_apart() -> None:
    repeats = Repeats()
    assert repeats.admit('a', 'lost', 0.0) == 0
    assert repeats.admit('b', 'lost', 0.0) == 0


def test_health_reports_only_changes() -> None:
    board = HealthBoard()
    assert board.report('cam', OK, 'fine', 0.0) is not None
    assert board.report('cam', OK, 'fine', 1.0) is None
    previous = board.report('cam', ERROR, 'broken', 2.0)
    assert previous is not None
    assert previous.level == OK


def test_silent_module_goes_stale_once() -> None:
    board = HealthBoard(stale_s=5.0)
    board.report('cam', OK, 'fine', 0.0)
    assert board.sweep(4.0) == []
    assert board.sweep(6.0) == ['cam']
    assert board.modules['cam'].level == STALE
    assert board.sweep(7.0) == []
    assert board.report('cam', WARN, 'back', 8.0) is not None


def test_clock_pairs_log_once_a_period_and_on_a_jump() -> None:
    pairs = ClockPairs(period_s=60.0, jump_s=1.0)
    assert pairs.update(100.0, 160.0, 0.0)
    assert pairs.offset_s == 60.0
    assert not pairs.update(110.0, 170.0, 10.0)
    assert pairs.update(120.0, 190.0, 20.0)
    assert not pairs.update(130.0, 200.0, 30.0)
    assert pairs.update(190.0, 260.0, 90.0)
