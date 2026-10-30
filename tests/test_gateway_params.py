"""Tests of the autopilot parameter snapshot."""

from core_gateway.params import IDLE_DONE_S, ParamCollector


def test_not_ready_before_anything_arrives() -> None:
    assert not ParamCollector().ready(100.0)


def test_ready_when_complete() -> None:
    params = ParamCollector()
    params.add('A', 1.0, 2, 0.0)
    assert not params.ready(0.1)
    params.add('B', 2.0, 2, 0.2)
    assert params.ready(0.2)
    assert params.snapshot() == {'expected': 2, 'values': {'A': 1.0, 'B': 2.0}}


def test_ready_after_going_quiet_with_some_missing() -> None:
    params = ParamCollector()
    params.add('A', 1.0, 5, 0.0)
    assert not params.ready(IDLE_DONE_S - 1.0)
    assert params.ready(IDLE_DONE_S + 1.0)
    assert params.missing == 4


def test_published_once() -> None:
    params = ParamCollector()
    params.add('A', 1.0, 1, 0.0)
    params.published = True
    assert not params.ready(1.0)
