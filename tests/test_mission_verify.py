"""Tests of the mission check."""

import importlib.util
import json
from pathlib import Path
from types import ModuleType

import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/data/mission-verify.py'


def _load() -> ModuleType:
    spec = importlib.util.spec_from_file_location('mission_verify', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


verify = _load()
TOPICS = [
    '/frontseat/gps',
    '/frontseat/heading',
    '/frontseat/mode',
    '/helm/output',
    '/helm/status',
    '/events',
    '/diagnostics',
    '/rosout',
]


def mission(tmp_path: Path, topics: list[str] = TOPICS, gps_valid: bool = True) -> Path:
    (tmp_path / 'manifest.yaml').write_text(
        yaml.safe_dump(
            {
                'ended_at': 'now',
                'recording_enabled': True,
                'modules': {},
                'core': {'dirty': False},
            }
        )
    )
    logs = tmp_path / 'experiment_logs'
    logs.mkdir()
    rows = [{'t_mono': float(i), 'gps_valid': gps_valid} for i in range(10)]
    (logs / 'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in rows) + '\n{"cut')
    events = [{'kind': 'clock', 'level': 'ok', 'fields': {'offset_s': 0.2}}]
    (logs / 'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events) + '\n')
    bag = tmp_path / 'rosbags' / 'rosbag_1'
    bag.mkdir(parents=True)
    info = {
        'rosbag2_bagfile_information': {
            'topics_with_message_count': [
                {'topic_metadata': {'name': t}, 'message_count': 5} for t in topics
            ]
        }
    }
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(info))
    return tmp_path


def names(report: dict, level: str) -> list[str]:
    return [c['name'] for c in report['checks'] if c['level'] == level]


def test_clean_mission_passes(tmp_path: Path) -> None:
    report = verify.verify(mission(tmp_path))
    assert report['result'] == 'pass'


def test_missing_command_topic_fails(tmp_path: Path) -> None:
    topics = [t for t in TOPICS if t != '/helm/output']
    report = verify.verify(mission(tmp_path, topics))
    assert 'topics' in names(report, 'fail')
    assert report['result'] == 'fail'


def test_all_zero_gps_fails(tmp_path: Path) -> None:
    report = verify.verify(mission(tmp_path, gps_valid=False))
    assert 'gps' in names(report, 'fail')


def test_unclean_end_warns(tmp_path: Path) -> None:
    path = mission(tmp_path)
    manifest = yaml.safe_load((path / 'manifest.yaml').read_text())
    del manifest['ended_at']
    (path / 'manifest.yaml').write_text(yaml.safe_dump(manifest))
    assert 'end' in names(verify.verify(path), 'warn')


def test_unclosed_bag_warns(tmp_path: Path) -> None:
    path = mission(tmp_path)
    (path / 'rosbags' / 'rosbag_2').mkdir()
    assert 'recording' in names(verify.verify(path), 'warn')


def test_large_clock_offset_warns(tmp_path: Path) -> None:
    path = mission(tmp_path)
    event = {'kind': 'clock', 'level': 'ok', 'fields': {'offset_s': 3600.0}}
    (path / 'experiment_logs' / 'events.jsonl').write_text(json.dumps(event) + '\n')
    assert 'clock' in names(verify.verify(path), 'warn')


def test_collision_override_is_reported(tmp_path: Path) -> None:
    path = mission(tmp_path)
    with open(path / 'experiment_logs' / 'events.jsonl', 'a') as stream:
        stream.write(json.dumps({'source': 'helm', 'fields': {'override': 'collision'}}) + '\n')
    assert 'collision' in names(verify.verify(path), 'warn')
