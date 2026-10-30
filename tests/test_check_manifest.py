"""Tests of the manifest check."""

import importlib.util
from pathlib import Path
from types import ModuleType

import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/dev/check-manifest.py'


def _load() -> ModuleType:
    spec = importlib.util.spec_from_file_location('check_manifest', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


check = _load().check
CODE = """
TOPIC = '/sensors/x/data'
class N:
    def __init__(self):
        self.create_subscription(Int, '/frontseat/gps', self.cb, 10)
        self.create_publisher(Int, TOPIC, 10)
"""


def module(tmp_path: Path, code: str = CODE, **manifest: object) -> Path:
    base = {
        'kind': 'driver',
        'read_topics': ['/frontseat/gps'],
        'write_topics': ['/sensors/x/data'],
    }
    (tmp_path / 'manifest.yaml').write_text(yaml.safe_dump({**base, **manifest}))
    (tmp_path / 'node.py').write_text(code)
    return tmp_path


def test_matching_manifest_passes(tmp_path: Path) -> None:
    assert check(module(tmp_path)) == []


def test_undeclared_topic_is_found(tmp_path: Path) -> None:
    errors = check(module(tmp_path, write_topics=[]))
    assert any('code writes /sensors/x/data' in e for e in errors)


def test_stale_manifest_entry_is_found(tmp_path: Path) -> None:
    errors = check(module(tmp_path, read_topics=['/frontseat/gps', '/gone']))
    assert any('/gone' in e for e in errors)


def test_core_topics_are_refused(tmp_path: Path) -> None:
    code = "class N:\n    def f(self):\n        self.create_publisher(Int, '/helm/output', 10)\n"
    errors = check(module(tmp_path, code, read_topics=[], write_topics=['/helm/output']))
    assert any('reserved' in e for e in errors)


def test_only_a_skill_may_request_motion(tmp_path: Path) -> None:
    code = "class N:\n    def f(self):\n        self.create_publisher(Int, '/helm/setpoint', 10)\n"
    errors = check(module(tmp_path, code, read_topics=[], write_topics=['/helm/setpoint']))
    assert any('only a skill' in e for e in errors)
    assert (
        check(module(tmp_path, code, kind='skill', read_topics=[], write_topics=['/helm/setpoint']))
        == []
    )


def test_missing_kind_is_refused(tmp_path: Path) -> None:
    assert any('kind' in e for e in check(module(tmp_path, kind='payload')))
