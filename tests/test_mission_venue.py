"""Tests that mission-venue.py files declared simulations under sim/ without classifying them.

Needs PyYAML, so it is skipped where only pytest is installed.
"""

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest

yaml = pytest.importorskip('yaml')

VENUE = Path(__file__).resolve().parents[1] / 'scripts' / 'data' / 'mission-venue.py'


def _load_venue() -> ModuleType:
    spec = importlib.util.spec_from_file_location('mission_venue', VENUE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


venue = _load_venue()


def _mission(root: Path, manifest: dict) -> Path:
    mission = root / '20260925_120000_test'
    mission.mkdir(parents=True)
    (mission / 'manifest.yaml').write_text(yaml.safe_dump(manifest))
    return mission


def test_declared_sim_is_filed_under_sim(tmp_path: Path) -> None:
    mission = _mission(tmp_path, {'mission_name': 'test', 'venue': 'sim'})
    verdict, _ = venue.file_mission(str(mission), str(tmp_path), verdict=None)
    assert verdict.bucket == 'sim'
    filed = tmp_path / 'sim' / mission.name
    assert filed.is_dir()
    assert yaml.safe_load((filed / 'manifest.yaml').read_text())['venue'] == 'sim'


def test_stamped_lab_venue_is_not_treated_as_declared(tmp_path: Path) -> None:
    mission = _mission(tmp_path, {'mission_name': 'test', 'venue': 'lab'})
    assert venue.declared_venue(str(mission)) is None


def test_mission_without_evidence_is_unsorted(tmp_path: Path) -> None:
    mission = _mission(tmp_path, {'mission_name': 'test'})
    verdict = venue.classify_mission(str(mission), allow_docker=False)
    assert verdict.bucket == '_unsorted'
