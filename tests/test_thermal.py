"""Tests for the sysfs thermal-zone reader behind /backseat/temperature.

`core_ops.thermal` has no ROS imports, so these run on a plain laptop. The
zones are fake directories under tmp_path laid out like /sys/class/thermal.
"""

import errno
import importlib.util
import os
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
THERMAL = REPO_ROOT / 'src/core/core_ops/core_ops/thermal.py'


def _load_thermal() -> ModuleType:
    spec = importlib.util.spec_from_file_location('thermal', THERMAL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


thermal = _load_thermal()
Zone = thermal.ZoneReading


def _zone(root: Path, index: int, zone_type: str | None, temp: str | None) -> Path:
    """Create root/thermal_zone<index>; a None field leaves that file out."""
    zone_dir = root / f'thermal_zone{index}'
    zone_dir.mkdir()
    if zone_type is not None:
        (zone_dir / 'type').write_text(f'{zone_type}\n')
    if temp is not None:
        (zone_dir / 'temp').write_text(temp)
    return zone_dir


# ---------------------------------------------------------------------------
# parse_millidegrees
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ('raw', 'expected'),
    [
        ('46656\n', 46.656),
        ('0', 0.0),
        ('-5000\n', -5.0),
        ('  45500  \n', 45.5),
    ],
)
def test_parse_millidegrees_converts_to_celsius(raw: str, expected: float) -> None:
    assert thermal.parse_millidegrees(raw) == pytest.approx(expected)


@pytest.mark.parametrize('raw', ['', '\n', '   ', 'N/A\n', '46.5', '��'])
def test_parse_millidegrees_rejects_empty_and_garbage(raw: str) -> None:
    assert thermal.parse_millidegrees(raw) is None


# ---------------------------------------------------------------------------
# read_thermal_zones
# ---------------------------------------------------------------------------


def test_reads_every_zone_in_number_order(tmp_path: Path) -> None:
    # thermal_zone10 sorts before thermal_zone2 as text.
    _zone(tmp_path, 10, 'tj-thermal', '46750\n')
    _zone(tmp_path, 2, 'gpu-thermal', '46000\n')
    _zone(tmp_path, 0, 'cpu-thermal', '46656\n')

    assert thermal.read_thermal_zones(tmp_path) == [
        Zone('cpu-thermal', 46.656),
        Zone('gpu-thermal', 46.0),
        Zone('tj-thermal', 46.75),
    ]


def test_zone_answering_eagain_is_skipped(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # On a Jetson the Orin Nano cv* zones fail every read with EAGAIN,
    # which open().read() reports as None rather than raising.
    _zone(tmp_path, 0, 'cpu-thermal', '46656\n')
    cv0 = _zone(tmp_path, 2, 'cv0-thermal', '0\n')
    cv0_inode = (cv0 / 'temp').stat().st_ino
    real_read = os.read

    def read(fd: int, n: int) -> bytes:
        if os.fstat(fd).st_ino == cv0_inode:
            raise BlockingIOError(errno.EAGAIN, os.strerror(errno.EAGAIN))
        return real_read(fd, n)

    monkeypatch.setattr(os, 'read', read)

    assert thermal.read_thermal_zones(tmp_path) == [Zone('cpu-thermal', 46.656)]


def test_unreadable_empty_and_garbage_zones_are_skipped(tmp_path: Path) -> None:
    _zone(tmp_path, 0, 'cpu-thermal', '46656\n')
    _zone(tmp_path, 1, 'empty-thermal', '')
    _zone(tmp_path, 2, 'garbage-thermal', 'N/A\n')
    _zone(tmp_path, 3, 'missing-thermal', None)
    # Opening a directory works; reading it raises IsADirectoryError.
    (_zone(tmp_path, 4, 'dir-thermal', None) / 'temp').mkdir()
    _zone(tmp_path, 5, 'soc0-thermal', '46375\n')

    assert thermal.read_thermal_zones(tmp_path) == [
        Zone('cpu-thermal', 46.656),
        Zone('soc0-thermal', 46.375),
    ]


def test_zone_without_type_is_named_after_its_directory(tmp_path: Path) -> None:
    _zone(tmp_path, 3, None, '41000\n')

    assert thermal.read_thermal_zones(tmp_path) == [Zone('thermal_zone3', 41.0)]


def test_unrelated_entries_are_ignored(tmp_path: Path) -> None:
    _zone(tmp_path, 0, 'cpu-thermal', '46656\n')
    (tmp_path / 'cooling_device0').mkdir()
    (tmp_path / 'thermal_zone_extra').mkdir()

    assert thermal.read_thermal_zones(tmp_path) == [Zone('cpu-thermal', 46.656)]


def test_missing_root_gives_no_readings(tmp_path: Path) -> None:
    assert thermal.read_thermal_zones(tmp_path / 'absent') == []


# ---------------------------------------------------------------------------
# pick_cpu_zone
# ---------------------------------------------------------------------------


def test_pick_prefers_cpu_zone() -> None:
    readings = [Zone('gpu-thermal', 50.0), Zone('cpu_thermal', 45.0)]

    assert thermal.pick_cpu_zone(readings) == Zone('cpu_thermal', 45.0)


def test_pick_falls_back_to_first_zone() -> None:
    readings = [Zone('gpu-thermal', 50.0), Zone('soc0-thermal', 45.0)]

    assert thermal.pick_cpu_zone(readings) == Zone('gpu-thermal', 50.0)


def test_pick_with_no_readings_is_none() -> None:
    assert thermal.pick_cpu_zone([]) is None
