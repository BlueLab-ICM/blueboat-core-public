#!/usr/bin/env python3
"""Read config/mission-config.yaml and each module's manifest for the mission scripts.

exports   print shell assignments for the mission
modules   print one line per module: name, mission data folder, shared data folder, persistent
"""

import os
import re
import shlex
import sys
from pathlib import Path

import yaml


def load(core_root: Path) -> dict:
    """Return the mission config: MISSION_CONFIG, or config/mission-config.yaml."""
    path = os.environ.get('MISSION_CONFIG') or core_root / 'config' / 'mission-config.yaml'
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def exports(core_root: Path) -> None:
    """Print the mission's shell variables."""
    data = load(core_root)
    rec = data.get('recording') or {}
    name = str((data.get('mission_metadata') or {}).get('name', 'mission'))
    name = re.sub(r'[^A-Za-z0-9_-]+', '_', name).strip('_') or 'mission'
    excludes = rec.get('exclude_topics') or []
    values = {
        'MISSION_NAME': name,
        'RECORDING_ENABLED': str(bool(rec.get('enabled', False))).upper(),
        'BAG_FORMAT': rec.get('format', 'mcap'),
        'BAG_SPLIT_S': str(rec.get('split_seconds', 300)),
        'BAG_EXCLUDE_REGEX': '|'.join(f'^{t}$' for t in excludes),
        'MODULES': ' '.join(data.get('modules') or []),
        'APPLICATION': str(data.get('application') or ''),
    }
    for key, value in values.items():
        print(f'{key}={shlex.quote(value)}')


def modules(core_root: Path) -> None:
    """Print ``name mission_dir shared_dir persistent`` per module; ``-`` means none."""
    for name in load(core_root).get('modules') or []:
        manifest = core_root / 'src' / 'modules' / name / 'manifest.yaml'
        data_dir = shared = '-'
        persistent = 'no'
        if manifest.is_file():
            with open(manifest, encoding='utf-8') as stream:
                info = yaml.safe_load(stream) or {}
            data = info.get('data') or {}
            data_dir = data.get('mission_dir') or '-'
            shared = data.get('shared_dir') or '-'
            persistent = 'yes' if info.get('persistent') else 'no'
        print(name, data_dir, shared, persistent)


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[2]
    {'exports': exports, 'modules': modules}[sys.argv[1]](root)
