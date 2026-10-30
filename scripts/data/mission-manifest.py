#!/usr/bin/env python3
"""Write what a mission ran: code, images, configuration and clock state.

start     create manifest.yaml, save the config and a diff of any uncommitted code
images    add the image id of every running container
end       stamp the end of the mission
"""

import argparse
import datetime
import json
import shutil
import subprocess
import uuid
from pathlib import Path

import yaml

# Never copied into a mission folder: vehicle.local.env holds credentials.
CONFIG_FILES = ('mission-config.yaml', 'module-registry.yaml', 'vehicle.env', 'helm.yaml')


def run(*cmd: str) -> str:
    """Return a command's output, or '' if it fails."""
    try:
        return subprocess.run(
            cmd, capture_output=True, text=True, timeout=20, check=True
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def repo_state(path: Path, diff_file: Path) -> dict[str, object]:
    """Return a repository's commit and branch; save its uncommitted diff, if any."""
    root = str(path)
    if not run('git', '-C', root, 'rev-parse', '--git-dir'):
        return {'commit': None}
    diff = run('git', '-C', root, 'diff', 'HEAD')
    if diff:
        diff_file.write_text(diff + '\n', encoding='utf-8')
    return {
        'commit': run('git', '-C', root, 'rev-parse', '--short=12', 'HEAD'),
        'branch': run('git', '-C', root, 'branch', '--show-current'),
        'dirty': bool(diff),
    }


def read(path: Path) -> dict:
    """Return the manifest."""
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def write(path: Path, manifest: dict) -> None:
    """Replace the manifest in one step."""
    tmp = path.with_suffix('.tmp')
    with open(tmp, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False)
    tmp.replace(path)


def start(args: argparse.Namespace) -> None:
    """Create the manifest."""
    mission = Path(args.mission_dir)
    core = Path(args.core_root)
    config_dir = mission / 'config'
    config_dir.mkdir(parents=True, exist_ok=True)
    for name in CONFIG_FILES:
        source = core / 'config' / name
        if source.is_file():
            shutil.copy2(source, config_dir / name)
    modules = {}
    for name in args.modules.split():
        modules[name] = repo_state(core / 'src' / 'modules' / name, config_dir / f'{name}.diff')
    manifest = {
        'schema': 2,
        'mission_id': args.mission_id,
        'mission_name': args.name,
        'started_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'platform': args.platform,
        'vehicle': args.vehicle,
        'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip(),
        'clock_synchronized': run('timedatectl', 'show', '-p', 'NTPSynchronized', '--value'),
        'core': repo_state(core, config_dir / 'blueboat-core.diff'),
        'application': args.application,
        'modules': modules,
        'recording_enabled': args.recording == 'TRUE',
        'bag_format': args.bag_format,
    }
    if args.sim:
        manifest['venue'] = 'sim'
    write(mission / 'manifest.yaml', manifest)


def images(args: argparse.Namespace) -> None:
    """Add the image id of every running container."""
    path = Path(args.mission_dir) / 'manifest.yaml'
    manifest = read(path)
    found = {}
    for line in run('docker', 'ps', '--format', '{{json .}}').splitlines():
        info = json.loads(line)
        name = info.get('Names', '')
        found[name] = {
            'image': info.get('Image', ''),
            'id': run('docker', 'inspect', '--format', '{{.Image}}', name)[:19],
        }
    manifest['containers'] = found
    write(path, manifest)


def end(args: argparse.Namespace) -> None:
    """Stamp the end of the mission."""
    path = Path(args.mission_dir) / 'manifest.yaml'
    manifest = read(path)
    now = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    manifest['ended_at'] = now
    if args.recovered:
        manifest['end'] = 'unclean, recovered at next start'
    write(path, manifest)


def main() -> None:
    """Parse the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('start')
    p.add_argument('mission_dir')
    p.add_argument('--core-root', required=True)
    p.add_argument('--mission-id', default=uuid.uuid4().hex[:12])
    p.add_argument('--name', default='mission')
    p.add_argument('--platform', default='')
    p.add_argument('--vehicle', default='')
    p.add_argument('--application', default='')
    p.add_argument('--modules', default='')
    p.add_argument('--recording', default='FALSE')
    p.add_argument('--bag-format', default='mcap')
    p.add_argument('--sim', action='store_true')
    p.set_defaults(func=start)
    p = commands.add_parser('images')
    p.add_argument('mission_dir')
    p.set_defaults(func=images)
    p = commands.add_parser('end')
    p.add_argument('mission_dir')
    p.add_argument('--recovered', action='store_true')
    p.set_defaults(func=end)
    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
