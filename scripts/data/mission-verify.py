#!/usr/bin/env python3
"""Check a mission's records, so a problem shows up on the quay and not in the lab.

Writes <mission>/verify.json and prints one line per check. Standard library plus PyYAML, so it
runs on the vehicle's host. Exit code 1 when a check fails.
"""

import argparse
import json
import sys
from itertools import pairwise
from pathlib import Path

import yaml

REQUIRED_TOPICS = (
    '/frontseat/gps',
    '/frontseat/heading',
    '/frontseat/mode',
    '/helm/output',
    '/helm/status',
    '/events',
    '/diagnostics',
    '/rosout',
)
# Topics each module must have produced, by its name in mission-config.yaml.
MODULE_TOPICS = {
    'camera-payload': '/sensors/camera/image',
    'computer-vision-perception': '/perception/detections',
    'rl-policy-payload': '/skills/rl_policy/state',
    'argos-payload': '/sensors/argos/fix',
}
# Allowed silence in telemetry, seconds.
MAX_GAP_S = 5.0
# Clock offsets above this mean the logged times need correcting.
CLOCK_WARN_S = 5.0
PASS, WARN, FAIL = 'pass', 'warn', 'fail'


def _lines(path: Path) -> list[dict]:
    records = []
    with open(path, encoding='utf-8', errors='replace') as stream:
        for line in stream:
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                continue  # a power cut can leave one partial last line
    return records


def check_manifest(mission: Path) -> tuple[list[tuple[str, str, str]], dict]:
    """Check the mission's identity and how it ended."""
    path = mission / 'manifest.yaml'
    if not path.is_file():
        return [(FAIL, 'manifest', 'manifest.yaml is missing')], {}
    with open(path, encoding='utf-8') as stream:
        manifest = yaml.safe_load(stream) or {}
    out = []
    if not manifest.get('ended_at'):
        out.append((WARN, 'end', 'no end time: the mission did not stop cleanly'))
    elif manifest.get('end'):
        out.append((WARN, 'end', str(manifest['end'])))
    else:
        out.append((PASS, 'end', f'ended {manifest["ended_at"]}'))
    dirty = [n for n, r in (manifest.get('modules') or {}).items() if r.get('dirty')]
    if (manifest.get('core') or {}).get('dirty'):
        dirty.append('blueboat-core')
    if dirty:
        out.append((WARN, 'code', f'uncommitted code in {", ".join(dirty)}: see config/*.diff'))
    return out, manifest


def check_bags(mission: Path, manifest: dict) -> list[tuple[str, str, str]]:
    """Check that the recording exists and holds the topics it must."""
    if not manifest.get('recording_enabled'):
        return [(WARN, 'recording', 'recording was off')]
    bags = sorted((mission / 'rosbags').glob('*'))
    bags = [b for b in bags if b.is_dir()]
    if not bags:
        return [(FAIL, 'recording', 'no bag was recorded')]
    counts: dict[str, int] = {}
    unclosed = []
    for bag in bags:
        meta = bag / 'metadata.yaml'
        if not meta.is_file():
            unclosed.append(bag.name)
            continue
        with open(meta, encoding='utf-8') as stream:
            info = yaml.safe_load(stream)['rosbag2_bagfile_information']
        for item in info['topics_with_message_count']:
            name = item['topic_metadata']['name']
            counts[name] = counts.get(name, 0) + item['message_count']
    out = []
    if unclosed:
        out.append((WARN, 'recording', f'not closed, run ros2 bag reindex: {", ".join(unclosed)}'))
    if not counts:
        return out or [(FAIL, 'recording', 'bags hold no metadata')]
    required = list(REQUIRED_TOPICS)
    required += [MODULE_TOPICS[m] for m in manifest.get('modules') or {} if m in MODULE_TOPICS]
    missing = [t for t in required if counts.get(t, 0) == 0]
    if missing:
        out.append((FAIL, 'topics', f'no messages on {", ".join(missing)}'))
    else:
        out.append((PASS, 'topics', f'all {len(required)} required topics present'))
    return out


def check_telemetry(mission: Path) -> list[tuple[str, str, str]]:
    """Check the slim telemetry for bad GPS and gaps."""
    path = mission / 'experiment_logs' / 'telemetry.jsonl'
    if not path.is_file():
        return [(FAIL, 'telemetry', 'telemetry.jsonl is missing')]
    rows = _lines(path)
    if not rows:
        return [(FAIL, 'telemetry', 'telemetry.jsonl is empty')]
    out = []
    valid = sum(1 for r in rows if r.get('gps_valid'))
    share = valid / len(rows)
    if valid == 0:
        out.append((FAIL, 'gps', f'no valid GPS in {len(rows)} rows'))
    elif share < 0.95:
        out.append((WARN, 'gps', f'valid GPS in {share:.0%} of {len(rows)} rows'))
    else:
        out.append((PASS, 'gps', f'valid GPS in {share:.0%} of {len(rows)} rows'))
    times = [r['t_mono'] for r in rows if isinstance(r.get('t_mono'), int | float)]
    gaps = [b - a for a, b in pairwise(times) if b - a > MAX_GAP_S]
    if gaps:
        out.append((WARN, 'gaps', f'{len(gaps)} telemetry gaps, longest {max(gaps):.0f} s'))
    return out


def check_events(mission: Path) -> list[tuple[str, str, str]]:
    """Check the event log: clock state and errors."""
    path = mission / 'experiment_logs' / 'events.jsonl'
    if not path.is_file():
        return [(FAIL, 'events', 'events.jsonl is missing')]
    events = _lines(path)
    out = []
    clocks = [e for e in events if e.get('kind') == 'clock']
    if not clocks:
        out.append((WARN, 'clock', 'no GPS time heard: logged times cannot be corrected'))
    else:
        offset = clocks[-1].get('fields', {}).get('offset_s', 0.0)
        level = WARN if abs(offset) > CLOCK_WARN_S else PASS
        out.append((level, 'clock', f'backseat clock is {offset:+.1f} s from GPS'))
    errors = [e for e in events if e.get('level') == 'error']
    if errors:
        sources = sorted({str(e.get('source')) for e in errors})
        out.append((WARN, 'errors', f'{len(errors)} error events from {", ".join(sources)}'))
    overrides = [
        e
        for e in events
        if e.get('source') == 'helm' and e.get('fields', {}).get('override') == 'collision'
    ]
    if overrides:
        out.append((WARN, 'collision', f'the collision filter acted {len(overrides)} times'))
    return out


def verify(mission: Path) -> dict:
    """Run every check on one mission folder."""
    checks, manifest = check_manifest(mission)
    checks += check_bags(mission, manifest)
    checks += check_telemetry(mission)
    checks += check_events(mission)
    worst = FAIL if any(c[0] == FAIL for c in checks) else WARN
    if not any(c[0] != PASS for c in checks):
        worst = PASS
    return {
        'result': worst,
        'checks': [{'level': c[0], 'name': c[1], 'detail': c[2]} for c in checks],
    }


def main() -> int:
    """Check one mission folder."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission_dir')
    args = parser.parse_args()
    mission = Path(args.mission_dir)
    report = verify(mission)
    (mission / 'verify.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    for check in report['checks']:
        print(f'  [{check["level"]}] {check["name"]}: {check["detail"]}')
    print(f'Mission check: {report["result"]}')
    return 1 if report['result'] == FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
