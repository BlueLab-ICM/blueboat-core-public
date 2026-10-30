#!/usr/bin/env python3
"""Decide whether a mission ran on the water or dry in the lab, and file it.

log       sample motion for the life of a mission -> system/motion.csv
classify  print a verdict and the evidence behind it
file      classify, stamp manifest.yaml, move into lab/ water/ sim/ _unsorted/
backfill  one-off: sort missions still sitting at the root

"""

import argparse
import csv
import glob
import io
import json
import math
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
import urllib.request

# Thresholds sit at the geometric mean of the measured lab and water values, so
# each is ~25x clear of the lab floor and ~25x clear of the water level.
YAW_RATE_SD_THRESH = 0.005  # rad/s   lab 0.0002 | water 0.1425
HEADING_CIRC_SD_THRESH = 0.1  # rad     lab ~0.005 | water ~1.2
CURRENT_MEAN_THRESH = 2.0  # A       lab 0.39   | water 7.80

MIN_SAMPLES = 60  # ~1 min at 1 Hz; below this the sd is meaningless
MIN_SIGNALS = 2  # never decide on a single sensor
FSYNC_EVERY = 10  # rows between forced flushes to disk

MAV_MESSAGES = 'mavlink/vehicles/1/components/1/messages'

ROS_IMAGE = os.environ.get('BRAIN_IMAGE', 'blueboat-brain:latest')
BUCKETS = ('lab', 'water', 'sim', '_unsorted')
# Kept at the data root across missions, never filed.
SHARED_DIRS = ('argos_history',)


class Verdict:
    """A venue decision plus the evidence it rests on, so it can be argued with later."""

    def __init__(
        self,
        venue: str,
        reason: str,
        detail: dict[str, float] | None = None,
        votes: dict[str, bool] | None = None,
        source: str = '?',
    ) -> None:
        self.venue = venue  # "lab" | "water" | "sim" | "UNSORTED"
        self.reason = reason
        self.detail = detail or {}
        self.votes = votes or {}
        self.source = source

    @property
    def bucket(self) -> str:
        """Directory this verdict files into; anything undecided goes to _unsorted."""
        return self.venue if self.venue in ('lab', 'water', 'sim') else '_unsorted'

    def line(self) -> str:
        """One-line human-readable form, evidence included."""
        bits = ' '.join(f'{k}={v:.4f}' for k, v in self.detail.items())
        vs = ' '.join(f'{k}={"water" if w else "lab"}' for k, w in self.votes.items())
        return (
            f'VERDICT={self.venue} SOURCE={self.source} REASON={self.reason!r} {bits} {vs}'
        ).strip()

    def to_json(self) -> str:
        """Serialise for crossing the container boundary in classify_via_docker."""
        return json.dumps(
            {
                'venue': self.venue,
                'reason': self.reason,
                'detail': self.detail,
                'votes': self.votes,
                'source': self.source,
            }
        )

    @staticmethod
    def from_json(s: str) -> 'Verdict':
        """Rebuild a verdict emitted by a subprocess."""
        d = json.loads(s)
        return Verdict(d['venue'], d['reason'], d['detail'], d['votes'], d['source'])


def vehicle_env(name: str) -> str:
    """Read one variable from vehicle.env, the single source of per-vehicle values.

    It is bash, so it has to be sourced rather than parsed -- duplicating its
    defaults here is exactly what its own comment warns against.
    """
    env = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..', 'config', 'vehicle.env'
    )
    out = subprocess.run(
        ['bash', '-c', f'source "{env}"; printf %s "${name}"'],
        capture_output=True,
        text=True,
    )
    value = out.stdout.strip()
    if not value:
        sys.exit(f'mission-venue: could not read {name} from {env}')
    return value


# --- evidence ---------------------------------------------------------------


def circular_sd(angles_rad: list[float]) -> float | None:
    """Spread of a heading series, wrap-safe. 0 = pinned, grows without bound."""
    if len(angles_rad) < 2:
        return None
    c = statistics.mean(math.cos(a) for a in angles_rad)
    s = statistics.mean(math.sin(a) for a in angles_rad)
    r = math.hypot(c, s)
    if r <= 1e-12:
        return float('inf')
    return math.sqrt(max(0.0, -2.0 * math.log(min(1.0, r))))


def decide(yaw_rates: list[float], headings_rad: list[float], currents: list[float]) -> Verdict:
    """Majority vote over whichever signals are present. Ties -> UNSORTED."""
    votes, detail = {}, {}

    if len(yaw_rates) >= MIN_SAMPLES:
        sd = statistics.stdev(yaw_rates)
        votes['yaw_rate'] = sd > YAW_RATE_SD_THRESH
        detail['yaw_rate_sd'] = sd

    if len(headings_rad) >= MIN_SAMPLES:
        sd = circular_sd(headings_rad)
        if sd is not None:
            votes['heading'] = sd > HEADING_CIRC_SD_THRESH
            detail['heading_circ_sd'] = sd

    if len(currents) >= MIN_SAMPLES:
        mean = statistics.mean(abs(c) for c in currents)
        votes['current'] = mean > CURRENT_MEAN_THRESH
        detail['current_mean'] = mean

    if len(votes) < MIN_SIGNALS:
        return Verdict('UNSORTED', f'only {len(votes)} usable signal(s)', detail, votes)

    water = sum(votes.values())
    if water * 2 > len(votes):
        return Verdict('water', f'{water}/{len(votes)} signals say water', detail, votes)
    if water * 2 < len(votes):
        return Verdict('lab', f'{len(votes) - water}/{len(votes)} signals say lab', detail, votes)
    return Verdict('UNSORTED', f'signals split {water}/{len(votes)}', detail, votes)


def from_motion_csv(path: str) -> tuple[list[float], list[float], list[float]]:
    """Read the sampler's log, tolerating a NUL-padded tail.

    A power cut leaves the last blocks unflushed: ext4 has the file size but
    not the data, so the file ends in zeros that csv refuses to parse at all.
    Everything before the first NUL did reach disk, so keep it and drop the
    rest -- a short log still classifies, a hard error does not.
    """
    with open(path, newline='', errors='replace') as f:
        text = f.read()
    nul = text.find('\0')
    if nul != -1:
        text = text[: text.rfind('\n', 0, nul) + 1]  # also drops the partial row

    def num(row: dict[str, str], k: str) -> float | None:
        try:
            v = float(row[k])
            return None if math.isnan(v) else v
        except (KeyError, ValueError, TypeError):
            return None

    yaw, head, cur = [], [], []
    for row in csv.DictReader(io.StringIO(text)):
        for key, sink in (('yawspeed', yaw), ('yaw', head), ('current_a', cur)):
            v = num(row, key)
            if v is not None:
                sink.append(v)
    return yaw, head, cur


def from_bag(path: str) -> tuple[list[float], list[float], list[float]]:
    """Read motion signals from a rosbag. Imports ROS lazily: it only exists in the container."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Float64

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')
    )
    yaw, head, cur = [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/frontseat/yaw_rate':
            v = deserialize_message(data, Float64).data
            if not math.isnan(v):
                yaw.append(v)
        elif topic == '/frontseat/heading':
            v = deserialize_message(data, Float64).data
            if not math.isnan(v):
                head.append(math.radians(v))  # bag logs degrees
        elif topic == '/frontseat/battery':
            v = deserialize_message(data, BatteryState).current
            if not math.isnan(v):
                cur.append(v)
    return yaw, head, cur


def classify_in_process(mission_dir: str) -> Verdict:
    """Prefer the live sampler; fall back to the largest bag."""
    motion = os.path.join(mission_dir, 'system', 'motion.csv')
    if os.path.exists(motion):
        y, h, c = from_motion_csv(motion)
        if len(y) >= MIN_SAMPLES or len(h) >= MIN_SAMPLES:
            v = decide(y, h, c)
            v.source = 'motion.csv'
            return v

    bags = glob.glob(os.path.join(mission_dir, 'rosbags', '**', '*.mcap'), recursive=True)
    if bags:
        biggest = max(bags, key=os.path.getsize)
        try:
            y, h, c = from_bag(biggest)
        except ImportError:
            return Verdict('UNSORTED', 'bag needs ROS', source='bag-needs-ros')
        v = decide(y, h, c)
        v.source = os.path.basename(biggest)
        return v

    return Verdict('UNSORTED', 'no bag, no motion.csv', source='no-evidence')


def classify_via_docker(mission_dir: str) -> Verdict:
    """Replay a bag inside the brain image, which is the only place ROS exists.

    A real process boundary, so the verdict comes back as JSON rather than
    something a regex has to pick apart.
    """
    me = os.path.abspath(__file__)
    proc = subprocess.run(
        [
            'docker',
            'run',
            '--rm',
            '-v',
            f'{mission_dir}:/m:ro',  # read-only: never mutate the archive
            '-v',
            f'{me}:/mv.py:ro',
            ROS_IMAGE,
            'bash',
            '-lc',
            'source /opt/ros/*/setup.bash; python3 /mv.py classify /m --json',
        ],
        capture_output=True,
        text=True,
    )
    for line in reversed(proc.stdout.strip().splitlines()):
        if line.startswith('{'):
            return Verdict.from_json(line)
    err = (proc.stderr or '').strip().splitlines()
    tail = err[-1] if err else f'exit {proc.returncode}, no output'
    return Verdict(
        'UNSORTED', f'classification in container failed: {tail}', source='docker-failed'
    )


def declared_venue(mission_dir: str) -> Verdict | None:
    """Return sim if start-mission.sh --sim declared it; a simulated boat moves like a real one."""
    import yaml

    try:
        with open(os.path.join(mission_dir, 'manifest.yaml')) as f:
            manifest = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return None
    if manifest.get('venue') == 'sim':
        return Verdict('sim', 'declared by start-mission.sh --sim', source='manifest')
    return None


def classify_mission(mission_dir: str, allow_docker: bool = True) -> Verdict:
    """Classify a mission, always returning a Verdict.

    Unreadable evidence is UNSORTED, not a crash: a mission that cannot be
    classified must still get filed somewhere.
    """
    declared = declared_venue(mission_dir)
    if declared:
        return declared
    try:
        v = classify_in_process(mission_dir)
    except Exception as e:
        return Verdict(
            'UNSORTED', f'classification failed: {type(e).__name__}: {e}', source='error'
        )
    if v.source == 'bag-needs-ros' and allow_docker:
        try:
            return classify_via_docker(mission_dir)
        except Exception as e:
            return Verdict(
                'UNSORTED',
                f'container classification failed: {type(e).__name__}: {e}',
                source='error',
            )
    return v


def stamp_manifest(mission_dir: str, verdict: Verdict) -> None:
    """Write the verdict next to the data, so a wrong call can be audited."""
    import yaml

    path = os.path.join(mission_dir, 'manifest.yaml')
    try:
        with open(path) as f:
            manifest = yaml.safe_load(f) or {}
    except FileNotFoundError:
        manifest = {}
    manifest['venue'] = verdict.bucket
    manifest['venue_evidence'] = verdict.line()
    with open(path, 'w') as f:
        yaml.safe_dump(manifest, f, sort_keys=False)


def repoint_current_mission(root: str, old: str, new: str) -> None:
    """.current_mission still names the mission we just moved."""
    cur = os.path.join(root, '.current_mission')
    try:
        with open(cur) as f:
            if f.read().strip() == old:
                with open(cur, 'w') as w:
                    w.write(new + '\n')
    except FileNotFoundError:
        pass


def file_mission(
    mission_dir: str, root: str, dry_run: bool = False, verdict: Verdict | None = None
) -> tuple[Verdict | None, str]:
    """Move a finished mission into its bucket. Safe to re-run."""
    mission_dir = os.path.abspath(mission_dir)
    if os.path.basename(os.path.dirname(mission_dir)) in BUCKETS:
        return None, 'already filed'

    v = verdict or classify_mission(mission_dir)
    dest = os.path.join(root, v.bucket, os.path.basename(mission_dir))
    if dry_run:
        return v, f'[dry-run] -> {v.bucket}/'
    if os.path.exists(dest):
        return v, f'!! {dest} exists — left in place'

    os.makedirs(os.path.join(root, v.bucket), exist_ok=True)
    try:
        stamp_manifest(mission_dir, v)
    except Exception as e:
        print(f'  (manifest not stamped: {e})', file=sys.stderr)
    shutil.move(mission_dir, dest)
    repoint_current_mission(root, mission_dir, dest)
    return v, f'-> {v.bucket}/'


# --- subcommands ------------------------------------------------------------


def cmd_log(args: argparse.Namespace) -> int:
    """Poll the frontseat, not ROS, so this works with recording disabled."""
    out_dir = os.path.join(args.mission_dir, 'system')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'motion.csv')
    period = 1.0 / args.hz
    mav_url = vehicle_env('MAVLINK2REST_URL')
    attitude_url = f'{mav_url}/{MAV_MESSAGES}/ATTITUDE'
    sys_status_url = f'{mav_url}/{MAV_MESSAGES}/SYS_STATUS'

    running = [True]
    signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))

    def fetch(url: str) -> dict | None:
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                return json.load(r)['message']
        except Exception:
            return None

    fields = ['time', 'roll', 'pitch', 'yaw', 'rollspeed', 'pitchspeed', 'yawspeed', 'current_a']
    with open(path, 'w', buffering=1) as f:  # line-buffered: survives a kill
        f.write(','.join(fields) + '\n')
        last_boot_ms = None
        rows = 0
        while running[0]:
            att = fetch(attitude_url)
            if att is not None and att.get('time_boot_ms') != last_boot_ms:
                last_boot_ms = att.get('time_boot_ms')
                sysst = fetch(sys_status_url)
                cur = ''
                if sysst is not None:
                    raw = sysst.get('current_battery', -1)
                    if raw is not None and raw != -1:
                        cur = f'{abs(raw) / 100.0:.3f}'
                nan = float('nan')
                f.write(
                    ','.join(
                        [f'{time.time():.3f}']
                        + [f'{att.get(k, nan):.6f}' for k in fields[1:-1]]
                        + [cur]
                    )
                    + '\n'
                )
                # Line buffering survives a kill but not a power cut -- the
                # unflushed tail comes back as NULs. Force it to disk.
                rows += 1
                if rows % FSYNC_EVERY == 0:
                    os.fsync(f.fileno())
            time.sleep(period)
        os.fsync(f.fileno())
    return 0


def cmd_classify(args: argparse.Namespace) -> int:
    """Print a verdict without moving anything."""
    v = classify_mission(args.mission_dir, allow_docker=not args.no_docker)
    print(v.to_json() if args.json else v.line())
    return 0


def cmd_file(args: argparse.Namespace) -> int:
    """Classify one mission and move it into its bucket."""
    root = vehicle_env('DEPLOY_DATA_ROOT')
    v, msg = file_mission(args.mission_dir, root, dry_run=args.dry_run)
    if v is None:
        print(f'mission-venue: {msg}')
        return 0
    print(f'mission-venue: {v.line()}')
    print(f'mission-venue: {os.path.basename(args.mission_dir)} {msg}')
    return 0


def cmd_backfill(args: argparse.Namespace) -> int:
    """Sort every mission still sitting at the root of the data directory."""
    root = vehicle_env('DEPLOY_DATA_ROOT')
    print(f'{"MISSION":<54} {"VERDICT":<9} EVIDENCE')
    print('-' * 120)
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isdir(d) or name in BUCKETS or name in SHARED_DIRS:
            continue
        try:
            v = classify_mission(d)
            print(f'{name:<54} {v.bucket:<9} {v.line().split(" ", 1)[1]}')
            if not args.dry_run:
                file_mission(d, root, verdict=v)
        except Exception as e:
            print(f'{name:<54} {"SKIPPED":<9} {type(e).__name__}: {e}')
    return 0


def main() -> int:
    """Dispatch to the requested subcommand."""
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('log', help='sample motion into system/motion.csv')
    s.add_argument('mission_dir', type=os.path.abspath)
    s.add_argument('--hz', type=float, default=1.0)
    s.set_defaults(func=cmd_log)

    s = sub.add_parser('classify', help='print verdict and evidence')
    s.add_argument('mission_dir', type=os.path.abspath)
    s.add_argument('--json', action='store_true')
    s.add_argument(
        '--no-docker', action='store_true', help='do not shell out to the ROS image for bag replay'
    )
    s.set_defaults(func=cmd_classify)

    s = sub.add_parser('file', help='classify and move into lab/ water/ sim/ _unsorted/')
    s.add_argument('mission_dir', type=os.path.abspath)
    s.add_argument('--dry-run', action='store_true')
    s.set_defaults(func=cmd_file)

    s = sub.add_parser('backfill', help='sort missions still at the root')
    s.add_argument('--dry-run', action='store_true')
    s.set_defaults(func=cmd_backfill)

    args = p.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
