"""ROS node that runs uploaded plans as behaviour trees.

Requests on /executive/request, status on /executive/status (latched). The node owns the run
lifecycle; the application supplies the plan schema and the trees. Motion goes to the helm:
navigation as setpoints from source ``executive``, skills by granting them the helm.
"""

from __future__ import annotations

import collections
import hashlib
import json
import math
import os
import shutil
import time
from pathlib import Path
from typing import Any

import py_trees
import rclpy
from core_common.health import OK, ModuleHealth
from core_interfaces.msg import HelmGrant, HelmStatus, Setpoint
from core_interfaces.srv import Engage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from py_trees.common import Status
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import Float64, String

from core_executive.application import Application, PlanError, load_application
from core_executive.behaviours import MissionPlan, RunContext, ll
from core_executive.geometry import LatLon
from core_executive.run_recorder import RunRecorder, mounted_log_root
from core_executive.skills import SkillClient

LOG_ROOT = Path('/app/backseat_ws/experiment_logs')
SOURCE = 'executive'
TICK_S = 0.1
STATUS_PERIOD_S = 0.5
OUTSIDE_AREA_GRACE_S = 2.0
HELM_ENGAGE_GRACE_S = 5.0
GPS_STALE_S = 2.0
# Beyond the fence radius before the helm stops the vehicle.
FENCE_ABORT_MARGIN_M = 5.0
# The autopilot's home must match the plan's within this, so a lost link returns there.
HOME_MATCH_M = 3.0
# Free space needed on the log disk: frames at 5 Hz are about 2.7 GB an hour.
MIN_FREE_BYTES = 512 * 1024**2
ACK_HISTORY = 64
# ArduRover FS_ACTION values that drive home: 1 RTL, 3 SmartRTL or RTL.
FS_ACTIONS_HOME = (1, 3)

IDLE, READY, STARTING, RUNNING, PAUSED, RETURNING, COMPLETED, STOPPED, CANCELLED = (
    'idle',
    'ready',
    'starting',
    'running',
    'paused',
    'returning',
    'completed',
    'stopped',
    'cancelled',
)
FINISHED = (READY, COMPLETED, STOPPED, CANCELLED)
ACTIVE_STATES = (STARTING, RUNNING, PAUSED, RETURNING)

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class RosWorld:
    """The World the trees see on the vessel. Applications extend it."""

    def __init__(self, node: ExecutiveNode) -> None:
        self.node = node
        self.directive: dict[str, Any] | None = None
        self.skills: dict[str, SkillClient] = {}

    def now(self) -> float:
        """Return monotonic time."""
        return time.monotonic()

    def position(self) -> LatLon | None:
        """Return the latest valid, fresh GPS position."""
        return self.node.fresh_position()

    def heading_rad(self) -> float | None:
        """Return the heading in radians, NED."""
        h = self.node.heading_deg
        return None if h is None else math.radians(h)

    def command(self, directive: dict[str, Any]) -> None:
        """Set this tick's directive."""
        self.directive = directive

    def event(self, kind: str, **fields: object) -> None:
        """Log an event to the run directory and to /events."""
        self.node.event(kind, **fields)

    def skill(self, name: str) -> SkillClient:
        """Return the client of skill ``name``, creating it on first use."""
        if name not in self.skills:
            self.skills[name] = SkillClient(self.node, name)
        return self.skills[name]


class ExecutiveNode(Node):
    """Run uploaded plans as behaviour trees."""

    def __init__(self) -> None:
        super().__init__('executive_node')
        self.declare_parameter('log_root', str(LOG_ROOT))
        self.declare_parameter('require_log_mount', True)
        # The autopilot must drive home if the base-station link drops
        # (FS_GCS_ENABLE on, FS_ACTION RTL), to the home the plan set. A simulated frontseat has
        # no ground station whose heartbeat would keep that failsafe from firing.
        self.declare_parameter('require_gcs_failsafe', os.environ.get('SIM_MODE') != 'true')
        # On the vehicle every module the plan needs must report healthy. A SITL laptop runs
        # stand-ins that do not report.
        self.declare_parameter(
            'require_module_health', os.environ.get('BACKSEAT_PLATFORM', '') == 'jetson'
        )

        self.state = IDLE
        self.message = 'no plan loaded'
        self.plan: MissionPlan | None = None
        self.plan_hash = ''
        self.ctx: RunContext | None = None
        self.tree: py_trees.behaviour.Behaviour | None = None
        self.recorder: RunRecorder | None = None
        # The last run's recorder, kept so answers about a run can still be written after it.
        self.last_recorder: RunRecorder | None = None
        self.next_trial = 1
        self.last_ack: dict[str, Any] = {}
        self.acks: collections.OrderedDict[str, dict[str, Any]] = collections.OrderedDict()
        self.cancel_reason = ''
        self.autopilot_home: LatLon | None = None
        self.autopilot_home_time = 0.0
        self.failsafe: dict[str, Any] = {}
        self.clock_offset_s: float | None = None
        self.run_started = 0.0
        self.engaged_at: float | None = None
        self.hold_point: LatLon | None = None
        self.outside_since: float | None = None
        self._status_dirty = True
        self._last_status = 0.0

        self.gps: NavSatFix | None = None
        self.gps_time = 0.0
        self.heading_deg: float | None = None
        self.frontseat_mode = ''
        self.helm = HelmStatus()
        self.module_health: dict[str, DiagnosticStatus] = {}
        self.gps_utc_offset: float | None = None

        self.health = ModuleHealth(self, 'executive')
        self.health.ok('no plan loaded')
        self.setpoint_pub = self.create_publisher(Setpoint, '/helm/setpoint', 10)
        self.grant_pub = self.create_publisher(HelmGrant, '/helm/grant', 10)
        self.set_home_pub = self.create_publisher(NavSatFix, '/frontseat/set_home', 10)
        self.status_pub = self.create_publisher(String, '/executive/status', LATCHED)
        self.create_subscription(String, '/executive/request', self._on_request, 10)
        self.create_subscription(NavSatFix, '/frontseat/gps', self._on_gps, 10)
        self.create_subscription(Float64, '/frontseat/heading', self._on_heading, 10)
        self.create_subscription(String, '/frontseat/mode', self._on_mode, 10)
        self.create_subscription(HelmStatus, '/helm/status', self._on_helm, LATCHED)
        self.create_subscription(NavSatFix, '/frontseat/home', self._on_home, LATCHED)
        self.create_subscription(String, '/frontseat/failsafe', self._on_failsafe, LATCHED)
        self.create_subscription(DiagnosticArray, '/system/health', self._on_health, LATCHED)
        self.create_subscription(TimeReference, '/frontseat/time', self._on_time, 10)
        self.engage_client = self.create_client(Engage, '/helm/engage')

        self.app: Application = load_application(os.environ.get('BLUEBOAT_APPLICATION', ''), self)
        self.world = self.app.make_world(RosWorld)

        self.create_timer(TICK_S, self._tick)
        self.get_logger().info(f'Executive online, application {self.app.name}.')

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self.gps = msg
        self.gps_time = time.monotonic()

    def fresh_position(self) -> LatLon | None:
        """Return the GPS position if it is recent, else None."""
        if self.gps is None or time.monotonic() - self.gps_time > GPS_STALE_S:
            return None
        return LatLon(self.gps.latitude, self.gps.longitude)

    def _on_heading(self, msg: Float64) -> None:
        self.heading_deg = float(msg.data)

    def _on_mode(self, msg: String) -> None:
        self.frontseat_mode = msg.data

    def _on_helm(self, msg: HelmStatus) -> None:
        self.helm = msg

    def _on_health(self, msg: DiagnosticArray) -> None:
        self.module_health = {s.name: s for s in msg.status}

    def _on_home(self, msg: NavSatFix) -> None:
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.autopilot_home = LatLon(msg.latitude, msg.longitude)
            self.autopilot_home_time = time.monotonic()

    def _on_failsafe(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            self.failsafe = data

    def _on_time(self, msg: TimeReference) -> None:
        gps = msg.time_ref.sec + msg.time_ref.nanosec * 1e-9
        self.gps_utc_offset = gps - time.time()

    def utc_now(self) -> float:
        """Return UTC seconds, from the autopilot's GPS time when it has been heard.

        The backseat has no battery-backed clock, so at sea its own time can be hours off.
        """
        return time.time() + (self.gps_utc_offset or 0.0)

    def mark_status_dirty(self) -> None:
        """Publish the status on the next tick."""
        self._status_dirty = True

    def send_set_home(self, home: LatLon) -> None:
        """Ask the gateway to set the autopilot's home (MAV_CMD_DO_SET_HOME)."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = home.lat
        msg.longitude = home.lon
        msg.altitude = math.nan  # the gateway uses the vehicle's current altitude
        self.set_home_pub.publish(msg)

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Ignoring a request that is not JSON.')
            return
        if not isinstance(req, dict):
            return
        req_id = str(req.get('id', ''))
        if req_id and req_id in self.acks:
            self._status_dirty = True  # a resend: repeat the answer, do nothing twice
            return
        cmd = str(req.get('cmd', ''))
        handlers = {
            'load': self._load,
            'start': self._start,
            'pause': self._pause,
            'resume': self._resume,
            'skip': self._skip,
            'stop': self._stop_request,
            'clear': self._clear,
            'set_home': self._set_home_request,
            **self.app.requests(),
        }
        handler = handlers.get(cmd)
        if handler is None:
            ok, text = False, f'unknown command {cmd!r}'
        else:
            try:
                ok, text = handler(req)
            except Exception as exc:  # noqa: BLE001 - a bad request must not kill the run
                self.get_logger().error(f'Request {cmd} failed: {exc!r}')
                ok, text = False, f'{cmd} failed on the vehicle: {exc}'
        self.last_ack = {'id': req_id, 'cmd': cmd, 'ok': ok, 'message': text}
        if req_id:
            self.acks[req_id] = self.last_ack
            while len(self.acks) > ACK_HISTORY:
                self.acks.popitem(last=False)
        self.get_logger().info(f'Request {cmd} ({req_id}): {"ok" if ok else "refused"}: {text}')
        self.event('request', cmd=cmd, id=req_id, ok=ok, message=text)
        self._status_dirty = True

    def _load(self, req: dict[str, Any]) -> tuple[bool, str]:
        if self.state in ACTIVE_STATES:
            return False, f'cannot load a plan while {self.state}'
        question = self.app.blocking_question()
        if question:
            return False, question
        try:
            plan = self.app.parse_plan(req.get('plan'))
        except PlanError as exc:
            return False, 'plan refused:\n' + '\n'.join(exc.errors)
        problems = self.app.check_plan(plan, live=False)
        if problems:
            return False, 'plan refused:\n' + '\n'.join(problems)
        self.plan = plan
        self.plan_hash = hashlib.sha256(
            json.dumps(plan.raw, sort_keys=True).encode('utf-8')
        ).hexdigest()[:12]
        self.clock_offset_s = (plan.laptop_utc - time.time()) if plan.laptop_utc else None
        self.next_trial = 1
        self.ctx = None
        self.state = READY
        # The autopilot returns here if the link drops; set it now so the
        # operator sees it in QGroundControl before starting.
        self.send_set_home(plan.home)
        self.message = f'{plan.experiment} plan {plan.name!r} loaded, {plan.trial_count} trials'
        return True, self.message

    def _start(self, _req: dict[str, Any]) -> tuple[bool, str]:
        if self.state not in FINISHED:
            return False, f'cannot start while {self.state}'
        if self.plan is None:
            return False, 'no plan loaded'
        question = self.app.blocking_question()
        if question:
            return False, question
        problems = self._preflight(self.plan)
        if problems:
            return False, 'not starting:\n' + '\n'.join(problems)
        root = mounted_log_root(
            Path(str(self.get_parameter('log_root').value)),
            require_mount=bool(self.get_parameter('require_log_mount').value),
        )
        if root is None:
            return False, 'not starting: experiment_logs is not mounted (start the mission first)'
        self.recorder = RunRecorder(root, self.plan.name)
        self.last_recorder = self.recorder
        self.recorder.write_plan(self.plan.raw, self._run_metadata())
        self.ctx = RunContext(self.plan, self.world)
        self.cancel_reason = ''
        self.next_trial = 1
        self.state = STARTING
        self.message = 'engaging the helm'
        self.engaged_at = None
        self._engage_helm()
        return True, f'starting; logging to {self.recorder.dir}'

    def _preflight(self, plan: MissionPlan) -> list[str]:
        problems = []
        if self.frontseat_mode != 'GUIDED':
            problems.append(f'autopilot is in {self.frontseat_mode or "unknown"} mode, not GUIDED')
        pos = self.fresh_position()
        if pos is None:
            problems.append('no fresh GPS fix')
        elif not plan.geofence.contains(pos):
            problems.append('the vehicle is outside the plan geofence')
        elif plan.operating_area is not None and not plan.operating_area.contains(pos):
            problems.append('the vehicle is outside the operating area')
        if self.heading_deg is None:
            problems.append('no heading')
        if not self.engage_client.service_is_ready():
            problems.append('the helm is not answering /helm/engage')
        problems += self.app.check_plan(plan, live=True)
        problems += self._home_and_failsafe_problems(plan)
        problems += self._disk_problems()
        if bool(self.get_parameter('require_module_health').value):
            problems += self._health_problems(self.app.required_modules(plan))
        problems += self.app.preflight(plan)
        return problems

    def _home_and_failsafe_problems(self, plan: MissionPlan) -> list[str]:
        problems = []
        home = self.autopilot_home
        if home is None:
            problems.append('no autopilot home reported yet (gateway /frontseat/home)')
        elif home.distance_m(plan.home) > HOME_MATCH_M:
            self.send_set_home(plan.home)
            problems.append(
                f'autopilot home is {home.distance_m(plan.home):.1f} m from the plan home; '
                'setting it now, start again in a few seconds'
            )
        if bool(self.get_parameter('require_gcs_failsafe').value):
            fs = self.failsafe
            enable, action = fs.get('FS_GCS_ENABLE'), fs.get('FS_ACTION')
            if enable is None or action is None:
                problems.append('autopilot failsafe parameters not read yet (gateway)')
            elif int(enable) < 1 or int(action) not in FS_ACTIONS_HOME:
                problems.append(
                    f'autopilot will not return home on link loss (FS_GCS_ENABLE={enable:g}, '
                    f'FS_ACTION={action:g}); in QGroundControl set FS_GCS_ENABLE to 1 or 2 '
                    'and FS_ACTION to 3 (SmartRTL, else RTL) or 1 (RTL); 4 can end in Hold'
                )
        return problems

    def _disk_problems(self) -> list[str]:
        root = Path(str(self.get_parameter('log_root').value))
        try:
            free = shutil.disk_usage(root if root.exists() else root.parent).free
        except OSError:
            return []
        if free < MIN_FREE_BYTES:
            return [f'only {free / 1024**3:.1f} GB free for logs']
        return []

    def _health_problems(self, names: list[str]) -> list[str]:
        problems = []
        for name in names:
            status = self.module_health.get(name)
            if status is None:
                problems.append(f'module {name} has not reported health')
            elif status.level != OK:
                problems.append(f'module {name}: {status.message}')
        return problems

    def _run_metadata(self) -> dict[str, Any]:
        return {
            'application': self.app.name,
            'plan_hash': self.plan_hash,
            'mission_id': os.environ.get('MISSION_ID', ''),
            'vehicle': os.environ.get('VEHICLE_NAME', ''),
            'vehicle_clock_at_start': time.time(),
            'clock_offset_s': self.clock_offset_s,
            'clock_note': 'laptop UTC minus vehicle clock at plan upload; add it to t',
        }

    def _pause(self, _req: dict[str, Any]) -> tuple[bool, str]:
        if self.state != RUNNING:
            return False, f'cannot pause while {self.state}'
        self._halt_tree()
        _, self.next_trial = self._trial_progress()
        self._save_results()
        self.hold_point = self.fresh_position()
        self.state = PAUSED
        if self.ctx is not None:
            self.ctx.set_stage('paused', 'paused, holding position', self.hold_point)
        self.message = f'paused in trial {self.next_trial}; resume reruns it from the start'
        return True, self.message

    def _resume(self, _req: dict[str, Any]) -> tuple[bool, str]:
        if self.state != PAUSED:
            return False, f'cannot resume while {self.state}'
        if self.helm.state in ('idle', ''):
            return False, 'the helm is not engaged any more; stop and start again'
        self._build(self.next_trial)
        self.state = RUNNING
        self.message = f'resumed at trial {self.next_trial}'
        return True, self.message

    def _skip(self, _req: dict[str, Any]) -> tuple[bool, str]:
        if self.state not in (RUNNING, PAUSED) or self.ctx is None:
            return False, f'cannot skip while {self.state}'
        in_progress, upcoming = self._trial_progress()
        skipped = in_progress if in_progress is not None else upcoming
        entry = self.ctx.result(skipped)
        entry['end_reason'] = 'skipped'
        self.event('trial_skipped', trial=skipped)
        self._save_results()
        self.next_trial = skipped + 1
        if self.plan is not None and self.next_trial > self.plan.trial_count:
            self._halt_tree()
            self.state = RETURNING
            self.message = 'last trial skipped; returning home'
            return True, self.message
        self._halt_tree()
        if self.state == PAUSED:
            # Skipping while paused stays paused: resume starts the next trial.
            self.message = f'trial {skipped} skipped; paused before trial {self.next_trial}'
            return True, self.message
        self._build(self.next_trial)
        self.state = RUNNING
        self.message = f'trial {skipped} skipped; running trial {self.next_trial}'
        return True, self.message

    def _stop_request(self, _req: dict[str, Any]) -> tuple[bool, str]:
        if self.state in (IDLE, *FINISHED):
            return False, f'nothing to stop ({self.state})'
        self._finish(STOPPED, 'stopped by the operator')
        return True, 'stopped; the helm stopped the vehicle'

    def _clear(self, _req: dict[str, Any]) -> tuple[bool, str]:
        if self.state in ACTIVE_STATES:
            return False, f'cannot clear while {self.state}'
        question = self.app.blocking_question()
        if question:
            return False, question
        self.plan = None
        self.ctx = None
        self.state = IDLE
        self.message = 'no plan loaded'
        return True, 'plan cleared'

    def _set_home_request(self, req: dict[str, Any]) -> tuple[bool, str]:
        """Set the autopilot's home from the console (outside a run)."""
        if self.state in ACTIVE_STATES:
            return False, f'cannot move home while {self.state}; the plan sets it'
        try:
            home = LatLon(float(req['lat']), float(req['lon']))
        except (KeyError, TypeError, ValueError):
            return False, 'set_home needs numeric lat and lon'
        if not (math.isfinite(home.lat) and math.isfinite(home.lon)):
            return False, 'set_home needs finite lat and lon'
        pos = self.fresh_position()
        if pos is not None and pos.distance_m(home) > 1000.0:
            return False, 'home is more than 1 km from the vehicle'
        self.send_set_home(home)
        return True, 'autopilot home sent; it shows in QGroundControl once accepted'

    def _engage_helm(self) -> None:
        plan = self.plan
        assert plan is not None
        req = Engage.Request()
        req.engage = True
        req.requester = SOURCE
        req.fence_latitude = plan.geofence.center.lat
        req.fence_longitude = plan.geofence.center.lon
        req.fence_radius_m = plan.geofence.radius_m
        req.fence_abort_margin_m = FENCE_ABORT_MARGIN_M
        req.max_speed_mps = self.app.max_speed_mps
        req.max_yaw_rate_rps = self.app.max_yaw_rate_rps
        self.engage_client.call_async(req).add_done_callback(self._on_engaged)

    def _on_engaged(self, future: Future) -> None:
        response = future.result()
        if response is None or not response.success:
            text = response.message if response else 'no answer'
            self._finish(STOPPED, f'the helm refused to engage: {text}')
            return
        assert self.plan is not None and self.ctx is not None
        self.run_started = time.monotonic()
        self.engaged_at = time.monotonic()
        self.event('run_start', plan=self.plan.name, experiment=self.plan.experiment)
        self.app.on_run_start(self.ctx)
        self._build(1)
        self.state = RUNNING
        self.message = 'running'
        self._status_dirty = True

    def _release_helm(self) -> None:
        if self.engage_client.service_is_ready():
            req = Engage.Request()
            req.engage = False
            req.requester = SOURCE
            self.engage_client.call_async(req)

    def _trial_progress(self) -> tuple[int | None, int]:
        """Return (trial in progress or None, trial to run next).

        A trial is in progress from its start until it completes or is skipped; an aborted
        or paused trial stays the one to run next, so resuming reruns it from its start.
        """
        ctx = self.ctx
        if ctx is None or ctx.trial == 0:
            return None, self.next_trial
        entry = next((e for e in ctx.results if e['trial'] == ctx.trial), None)
        if entry is not None and entry.get('end_reason') in ('incomplete', 'aborted'):
            return ctx.trial, ctx.trial
        return None, ctx.trial + 1

    def _build(self, first_trial: int) -> None:
        assert self.ctx is not None
        self.ctx.abort_reason = ''
        self.tree = self.app.build_tree(self.ctx, first_trial)
        self.next_trial = first_trial
        self.outside_since = None
        self.event('tree_built', first_trial=first_trial)

    def _halt_tree(self) -> None:
        if self.tree is not None:
            self.tree.stop(Status.INVALID)
        self.tree = None
        self.app.on_halt()

    def _tick(self) -> None:
        self.world.directive = None
        for skill in self.world.skills.values():
            skill.tick()
        self.app.tick()
        if self.state == RUNNING:
            self._tick_running()
        elif self.state == RETURNING:
            self._tick_returning()
        elif self.state == PAUSED and self.hold_point is not None:
            self.world.directive = {'kind': 'hold', **ll(self.hold_point)}
        self._publish_directive()
        now = time.monotonic()
        if self._status_dirty or now - self._last_status >= STATUS_PERIOD_S:
            self._publish_status()

    def _tick_running(self) -> None:
        assert self.ctx is not None and self.plan is not None
        if not self._helm_engaged():
            self._finish(STOPPED, f'the helm disengaged: {self.helm.reason or "no reason given"}')
            return
        if self._left_operating_area():
            self._cancel_run('left the operating area')
            return
        if self.tree is None:
            return
        self.tree.tick_once()
        _, self.next_trial = self._trial_progress()
        if self.tree.status == Status.SUCCESS:
            self._save_results()
            self._finish(COMPLETED, 'all trials done')
        elif self.tree.status == Status.FAILURE:
            self._abort_trial(self.ctx.abort_reason or 'a behaviour failed')
        else:
            self._save_results_if_changed()
            if self.world.directive is None:
                # Between behaviours (e.g. a parallel waiting on a skill after arriving):
                # hold where the vehicle is.
                pos = self.fresh_position()
                if self.hold_point is None and pos is not None:
                    self.hold_point = pos
                if self.hold_point is not None:
                    self.world.directive = {'kind': 'hold', **ll(self.hold_point)}
            else:
                self.hold_point = None

    def _tick_returning(self) -> None:
        assert self.plan is not None
        if not self._helm_engaged():
            self._finish(STOPPED, f'the helm disengaged: {self.helm.reason or "no reason given"}')
            return
        home = self.plan.home
        self.world.directive = {'kind': 'waypoint', **ll(home)}
        pos = self.fresh_position()
        if pos is not None and pos.distance_m(home) <= self.plan.arrive_tol_m:
            self.hold_point = home
            if self.cancel_reason:
                self._finish(CANCELLED, f'cancelled ({self.cancel_reason}); at home')
            elif self.plan.trial_count < self.next_trial:
                self._finish(COMPLETED, 'returned home after the last trial')
            else:
                self.state = PAUSED
                self.message = f'{self.message}; at home, paused before trial {self.next_trial}'
                if self.ctx is not None:
                    self.ctx.set_stage(
                        'paused', f'at home, paused before trial {self.next_trial}', home
                    )
                self.event('paused_at_home', next_trial=self.next_trial)
            self._status_dirty = True

    def _abort_trial(self, reason: str) -> None:
        assert self.ctx is not None and self.plan is not None
        trial, self.next_trial = self._trial_progress()
        # Halt first: an interrupted behaviour records itself.
        self._halt_tree()
        if trial is not None:
            entry = self.ctx.result(trial)
            entry['end_reason'] = 'aborted'
            entry['abort_reason'] = reason
        self.event('trial_aborted', trial=trial, reason=reason)
        self._save_results()
        self.state = RETURNING
        what = f'trial {trial}' if trial is not None else 'run'
        self.message = f'{what} aborted: {reason}; returning home, then pausing'
        self.ctx.set_stage('returning', 'returning home, then pausing', self.plan.home)
        self.get_logger().warning(self.message)

    def _cancel_run(self, reason: str) -> None:
        """End the whole run: record the interrupted trial, drive home, then stop."""
        assert self.ctx is not None and self.plan is not None
        trial, _ = self._trial_progress()
        self._halt_tree()
        if trial is not None:
            entry = self.ctx.result(trial)
            entry['end_reason'] = 'cancelled'
            entry['abort_reason'] = reason
        self.cancel_reason = reason
        self.event('run_cancelled', trial=trial, reason=reason)
        self._save_results()
        self.state = RETURNING
        self.message = f'run cancelled: {reason}; returning home'
        self.ctx.set_stage('returning', 'run cancelled, returning home', self.plan.home)
        self.get_logger().warning(self.message)

    def _helm_engaged(self) -> bool:
        if self.engaged_at is None:
            return False
        if time.monotonic() - self.engaged_at < HELM_ENGAGE_GRACE_S:
            return True
        return self.helm.state not in ('idle', '')

    def _left_operating_area(self) -> bool:
        assert self.plan is not None
        area = self.plan.operating_area
        pos = self.fresh_position()
        if area is None or pos is None or area.contains(pos):
            self.outside_since = None
            return False
        now = time.monotonic()
        if self.outside_since is None:
            self.outside_since = now
            self.event('outside_operating_area', lat=pos.lat, lon=pos.lon)
        return now - self.outside_since >= OUTSIDE_AREA_GRACE_S

    def _finish(self, state: str, message: str) -> None:
        self._halt_tree()
        if self.state in ACTIVE_STATES:
            self._release_helm()
        self.state = state
        self.message = message
        self.engaged_at = None
        self.event('run_end', state=state, message=message)
        self._save_results()
        self.app.on_run_end(state, message)
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        self.get_logger().info(f'Run {state}: {message}')
        self._status_dirty = True

    def event(self, kind: str, level: bytes = OK, **fields: object) -> None:
        """Record one event in the run log, if a run is open, and on /events."""
        if self.recorder is not None:
            self.recorder.event(kind, **fields)
        self.health.event(kind, level=level, **fields)
        self._status_dirty = True

    def _save_results(self) -> None:
        if self.recorder is not None and self.ctx is not None:
            self.recorder.write_results(self.ctx.results)

    def _save_results_if_changed(self) -> None:
        if self.ctx is None:
            return
        snapshot = json.dumps(self.ctx.results, sort_keys=True)
        if snapshot != getattr(self, '_results_snapshot', ''):
            self._results_snapshot = snapshot
            self._save_results()

    def _publish_directive(self) -> None:
        if self.state not in (RUNNING, RETURNING, PAUSED):
            return
        directive = self.world.directive or {'kind': 'idle'}
        kind = directive.get('kind')
        grant = HelmGrant()
        grant.header.stamp = self.get_clock().now().to_msg()
        grant.source = str(directive.get('source', '')) if kind == 'skill' else SOURCE
        self.grant_pub.publish(grant)
        if kind == 'skill' or kind == 'idle':
            return
        msg = Setpoint()
        msg.header.stamp = grant.header.stamp
        msg.source = SOURCE
        if kind in ('waypoint', 'hold'):
            msg.mode = Setpoint.MODE_POSITION if kind == 'waypoint' else Setpoint.MODE_HOLD
            msg.latitude = float(directive['lat'])
            msg.longitude = float(directive['lon'])
        elif kind == 'velocity':
            msg.mode = Setpoint.MODE_VELOCITY
            msg.speed_mps = float(directive.get('vx', 0.0))
            msg.yaw_rate_rps = float(directive.get('yaw_rate', 0.0))
        else:
            return
        self.setpoint_pub.publish(msg)

    def _publish_status(self) -> None:
        self._status_dirty = False
        self._last_status = time.monotonic()
        plan = self.plan
        ctx = self.ctx
        status: dict[str, Any] = {
            'application': self.app.name,
            'state': self.state,
            'message': self.message,
            'vehicle_clock': time.time(),
            'clock_offset_s': self.clock_offset_s,
            'last_ack': self.last_ack,
            'acks': list(self.acks.values())[-16:],
            'helm': {
                'state': self.helm.state,
                'source': self.helm.granted_source,
                'override': self.helm.override,
                'reason': self.helm.reason,
            },
            'frontseat_mode': self.frontseat_mode,
            'autopilot_home': ll(self.autopilot_home) if self.autopilot_home else None,
            'failsafe': self.failsafe,
            'plan': None,
            'trial': None,
            'results': ctx.results if ctx else [],
            'run_dir': str(self.recorder.dir) if self.recorder else '',
            'run_id': self.recorder.dir.name if self.recorder else '',
        }
        if plan is not None:
            status['plan'] = {
                'name': plan.name,
                'experiment': plan.experiment,
                'trials': plan.trial_count,
                'hash': self.plan_hash,
            }
        if ctx is not None:
            now = time.monotonic()
            status['trial'] = {
                'index': ctx.trial,
                'next': self.next_trial,
                'stage': ctx.stage,
                'label': ctx.label,
                'stage_elapsed_s': round(now - ctx.stage_started, 1) if ctx.stage_started else 0,
                'target': ll(ctx.target) if ctx.target else None,
                'run_elapsed_s': round(now - self.run_started, 1) if self.run_started else 0,
            }
        self.app.extend_status(status, ctx)
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        if self.state in (RUNNING, RETURNING, PAUSED):
            self.health.ok(f'{self.state}: {self.message}'[:120])
        elif self.state == STOPPED:
            self.health.warn(self.message[:120])
        else:
            self.health.ok(self.message[:120])


def main(args: list[str] | None = None) -> None:
    """Run the executive until shut down; stop any run cleanly on the way out."""
    rclpy.init(args=args)
    node = ExecutiveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Executive interrupted; shutting down.')
    finally:
        if node.state in ACTIVE_STATES:
            node._finish(STOPPED, 'executive shut down')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
