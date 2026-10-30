"""Behaviours any mission can use.

They act only through World, so trees run the same on the vessel and in tests. Mission-specific
behaviours live in the application.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Protocol

import py_trees
from py_trees.common import Status

from core_executive.geometry import LatLon, Polygon


class MissionPlan(Protocol):
    """What the executive needs from an application's plan."""

    name: str
    experiment: str
    home: LatLon
    operating_area: Polygon | None
    arrive_tol_m: float
    laptop_utc: float | None
    raw: dict[str, Any]

    @property
    def trial_count(self) -> int:
        """Return how many trials the plan holds."""


class Skill(Protocol):
    """A module's action, as a behaviour sees it."""

    state: str  # idle | active | done
    outcome: str
    message: str
    result: dict[str, Any]

    def start(self, params: dict[str, Any]) -> None:
        """Send a goal, replacing any goal in progress."""

    def cancel(self) -> None:
        """Cancel the goal in progress."""

    def feedback_age(self) -> float:
        """Return seconds since the skill last reported."""


class World(Protocol):
    """The vehicle as the trees see it."""

    def now(self) -> float:
        """Return monotonic time in seconds."""

    def position(self) -> LatLon | None:
        """Return the latest valid GPS position, or None if there is none."""

    def heading_rad(self) -> float | None:
        """Return the heading (NED, 0 = north, clockwise positive) in radians."""

    def command(self, directive: dict[str, Any]) -> None:
        """Set what the vehicle should do this tick.

        Kinds: ``waypoint`` and ``hold`` (lat, lon), ``velocity`` (vx, yaw_rate), ``skill``
        (source: the module that may drive), ``idle``.
        """

    def event(self, kind: str, **fields: object) -> None:
        """Record one event in the run log."""

    def skill(self, name: str) -> Skill:
        """Return the client of skill ``name``."""


@dataclass
class RunContext:
    """State shared by every behaviour of one run."""

    plan: MissionPlan
    world: Any
    results: list[dict[str, Any]] = field(default_factory=list)
    stage: str = 'idle'
    label: str = ''
    stage_started: float = 0.0
    trial: int = 0
    target: LatLon | None = None
    abort_reason: str = ''
    # Free for the application: set by one behaviour, read by another.
    notes: dict[str, Any] = field(default_factory=dict)

    def set_stage(self, stage: str, label: str, target: LatLon | None = None) -> None:
        """Record the current stage, for the status and the log."""
        if stage != self.stage or label != self.label:
            self.world.event('stage', stage=stage, label=label, trial=self.trial)
            self.stage_started = self.world.now()
        self.stage, self.label, self.target = stage, label, target

    def result(self, trial: int) -> dict[str, Any]:
        """Return the result entry of ``trial``, creating it if needed."""
        for entry in self.results:
            if entry['trial'] == trial:
                return entry
        entry = {'trial': trial, 'end_reason': 'incomplete'}
        self.results.append(entry)
        return entry

    def trial_prefix(self) -> str:
        """Return e.g. ``trial 2/5`` for labels, or ``setup`` before the first trial."""
        if self.trial == 0:
            return 'setup'
        return f'trial {self.trial}/{self.plan.trial_count}'


def ll(p: LatLon) -> dict[str, float]:
    """Return a position as a JSON-ready dict."""
    return {'lat': p.lat, 'lon': p.lon}


def seq(name: str, children: list[py_trees.behaviour.Behaviour]) -> py_trees.composites.Sequence:
    """Return a Sequence that resumes at the running child."""
    return py_trees.composites.Sequence(name=name, memory=True, children=children)


class ContextBehaviour(py_trees.behaviour.Behaviour):
    """A behaviour bound to one run."""

    def __init__(self, name: str, ctx: RunContext) -> None:
        super().__init__(name)
        self.ctx = ctx
        self.world = ctx.world


class StartTrial(ContextBehaviour):
    """Open the result entry of a trial and describe what it is about."""

    def __init__(self, ctx: RunContext, trial: int, info: dict[str, Any]) -> None:
        super().__init__(f'start trial {trial}', ctx)
        self.trial = trial
        self.info = info

    def update(self) -> Status:
        """Record the start and succeed at once.

        A trial that is run again (after an abort or a pause) keeps what its earlier
        attempts recorded under ``previous_attempts``; nothing measured is overwritten.
        """
        self.ctx.trial = self.trial
        entry = self.ctx.result(self.trial)
        if set(entry) - {'trial', 'end_reason', 'previous_attempts'}:
            attempt = {k: v for k, v in entry.items() if k != 'previous_attempts'}
            if attempt.get('end_reason') == 'incomplete':
                attempt['end_reason'] = 'interrupted'
            attempts = [*entry.get('previous_attempts', []), attempt]
            entry.clear()
            entry.update({'trial': self.trial, 'previous_attempts': attempts})
        entry['end_reason'] = 'incomplete'
        entry.update(self.info)
        entry['started_t_mono'] = round(self.world.now(), 3)
        self.world.event('trial_start', trial=self.trial, **self.info)
        return Status.SUCCESS


class EndTrial(ContextBehaviour):
    """Close a trial's result entry."""

    def __init__(self, ctx: RunContext, trial: int) -> None:
        super().__init__(f'end trial {trial}', ctx)
        self.trial = trial

    def update(self) -> Status:
        """Mark the trial completed and succeed at once."""
        entry = self.ctx.result(self.trial)
        if entry.get('end_reason') == 'incomplete':
            entry['end_reason'] = 'completed'
        entry['ended_t_mono'] = round(self.world.now(), 3)
        self.world.event('trial_end', trial=self.trial, result=entry)
        return Status.SUCCESS


class NavigateTo(ContextBehaviour):
    """Drive to a point and succeed within ``tol_m``; fail if it takes far too long."""

    def __init__(
        self, ctx: RunContext, target: LatLon, tol_m: float, stage: str, label: str
    ) -> None:
        super().__init__(f'navigate: {label}', ctx)
        self.target = target
        self.tol_m = max(0.5, tol_m)
        self.stage = stage
        self.label = label
        self.deadline = math.inf

    def initialise(self) -> None:
        """Set a generous deadline from the distance to go."""
        pos = self.world.position()
        dist = pos.distance_m(self.target) if pos is not None else 100.0
        # 0.2 m/s plus two minutes: only a vehicle that is not making way misses it.
        self.deadline = self.world.now() + 120.0 + dist / 0.2

    def update(self) -> Status:
        """Command the waypoint until arrival."""
        self.ctx.set_stage(self.stage, f'{self.ctx.trial_prefix()}: {self.label}', self.target)
        self.world.command({'kind': 'waypoint', **ll(self.target)})
        pos = self.world.position()
        if pos is not None and pos.distance_m(self.target) <= self.tol_m:
            return Status.SUCCESS
        if self.world.now() > self.deadline:
            self.ctx.abort_reason = f'did not reach {self.label} in time'
            self.world.event('navigate_timeout', target=ll(self.target), label=self.label)
            return Status.FAILURE
        return Status.RUNNING


class HoldFor(ContextBehaviour):
    """Hold position at a point for a fixed time."""

    def __init__(
        self, ctx: RunContext, at: LatLon, duration_s: float, stage: str, label: str
    ) -> None:
        super().__init__(f'hold: {label}', ctx)
        self.at = at
        self.duration_s = duration_s
        self.stage = stage
        self.label = label
        self.until = 0.0

    def initialise(self) -> None:
        """Start the hold timer."""
        self.until = self.world.now() + self.duration_s

    def update(self) -> Status:
        """Command the hold point until the time is up."""
        self.ctx.set_stage(self.stage, f'{self.ctx.trial_prefix()}: {self.label}', self.at)
        self.world.command({'kind': 'hold', **ll(self.at)})
        return Status.SUCCESS if self.world.now() >= self.until else Status.RUNNING


class RunSkill(ContextBehaviour):
    """Run a module's skill to its end, letting it drive, and fail if it fails or goes silent.

    A skill that ends with an outcome in ``fail_on`` fails the behaviour; any other outcome
    succeeds. ``timeout_s`` of zero runs until the skill ends. The goal is cancelled however
    the behaviour ends, so an interrupted skill stops driving.
    """

    SILENCE_GRACE_S = 8.0
    SILENCE_LIMIT_S = 3.0

    def __init__(
        self,
        ctx: RunContext,
        skill: str,
        params: dict[str, Any],
        stage: str,
        label: str,
        timeout_s: float = 0.0,
        drives: bool = True,
        fail_on: tuple[str, ...] = ('failed',),
    ) -> None:
        super().__init__(f'skill: {label}', ctx)
        self.skill_name = skill
        self.params = params
        self.stage = stage
        self.label = label
        self.timeout_s = timeout_s
        self.drives = drives
        self.fail_on = fail_on
        self.started = 0.0

    def initialise(self) -> None:
        """Send the goal."""
        self.started = self.world.now()
        self.world.skill(self.skill_name).start(self.params)
        self.world.event('skill_start', skill=self.skill_name, params=self.params)

    def update(self) -> Status:
        """Let the skill drive until it ends."""
        skill = self.world.skill(self.skill_name)
        self.ctx.set_stage(self.stage, f'{self.ctx.trial_prefix()}: {self.label}')
        if self.drives:
            self.world.command({'kind': 'skill', 'source': self.skill_name})
        elapsed = self.world.now() - self.started
        if skill.state == 'done':
            self.world.event('skill_end', skill=self.skill_name, outcome=skill.outcome)
            if skill.outcome in self.fail_on:
                self.ctx.abort_reason = f'{self.skill_name}: {skill.message}'
                return Status.FAILURE
            return Status.SUCCESS
        if self.timeout_s > 0.0 and elapsed >= self.timeout_s:
            self.world.event('skill_timeout', skill=self.skill_name)
            return Status.SUCCESS
        if elapsed > self.SILENCE_GRACE_S and skill.feedback_age() > self.SILENCE_LIMIT_S:
            self.ctx.abort_reason = f'{self.skill_name} stopped reporting'
            return Status.FAILURE
        return Status.RUNNING

    def terminate(self, _new_status: Status) -> None:
        """Stop the skill if it is still going, however this behaviour ended."""
        skill = self.world.skill(self.skill_name)
        if skill.state != 'done':
            skill.cancel()
