"""Default application: follow waypoints, or hand the vehicle to one skill for a set time.

Runs with no modules at all, so a new system can drive before it has any of its own code.
"""

from __future__ import annotations

import re
from typing import Any

import py_trees

from core_executive.application import Application, PlanError
from core_executive.behaviours import (
    EndTrial,
    HoldFor,
    NavigateTo,
    RunContext,
    RunSkill,
    StartTrial,
    seq,
)
from core_executive.geometry import Geofence, LatLon, Polygon

_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,64}$')


class SurveyPlan:
    """A validated survey plan."""

    experiment = 'survey'

    def __init__(self, data: dict[str, Any]) -> None:
        errors: list[str] = []
        self.raw = data
        self.name = str(data.get('name', ''))
        if not _NAME_RE.match(self.name):
            errors.append('name: 1-64 characters from A-Z a-z 0-9 _ -')
        self.home = _position(data.get('home'), 'home', errors)
        fence = data.get('geofence') or {}
        self.geofence = Geofence(_position(fence, 'geofence', errors), _radius(fence, errors))
        self.waypoints = tuple(
            _position(p, f'waypoints[{i}]', errors) for i, p in enumerate(data.get('waypoints', []))
        )
        self.skill = str(data.get('skill', ''))
        self.skill_params = data.get('skill_params') or {}
        self.skill_timeout_s = float(data.get('skill_timeout_s', 0.0))
        if not self.waypoints and not self.skill:
            errors.append('a plan needs waypoints, a skill, or both')
        area = data.get('operating_area')
        self.operating_area = None
        if area:
            self.operating_area = Polygon(
                tuple(_position(v, f'operating_area[{i}]', errors) for i, v in enumerate(area))
            )
        self.arrive_tol_m = float(data.get('arrive_tol_m', 3.0))
        self.laptop_utc = data.get('laptop_utc')
        for point in self.waypoints:
            if not self.geofence.contains(point):
                errors.append(f'waypoint {point.lat}, {point.lon} is outside the geofence')
        if errors:
            raise PlanError(errors)

    @property
    def trial_count(self) -> int:
        """Return one: a survey is a single trial."""
        return 1


def _position(value: object, where: str, errors: list[str]) -> LatLon:
    try:
        if isinstance(value, dict):
            return LatLon(float(value['lat']), float(value['lon']))
        return LatLon(float(value[0]), float(value[1]))
    except (KeyError, IndexError, TypeError, ValueError):
        errors.append(f'{where}: needs a latitude and a longitude')
        return LatLon(0.0, 0.0)


def _radius(fence: dict[str, Any], errors: list[str]) -> float:
    try:
        radius = float(fence['radius_m'])
    except (KeyError, TypeError, ValueError):
        radius = 0.0
    if radius <= 0.0:
        errors.append('geofence.radius_m: must be positive')
    return radius


class SurveyApplication(Application):
    """Follow a waypoint list, then optionally run one skill, then return home."""

    name = 'survey'

    def parse_plan(self, data: object) -> SurveyPlan:
        """Validate a survey plan."""
        if not isinstance(data, dict):
            raise PlanError(['plan: must be a JSON object'])
        return SurveyPlan(data)

    def build_tree(self, ctx: RunContext, first_trial: int) -> py_trees.behaviour.Behaviour:
        """Return waypoints, skill, home."""
        del first_trial
        plan = ctx.plan
        tol = plan.arrive_tol_m
        children: list[py_trees.behaviour.Behaviour] = [StartTrial(ctx, 1, {})]
        for i, point in enumerate(plan.waypoints, start=1):
            children.append(NavigateTo(ctx, point, tol, 'waypoint', f'waypoint {i}'))
        if plan.skill:
            children.append(
                RunSkill(
                    ctx,
                    plan.skill,
                    plan.skill_params,
                    'skill',
                    plan.skill,
                    timeout_s=plan.skill_timeout_s,
                )
            )
        children += [
            NavigateTo(ctx, plan.home, tol, 'return_home', 'return home'),
            HoldFor(ctx, plan.home, 2.0, 'done', 'complete'),
            EndTrial(ctx, 1),
        ]
        return seq('survey', children)
