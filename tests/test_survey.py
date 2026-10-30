"""Tests of the default application."""

import pytest

pytest.importorskip('py_trees')
from core_executive.application import PlanError, load_application
from core_executive.behaviours import RunContext
from core_executive.geometry import LatLon, offset
from core_executive.survey import SurveyApplication
from py_trees.common import Status

HOME = LatLon(41.0, 2.0)


class FakeSkill:
    def __init__(self) -> None:
        self.state = 'idle'
        self.outcome = ''
        self.message = ''
        self.result: dict = {}
        self.params: dict | None = None
        self.cancelled = False
        self.ends_after = 5

    def start(self, params: dict) -> None:
        self.params = params
        self.state = 'active'

    def cancel(self) -> None:
        self.cancelled = True

    def feedback_age(self) -> float:
        return 0.1


class World:
    def __init__(self) -> None:
        self.pos = HOME
        self.t = 0.0
        self.directive: dict | None = None
        self.skills: dict[str, FakeSkill] = {}
        self.visited: list[str] = []

    def now(self) -> float:
        return self.t

    def position(self) -> LatLon:
        return self.pos

    def heading_rad(self) -> float:
        return 0.0

    def command(self, directive: dict) -> None:
        self.directive = directive

    def event(self, kind: str, **fields: object) -> None:
        self.visited.append(kind)

    def skill(self, name: str) -> FakeSkill:
        return self.skills.setdefault(name, FakeSkill())

    def step(self) -> None:
        self.t += 1.0
        d = self.directive or {}
        if d.get('kind') == 'waypoint':
            self.pos = LatLon(d['lat'], d['lon'])
        if d.get('kind') == 'skill':
            skill = self.skills[d['source']]
            skill.ends_after -= 1
            if skill.ends_after <= 0:
                skill.state, skill.outcome = 'done', 'completed'


def plan(**extra: object) -> dict:
    return {
        'name': 'test',
        'home': [HOME.lat, HOME.lon],
        'geofence': {'lat': HOME.lat, 'lon': HOME.lon, 'radius_m': 100.0},
        **extra,
    }


def run(data: dict) -> tuple[Status, World, RunContext]:
    app = SurveyApplication(None)
    world = World()
    ctx = RunContext(app.parse_plan(data), world)
    tree = app.build_tree(ctx, 1)
    for _ in range(100):
        tree.tick_once()
        if tree.status != Status.RUNNING:
            break
        world.step()
    return tree.status, world, ctx


def test_survey_visits_waypoints_and_returns() -> None:
    p = offset(HOME, 30.0, 0.0)
    status, world, ctx = run(plan(waypoints=[[p.lat, p.lon]]))
    assert status == Status.SUCCESS
    assert world.pos == HOME
    assert ctx.results[0]['end_reason'] == 'completed'


def test_a_skill_gets_the_helm_until_it_ends() -> None:
    status, world, _ = run(plan(skill='my_module', skill_params={'x': 1}))
    assert status == Status.SUCCESS
    assert world.skills['my_module'].params == {'x': 1}
    assert not world.skills['my_module'].cancelled


def test_a_failed_skill_fails_the_tree() -> None:
    app = SurveyApplication(None)
    world = World()
    ctx = RunContext(app.parse_plan(plan(skill='my_module')), world)
    tree = app.build_tree(ctx, 1)
    tree.tick_once()
    tree.tick_once()
    skill = world.skills['my_module']
    skill.state, skill.outcome, skill.message = 'done', 'failed', 'broke'
    tree.tick_once()
    assert tree.status == Status.FAILURE
    assert 'broke' in ctx.abort_reason


@pytest.mark.parametrize(
    'bad',
    [
        {'name': 'bad name!'},
        {'geofence': {'lat': 41.0, 'lon': 2.0, 'radius_m': 0.0}},
        {'waypoints': []},
        {'waypoints': [[42.0, 2.0]]},
    ],
)
def test_bad_plans_are_refused(bad: dict) -> None:
    data = {**plan(waypoints=[[HOME.lat, HOME.lon]]), **bad}
    with pytest.raises(PlanError):
        SurveyApplication(None).parse_plan(data)


def test_default_application_loads() -> None:
    assert load_application('', None).name == 'survey'
