"""The contract between the executive and an application.

An application is a set of modules plus mission trees. It plugs into the executive by naming
an object in BLUEBOAT_APPLICATION, ``package.module:factory``. The factory takes the node and
returns an :class:`Application`. Applications subclass it and override what they need.
"""

from __future__ import annotations

import importlib
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import py_trees

from core_executive.behaviours import MissionPlan, RunContext

if TYPE_CHECKING:
    from rclpy.node import Node

DEFAULT_APPLICATION = 'core_executive.survey:SurveyApplication'


class PlanError(ValueError):
    """A plan that cannot run; ``errors`` lists every problem found."""

    def __init__(self, errors: list[str]) -> None:
        super().__init__('; '.join(errors))
        self.errors = errors


class Application:
    """Base application: every hook does nothing, so a subclass overrides only what it uses."""

    name = 'base'
    # Largest velocity the helm accepts from this application. Zero keeps the helm default.
    max_speed_mps = 0.0
    max_yaw_rate_rps = 0.0

    def __init__(self, node: Node) -> None:
        self.node = node

    def make_world(self, default: Callable[[Node], object]) -> object:
        """Return the World for this application; ``default`` builds the core one."""
        return default(self.node)

    def parse_plan(self, data: object) -> MissionPlan:
        """Validate an uploaded plan, raising :class:`PlanError`."""
        raise NotImplementedError

    def build_tree(self, ctx: RunContext, first_trial: int) -> py_trees.behaviour.Behaviour:
        """Return the root behaviour running trials ``first_trial``.. of ``ctx.plan``."""
        raise NotImplementedError

    def check_plan(self, _plan: MissionPlan, *, live: bool) -> list[str]:
        """Return problems with a plan; ``live`` means the modules should be answering."""
        del live
        return []

    def preflight(self, _plan: MissionPlan) -> list[str]:
        """Return reasons not to start now."""
        return []

    def required_modules(self, _plan: MissionPlan) -> list[str]:
        """Return the health names that must report OK before this plan starts."""
        return []

    def blocking_question(self) -> str:
        """Return what the operator must answer before the next run, if anything."""
        return ''

    def tick(self) -> None:
        """Run once per executive tick, whatever the run state."""

    def on_run_start(self, _ctx: RunContext) -> None:
        """Run when a run starts."""

    def on_run_end(self, _state: str, _message: str) -> None:
        """Run when a run ends, after the tree stopped."""

    def on_halt(self) -> None:
        """Run when the tree is stopped: pause, abort or end."""

    def extend_status(self, _status: dict[str, Any], _ctx: RunContext | None) -> None:
        """Add application fields to /executive/status."""

    def requests(self) -> dict[str, Callable[[dict[str, Any]], tuple[bool, str]]]:
        """Return extra commands accepted on /executive/request."""
        return {}


def load_application(spec: str, node: Node) -> Application:
    """Import ``package.module:Name`` and build the application."""
    module_name, _, attr = (spec or DEFAULT_APPLICATION).partition(':')
    factory = getattr(importlib.import_module(module_name), attr)
    return factory(node)
