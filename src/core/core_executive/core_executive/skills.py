"""Call a module's skill over its /skills/<name> action, without blocking the tree."""

from __future__ import annotations

import json
import math
import time
from typing import Any

from action_msgs.msg import GoalStatus
from core_interfaces.action import RunSkill
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

IDLE, ACTIVE, DONE = 'idle', 'active', 'done'
# A skill server that does not appear in this long fails the goal.
SERVER_WAIT_S = 20.0


class SkillClient:
    """One skill's goal at a time. ``state`` moves idle, active, done; ``outcome`` says how."""

    def __init__(self, node: Node, name: str) -> None:
        self.name = name
        self._client = ActionClient(node, RunSkill, f'/skills/{name}')
        self.state = IDLE
        self.outcome = ''
        self.message = ''
        self.result: dict[str, Any] = {}
        self.feedback: dict[str, Any] = {}
        self._pending: str | None = None
        self._pending_since = 0.0
        self._goal: Any = None
        self._generation = 0
        self._last_report = -math.inf

    def start(self, params: dict[str, Any]) -> None:
        """Send a goal, replacing any goal in progress."""
        self.cancel()
        self._generation += 1
        self.state = ACTIVE
        self.outcome = ''
        self.message = ''
        self.result = {}
        self._pending = json.dumps(params)
        self._pending_since = time.monotonic()
        self._last_report = time.monotonic()
        self.tick()

    def tick(self) -> None:
        """Send a pending goal once the server is there, or give up."""
        if self._pending is None:
            return
        if self._client.server_is_ready():
            goal = RunSkill.Goal(params_json=self._pending)
            self._pending = None
            generation = self._generation
            future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            future.add_done_callback(lambda f: self._on_response(f, generation))
        elif time.monotonic() - self._pending_since > SERVER_WAIT_S:
            self._pending = None
            self._finish('failed', f'no /skills/{self.name} server')

    def cancel(self) -> None:
        """Cancel the goal in progress. A later result of that goal is ignored."""
        self._pending = None
        self._generation += 1
        if self._goal is not None:
            self._goal.cancel_goal_async()
            self._goal = None
        if self.state == ACTIVE:
            self.state = IDLE

    def feedback_age(self) -> float:
        """Return seconds since the skill last reported."""
        return time.monotonic() - self._last_report

    def _finish(self, outcome: str, message: str) -> None:
        self.state = DONE
        self.outcome = outcome
        self.message = message

    def _on_feedback(self, feedback: RunSkill.Impl.FeedbackMessage) -> None:
        self._last_report = time.monotonic()
        self.message = feedback.feedback.message
        try:
            self.feedback = json.loads(feedback.feedback.feedback_json or '{}')
        except json.JSONDecodeError:
            self.feedback = {}

    def _on_response(self, future: Future, generation: int) -> None:
        handle = future.result()
        if generation != self._generation:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self._finish('failed', f'{self.name} refused the goal')
            return
        self._goal = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, generation))

    def _on_result(self, future: Future, generation: int) -> None:
        if generation != self._generation:
            return
        self._goal = None
        response = future.result()
        if response is None or response.status == GoalStatus.STATUS_ABORTED:
            self._finish('failed', f'{self.name} aborted')
            return
        result = response.result
        try:
            self.result = json.loads(result.result_json or '{}')
        except json.JSONDecodeError:
            self.result = {}
        self._finish(result.outcome or 'completed', result.message)
