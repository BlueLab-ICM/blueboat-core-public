"""Talk to the executive from a shell, for systems without a console.

ros2 run core_executive executive_cli load <plan.json> | start | stop | status
"""

from __future__ import annotations

import json
import sys
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
ANSWER_WAIT_S = 15.0


class Cli(Node):
    """Send one request and wait for its answer in the status."""

    def __init__(self) -> None:
        super().__init__('executive_cli')
        self.status: dict | None = None
        self.pub = self.create_publisher(String, '/executive/request', 10)
        self.create_subscription(String, '/executive/status', self._on_status, LATCHED)

    def _on_status(self, msg: String) -> None:
        self.status = json.loads(msg.data)

    def request(self, body: dict) -> tuple[bool, str]:
        """Resend ``body`` until the status acknowledges its id."""
        req_id = uuid.uuid4().hex[:8]
        msg = String()
        msg.data = json.dumps({'id': req_id, **body})
        deadline = time.monotonic() + ANSWER_WAIT_S
        while time.monotonic() < deadline:
            self.pub.publish(msg)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                for ack in (self.status or {}).get('acks', []):
                    if ack.get('id') == req_id:
                        return bool(ack.get('ok')), str(ack.get('message'))
        return False, 'no answer from the executive'


def main() -> int:
    """Run one command; exit non-zero when it is refused."""
    args = sys.argv[1:]
    if not args or args[0] not in ('load', 'start', 'stop', 'status'):
        print(__doc__)
        return 2
    rclpy.init()
    node = Cli()
    try:
        if args[0] == 'status':
            deadline = time.monotonic() + 5.0
            while node.status is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(json.dumps(node.status, indent=2))
            return 0 if node.status is not None else 1
        body: dict = {'cmd': args[0]}
        if args[0] == 'load':
            body['plan'] = json.loads(Path(args[1]).read_text(encoding='utf-8'))
            body['plan'].setdefault('laptop_utc', time.time())
        ok, message = node.request(body)
        print(message)
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
