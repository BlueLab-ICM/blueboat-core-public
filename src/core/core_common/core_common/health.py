"""Report a module's health on /diagnostics and notable events on /events.

Copy this file into a module unchanged. It needs only diagnostic_msgs, which every ROS 2
image has. The source of truth is blueboat-core, src/core/core_common.
"""

import json

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR

# Latched so an event published before the recorder has discovered this node still lands.
EVENTS_QOS = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ModuleHealth:
    """Publish one health status per ``period_s`` and events on demand."""

    def __init__(self, node: Node, name: str, period_s: float = 1.0) -> None:
        self._node = node
        self.name = name
        self._level = OK
        self._message = 'starting'
        self._values: dict[str, object] = {}
        self._health_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._event_pub = node.create_publisher(DiagnosticArray, '/events', EVENTS_QOS)
        self._timer = node.create_timer(period_s, self._publish_health)

    def set(self, level: bytes, message: str, **values: object) -> None:
        """Set the status repeated on every period."""
        self._level = level
        self._message = message
        self._values = values

    def ok(self, message: str = 'running', **values: object) -> None:
        """Report that the module works."""
        self.set(OK, message, **values)

    def warn(self, message: str, **values: object) -> None:
        """Report that the module works, degraded."""
        self.set(WARN, message, **values)

    def error(self, message: str, **values: object) -> None:
        """Report that the module does not work."""
        self.set(ERROR, message, **values)

    def event(self, kind: str, level: bytes = OK, **fields: object) -> None:
        """Publish one event: a change someone reading the mission later should see."""
        status = DiagnosticStatus(level=level, name=self.name, message=kind)
        status.values = [KeyValue(key=k, value=json.dumps(v)) for k, v in fields.items()]
        self._event_pub.publish(self._array(status))

    def _publish_health(self) -> None:
        status = DiagnosticStatus(level=self._level, name=self.name, message=self._message)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in self._values.items()]
        self._health_pub.publish(self._array(status))

    def _array(self, status: DiagnosticStatus) -> DiagnosticArray:
        msg = DiagnosticArray()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.status.append(status)
        return msg
