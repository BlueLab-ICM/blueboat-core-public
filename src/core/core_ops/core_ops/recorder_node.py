"""Write the mission's event log: one fsynced timeline of what happened and why.

Takes /events from every node, WARN and above from /rosout, health changes from /diagnostics,
and GPS clock pairs. Publishes the module health summary on /system/health. Writes
experiment_logs/events.jsonl and, once, autopilot_params.json.
"""

from __future__ import annotations

import json
import os
import shutil
import time
from pathlib import Path
from typing import Any

import rclpy
from core_common.health import EVENTS_QOS, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import TimeReference
from std_msgs.msg import String

from core_ops.mission_log import (
    ERROR,
    LEVEL_NAMES,
    OK,
    SCHEMA_VERSION,
    WARN,
    ClockPairs,
    HealthBoard,
    Repeats,
    SyncedLog,
    mounted,
)

LOG_ROOT = Path('/app/backseat_ws/experiment_logs')
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
ROSOUT_WARN = 30
MIN_FREE_BYTES = 1024**3


def _level(value: object) -> int:
    """Return a DiagnosticStatus level as an int: rclpy gives bytes."""
    return int.from_bytes(value, 'big') if isinstance(value, bytes) else int(value)


def _boot_id() -> str:
    try:
        return Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip()
    except OSError:
        return ''


class RecorderNode(Node):
    """Keep the event log and the health summary."""

    def __init__(self) -> None:
        super().__init__('recorder_node')
        self.declare_parameter('log_root', str(LOG_ROOT))
        self.root = Path(str(self.get_parameter('log_root').value))
        self.mission_id = os.environ.get('MISSION_ID', '')
        self.boot_id = _boot_id()
        self.health = ModuleHealth(self, 'recorder')
        self.board = HealthBoard()
        self.clock_pairs = ClockPairs()
        self.repeats = Repeats()
        self.params_saved = False
        self.log: SyncedLog | None = None
        if mounted(self.root):
            self.root.mkdir(parents=True, exist_ok=True)
            self.log = SyncedLog(self.root / 'events.jsonl')
            self.health.ok('recording')
        else:
            self.get_logger().error(f'{self.root} is not a mount: no event log for this mission.')
            self.health.error('experiment_logs is not mounted')

        self.health_pub = self.create_publisher(DiagnosticArray, '/system/health', LATCHED)
        self.create_subscription(DiagnosticArray, '/events', self._on_event, EVENTS_QOS)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 50)
        self.create_subscription(Log, '/rosout', self._on_rosout, 100)
        self.create_subscription(TimeReference, '/frontseat/time', self._on_time, 10)
        self.create_subscription(String, '/frontseat/params', self._on_params, LATCHED)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self._sweep, clock=steady)
        self._write('recorder', OK, 'recorder_started', {'mission_id': self.mission_id})

    def _write(self, source: str, level: int, kind: str, fields: dict[str, Any]) -> None:
        if self.log is None:
            return
        now = time.time()
        offset = self.clock_pairs.offset_s
        self.log.write(
            {
                'v': SCHEMA_VERSION,
                't_mono': round(time.monotonic(), 3),
                't_system': round(now, 3),
                't_utc': round(now + offset, 3) if offset is not None else None,
                'boot_id': self.boot_id,
                'mission_id': self.mission_id,
                'source': source,
                'level': LEVEL_NAMES.get(level, str(level)),
                'kind': kind,
                'fields': fields,
            }
        )

    def _on_event(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            fields: dict[str, Any] = {}
            for item in status.values:
                try:
                    fields[item.key] = json.loads(item.value)
                except json.JSONDecodeError:
                    fields[item.key] = item.value
            self._write(status.name, _level(status.level), status.message, fields)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        now = time.monotonic()
        for status in msg.status:
            level = _level(status.level)
            previous = self.board.report(status.name, level, status.message, now)
            if previous is not None:
                self._write(
                    status.name,
                    level,
                    'health',
                    {'was': LEVEL_NAMES.get(previous.level, 'unknown'), 'message': status.message},
                )

    def _on_rosout(self, msg: Log) -> None:
        if msg.level < ROSOUT_WARN or msg.name == self.get_name():
            return
        held = self.repeats.admit(msg.name, msg.msg, time.monotonic())
        if held is None:
            return
        level = WARN if msg.level < 40 else ERROR
        self._write(msg.name, level, 'log', {'message': msg.msg, 'repeats_held': held})

    def _on_time(self, msg: TimeReference) -> None:
        gps = msg.time_ref.sec + msg.time_ref.nanosec * 1e-9
        system = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.clock_pairs.update(system, gps, time.monotonic()):
            self._write(
                'gateway',
                OK,
                'clock',
                {
                    'system_s': round(system, 3),
                    'gps_s': round(gps, 3),
                    'offset_s': round(gps - system, 3),
                },
            )

    def _on_params(self, msg: String) -> None:
        if self.params_saved or self.log is None:
            return
        self.params_saved = True
        path = self.root / 'autopilot_params.json'
        try:
            path.write_text(msg.data, encoding='utf-8')
        except OSError as exc:
            self.get_logger().error(f'Cannot save autopilot parameters: {exc}')

    def _sweep(self) -> None:
        now = time.monotonic()
        for name in self.board.sweep(now):
            self._write(name, ERROR, 'health', {'was': 'reporting', 'message': 'stale'})
        summary = DiagnosticArray()
        summary.header.stamp = self.get_clock().now().to_msg()
        for name, state in sorted(self.board.modules.items()):
            status = DiagnosticStatus(
                level=bytes([min(state.level, 3)]), name=name, message=state.message
            )
            status.values = [KeyValue(key='age_s', value=f'{now - state.heard:.1f}')]
            summary.status.append(status)
        self.health_pub.publish(summary)
        if self.log is None:
            return
        free = shutil.disk_usage(self.root).free
        if free < MIN_FREE_BYTES:
            self.health.error(f'only {free / 1024**3:.1f} GB free')
        else:
            self.health.ok('recording', free_gb=round(free / 1024**3, 1))

    def destroy_node(self) -> None:
        """Close the event log."""
        if self.log is not None:
            self._write('recorder', OK, 'recorder_stopped', {})
            self.log.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Spin the recorder until shutdown."""
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
