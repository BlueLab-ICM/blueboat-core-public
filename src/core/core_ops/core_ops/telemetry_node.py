"""5 Hz telemetry, one JSON object per line, from mission start to mission end.

experiment_logs/telemetry.jsonl. Every value has a data age: a field from a source that has gone
quiet is written as null, so a stale value cannot pass for a live one.
"""

from __future__ import annotations

import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from core_common.health import ModuleHealth
from core_interfaces.msg import HelmStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from vision_msgs.msg import Detection2DArray

from core_ops.mission_log import SCHEMA_VERSION, SyncedLog, mounted

LOG_ROOT = Path('/app/backseat_ws/experiment_logs')
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# A field older than this is written as null.
FRESH_S = 2.0
SYNC_EVERY = 25  # lines, 5 s at 5 Hz


class TelemetryNode(Node):
    """Write the slim telemetry log."""

    def __init__(self) -> None:
        super().__init__('telemetry_node')
        self.declare_parameter('log_root', str(LOG_ROOT))
        self.declare_parameter('log_hz', 5.0)
        self.root = Path(str(self.get_parameter('log_root').value))
        self.mission_id = os.environ.get('MISSION_ID', '')
        self.health = ModuleHealth(self, 'telemetry')
        self.log: SyncedLog | None = None
        if mounted(self.root):
            self.root.mkdir(parents=True, exist_ok=True)
            self.log = SyncedLog(self.root / 'telemetry.jsonl', sync_every=SYNC_EVERY)
            self.health.ok('recording')
        else:
            self.get_logger().error(f'{self.root} is not a mount: no telemetry for this mission.')
            self.health.error('experiment_logs is not mounted')

        self.values: dict[str, tuple[Any, float]] = {}
        self.helm: HelmStatus | None = None
        self.rl_active = False
        self.create_subscription(NavSatFix, '/frontseat/gps', self._on_gps, 10)
        for key, topic in (
            ('heading_deg', '/frontseat/heading'),
            ('speed_mps', '/frontseat/speed'),
            ('yaw_rate_rps', '/frontseat/yaw_rate'),
        ):
            self.create_subscription(
                Float64, topic, lambda m, k=key: self._keep(k, float(m.data)), 10
            )
        self.create_subscription(String, '/frontseat/mode', self._on_mode, 10)
        self.create_subscription(HelmStatus, '/helm/status', self._on_helm, LATCHED)
        self.create_subscription(String, '/executive/status', self._on_executive, LATCHED)
        self.create_subscription(Detection2DArray, '/perception/detections', self._on_det, 10)
        self.create_subscription(Bool, '/skills/rl_policy/active', self._on_rl_active, LATCHED)
        self.create_subscription(
            Float64MultiArray, '/skills/rl_policy/command', self._on_rl_command, 10
        )
        self.create_timer(1.0 / max(0.1, float(self.get_parameter('log_hz').value)), self._tick)

    def _keep(self, key: str, value: object) -> None:
        self.values[key] = (value, time.monotonic())

    def _fresh(self, key: str) -> object:
        item = self.values.get(key)
        if item is None or time.monotonic() - item[1] > FRESH_S:
            return None
        return item[0]

    def _age(self, key: str) -> float | None:
        item = self.values.get(key)
        return None if item is None else round(time.monotonic() - item[1], 2)

    def _on_gps(self, msg: NavSatFix) -> None:
        valid = (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and not (msg.latitude == 0.0 and msg.longitude == 0.0)
        )
        self._keep('gps', (msg.latitude, msg.longitude, msg.status.status, valid))

    def _on_mode(self, msg: String) -> None:
        self._keep('mode', msg.data)

    def _on_helm(self, msg: HelmStatus) -> None:
        self.helm = msg

    def _on_executive(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(status, dict):
            self._keep('run', status)

    def _on_det(self, msg: Detection2DArray) -> None:
        self._keep('buoy_detected', len(msg.detections) > 0)

    def _on_rl_active(self, msg: Bool) -> None:
        self.rl_active = bool(msg.data)

    def _on_rl_command(self, msg: Float64MultiArray) -> None:
        self._keep('rl_actions', [round(v, 6) for v in msg.data])

    def _tick(self) -> None:
        if self.log is None:
            return
        gps = self._fresh('gps')
        run = self._fresh('run') or {}
        trial = run.get('trial') or {}
        helm = self.helm
        self.log.write(
            {
                'v': SCHEMA_VERSION,
                'ts': datetime.now(timezone.utc).isoformat(),
                't_mono': round(time.monotonic(), 3),
                'mission_id': self.mission_id,
                'run_id': run.get('run_id', ''),
                'lat': round(gps[0], 8) if gps else None,
                'lon': round(gps[1], 8) if gps else None,
                'gps_status': gps[2] if gps else None,
                'gps_valid': bool(gps[3]) if gps else False,
                'gps_age_s': self._age('gps'),
                'heading_deg': self._fresh('heading_deg'),
                'speed_mps': self._fresh('speed_mps'),
                'yaw_rate_rps': self._fresh('yaw_rate_rps'),
                'autopilot_mode': self._fresh('mode'),
                'helm_state': helm.state if helm else None,
                'helm_source': helm.granted_source if helm else None,
                'helm_override': helm.override if helm else None,
                'state': trial.get('stage') or run.get('state'),
                'trial': trial.get('index'),
                'control_mode': 'rl' if self.rl_active else 'waypoint',
                'buoy_detected': self._fresh('buoy_detected'),
                'detection_age_s': self._age('buoy_detected'),
                'rl_actions': self._fresh('rl_actions') if self.rl_active else None,
                'rl_age_s': self._age('rl_actions'),
            }
        )

    def destroy_node(self) -> None:
        """Close the log."""
        if self.log is not None:
            self.log.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """Spin the telemetry log until shutdown."""
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
