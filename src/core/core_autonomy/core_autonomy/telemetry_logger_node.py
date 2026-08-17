"""
Telemetry Logger — flat, analysis-ready vehicle telemetry in JSON Lines.

This complements the rosbag rather than replacing it. The bag is the complete,
replayable record of everything that happened; this is a small flat summary
that opens with pandas, ``jq`` or a spreadsheet on a machine with no ROS
installation at all:

    import pandas as pd
    df = pd.read_json('telemetry_20260817_142312.jsonl', lines=True)

Each record is a self-contained JSON object terminated by a newline, and the
file is line-buffered, so an abrupt power loss can only ever damage the final
partial line — every record written before it stays intact. That matters on a
vehicle whose backseat computer can be power-cycled by relay.

Logging starts automatically when the node comes up. Set the ``session_name``
parameter to begin a new file — useful to mark the start of a run:

    ros2 param set /telemetry_logger_node session_name transect_3
"""

import json
import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, String


# (path, require_mount) — require_mount guards a directory that only reaches
# the host through a bind mount. Without that check a probe write would succeed
# inside the container's own filesystem and the whole log would vanish when the
# container is recreated.
_LOG_DIR_CANDIDATES = [
    (Path('/app/backseat_ws/telemetry_logs'), True),
    (Path('/tmp/blueboat_telemetry_logs'), False),
]


def _dir_is_mounted(path: Path) -> bool:
    """True if ``path`` or any ancestor is a mount point."""
    p = path
    while p != p.parent:
        if os.path.ismount(p):
            return True
        p = p.parent
    return False


class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger_node')

        self.declare_parameter('session_name', 'telemetry')
        self.declare_parameter('log_hz', 5.0)

        self._log_file = None
        self._log_path: Path | None = None

        self._lat: float | None = None
        self._lon: float | None = None
        self._heading_deg: float | None = None
        self._speed_mps: float | None = None
        self._yaw_rate_rads: float | None = None
        self._battery_v: float | None = None
        self._frontseat_mode: str | None = None
        self._brain_status: str | None = None

        self.create_subscription(NavSatFix, '/frontseat/gps', self._on_gps, 10)
        self.create_subscription(Float64, '/frontseat/heading', self._on_heading, 10)
        self.create_subscription(Float64, '/frontseat/speed', self._on_speed, 10)
        self.create_subscription(Float64, '/frontseat/yaw_rate', self._on_yaw_rate, 10)
        self.create_subscription(BatteryState, '/frontseat/battery', self._on_battery, 10)
        self.create_subscription(String, '/frontseat/mode', self._on_mode, 10)
        self.create_subscription(String, '/system/brain/status', self._on_status, 10)

        self.add_on_set_parameters_callback(self._on_param_change)

        hz = max(0.1, float(self.get_parameter('log_hz').value))
        self.create_timer(1.0 / hz, self._log_tick)

        self._open_log(str(self.get_parameter('session_name').value).strip() or 'telemetry')
        self.get_logger().info('Telemetry logger online.')

    # ------------------------------------------------------------------
    # Log file lifecycle
    # ------------------------------------------------------------------

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'session_name':
                name = str(p.value).strip()
                if name:
                    self._open_log(name)
                else:
                    self._close_log()
        return SetParametersResult(successful=True)

    def _open_log(self, name: str) -> None:
        self._close_log()
        ts = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        filename = f'{name}_{ts}.jsonl'

        log_dir: Path | None = None
        for candidate, require_mount in _LOG_DIR_CANDIDATES:
            if require_mount and not _dir_is_mounted(candidate):
                continue
            try:
                candidate.mkdir(parents=True, exist_ok=True)
                probe = candidate / '.write_test'
                probe.touch()
                probe.unlink()
                log_dir = candidate
                break
            except OSError:
                continue

        if log_dir is None:
            self.get_logger().error(
                'No writable log directory found; telemetry logging disabled.')
            return

        self._log_path = log_dir / filename
        try:
            # buffering=1 → line buffered, so every completed record is flushed.
            self._log_file = open(self._log_path, 'a', buffering=1, encoding='utf-8')
            self.get_logger().info(f'Telemetry log opened: {self._log_path}')
        except OSError as exc:
            self.get_logger().error(f'Failed to open {self._log_path}: {exc}')
            self._log_file = None
            self._log_path = None

    def _close_log(self) -> None:
        if self._log_file is None:
            return
        try:
            self._log_file.flush()
            self._log_file.close()
            self.get_logger().info(f'Telemetry log closed: {self._log_path}')
        except OSError:
            pass
        self._log_file = None
        self._log_path = None

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_gps(self, msg: NavSatFix) -> None:
        self._lat = float(msg.latitude)
        self._lon = float(msg.longitude)

    def _on_heading(self, msg: Float64) -> None:
        self._heading_deg = float(msg.data)

    def _on_speed(self, msg: Float64) -> None:
        self._speed_mps = float(msg.data)

    def _on_yaw_rate(self, msg: Float64) -> None:
        self._yaw_rate_rads = float(msg.data)

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_v = float(msg.voltage)

    def _on_mode(self, msg: String) -> None:
        self._frontseat_mode = msg.data or None

    def _on_status(self, msg: String) -> None:
        self._brain_status = msg.data or None

    # ------------------------------------------------------------------
    # Periodic record
    # ------------------------------------------------------------------

    def _log_tick(self) -> None:
        # No position yet means no useful record. Skipping beats writing rows
        # of nulls that have to be filtered out during analysis.
        if self._log_file is None or self._lat is None:
            return

        def rounded(value, digits):
            return round(value, digits) if value is not None else None

        record = {
            'ts': datetime.now(timezone.utc).isoformat(),
            'lat': rounded(self._lat, 8),
            'lon': rounded(self._lon, 8),
            'heading_deg': rounded(self._heading_deg, 2),
            'speed_mps': rounded(self._speed_mps, 3),
            'yaw_rate_rads': rounded(self._yaw_rate_rads, 4),
            'battery_v': rounded(self._battery_v, 2),
            'frontseat_mode': self._frontseat_mode,
            'brain_status': self._brain_status,
        }
        try:
            self._log_file.write(json.dumps(record) + '\n')
        except OSError as exc:
            self.get_logger().warn(f'Log write failed: {exc}')

    def destroy_node(self) -> None:
        self._close_log()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Telemetry logger interrupted — shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
