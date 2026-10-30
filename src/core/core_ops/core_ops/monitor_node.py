"""Backseat health on /diagnostics, temperature, and container stats."""

from __future__ import annotations

import os
import queue
import subprocess
import threading

import psutil
import rclpy
from core_common.health import ModuleHealth
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String

from core_ops.thermal import pick_cpu_zone, read_thermal_zones

WARN_PERCENT = 90.0
ERROR_DISK_PERCENT = 97.0
STATS_PERIOD_S = 5.0


class MonitorNode(Node):
    """Publish host health so a degraded computer is visible before it fails."""

    def __init__(self) -> None:
        super().__init__('monitor_node')
        self.health = ModuleHealth(self, 'backseat')
        self.temp_pub = self.create_publisher(Temperature, '/backseat/temperature', 10)
        self.stats_pub = self.create_publisher(String, '/ops/container_stats', 10)
        vehicle = os.environ.get('VEHICLE_NAME') or 'unknown'
        platform = os.environ.get('BACKSEAT_PLATFORM') or 'unknown'
        self.hardware_id = f'{vehicle}_{platform}_backseat'
        self._stats: queue.SimpleQueue[str] = queue.SimpleQueue()
        self.create_timer(1.0, self._sample)
        self.create_timer(STATS_PERIOD_S, self._publish_stats)

    def _sample(self) -> None:
        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        disk = psutil.disk_usage('/').percent
        zones = read_thermal_zones()
        cpu_zone = pick_cpu_zone(zones)
        if cpu_zone is not None:
            msg = Temperature()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.temperature = cpu_zone.celsius
            self.temp_pub.publish(msg)
        values = {
            'hardware_id': self.hardware_id,
            'cpu_percent': f'{cpu:.1f}',
            'memory_percent': f'{mem:.1f}',
            'disk_percent': f'{disk:.1f}',
            'temperatures': ', '.join(f'{z.name}: {z.celsius:.1f}C' for z in zones) or 'n/a',
        }
        if disk >= ERROR_DISK_PERCENT:
            self.health.error(f'disk {disk:.0f}% full', **values)
        elif max(cpu, mem, disk) >= WARN_PERCENT:
            self.health.warn('high utilization', **values)
        else:
            self.health.ok('running', **values)

    def _publish_stats(self) -> None:
        try:
            raw = self._stats.get_nowait()
        except queue.Empty:
            raw = None
        if raw is not None:
            msg = String()
            msg.data = raw
            self.stats_pub.publish(msg)
        threading.Thread(target=self._collect_stats, daemon=True).start()

    def _collect_stats(self) -> None:
        try:
            raw = subprocess.check_output(
                [
                    'docker',
                    'stats',
                    '--no-stream',
                    '--format',
                    '{{.Name}},{{.CPUPerc}},{{.MemUsage}}',
                ],
                timeout=4.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warning(f'No container stats: {exc}', throttle_duration_sec=60.0)
            return
        self._stats.put(raw.decode('utf-8').strip())


def main(args: list[str] | None = None) -> None:
    """Spin the monitor until shutdown."""
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
