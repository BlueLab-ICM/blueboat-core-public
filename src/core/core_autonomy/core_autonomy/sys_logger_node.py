"""
System Logger — backseat computer health on the standard ROS 2 diagnostics topic.

Publishes CPU, memory, disk and temperature for the backseat computer at 1 Hz
on ``/diagnostics`` (``diagnostic_msgs/DiagnosticArray``), the conventional
topic that generic ROS tooling — ``rqt_runtime_monitor``, Foxglove's diagnostics
panel — already knows how to display.

Board temperature is additionally republished on ``/backseat/temperature`` as a
plain ``sensor_msgs/Temperature``, which plots directly alongside the water and
autopilot temperatures the gateway publishes.  Thermal throttling is a real
constraint on a sealed hull in the sun: an Orin Nano running vision inference
inside a watertight enclosure is the first thing to suffer, and this is the
signal that shows it.
"""

import platform

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import Temperature

# Above this percentage on CPU, memory or disk the status is raised to WARN so
# it stands out in the diagnostics panel and in the recorded bag.
_UTILIZATION_WARN_PCT = 90.0


class SysLoggerNode(Node):
    def __init__(self):
        super().__init__('sys_logger')

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.temperature_pub = self.create_publisher(
            Temperature, '/backseat/temperature', 10)

        # Reported as the diagnostics hardware_id so a recorded bag identifies
        # which backseat computer produced it (Raspberry Pi 5 vs Jetson Orin).
        self._hardware_id = platform.node() or 'backseat'

        self.create_timer(1.0, self._timer_callback)
        self.get_logger().info(f'System logger online (hardware_id={self._hardware_id}).')

    def _read_temperatures(self, stamp) -> list[str]:
        """Return per-sensor temperature strings, publishing the first as the
        board temperature.

        ``psutil.sensors_temperatures()`` returns a platform-dependent mapping
        with no portable notion of "the CPU sensor", so the first reading is
        taken as representative and the full set is kept as a diagnostics
        key/value for anyone who needs a specific zone.
        """
        readings: list[str] = []
        try:
            temps = psutil.sensors_temperatures()
        except (AttributeError, OSError):
            return readings
        if not temps:
            return readings

        for name, entries in temps.items():
            for entry in entries:
                readings.append(f'{name}: {entry.current:.1f}C')

        first_zone = next(iter(temps.values()))
        if first_zone:
            msg = Temperature()
            msg.header.stamp = stamp
            msg.temperature = float(first_zone[0].current)
            self.temperature_pub.publish(msg)
        return readings

    def _timer_callback(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # interval=None compares against the previous call rather than blocking
        # for a sampling window, which keeps this callback non-blocking.
        cpu_percent = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        disk = psutil.disk_usage('/')
        temperatures = self._read_temperatures(msg.header.stamp)

        status = DiagnosticStatus()
        status.name = 'Backseat System Health'
        status.hardware_id = self._hardware_id
        status.values = [
            KeyValue(key='CPU Usage (%)', value=f'{cpu_percent:.1f}'),
            KeyValue(key='Memory Usage (%)', value=f'{mem.percent:.1f}'),
            KeyValue(key='Memory Total (GB)', value=f'{mem.total / (1024 ** 3):.2f}'),
            KeyValue(key='Disk Usage (%)', value=f'{disk.percent:.1f}'),
        ]
        if temperatures:
            status.values.append(
                KeyValue(key='Temperatures', value=', '.join(temperatures)))

        if max(cpu_percent, mem.percent, disk.percent) > _UTILIZATION_WARN_PCT:
            status.level = DiagnosticStatus.WARN
            status.message = 'High resource utilization on the backseat computer.'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'System operating normally'

        msg.status.append(status)
        self.diagnostics_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SysLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
