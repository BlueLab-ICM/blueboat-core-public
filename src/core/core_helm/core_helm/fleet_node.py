"""ROS node of the fleet link: publish this vehicle's state, republish what peers send."""

import math
import os
import time

import rclpy
from core_common.health import ModuleHealth
from core_interfaces.msg import PeerState
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from core_helm.fleet import PeerReport, UdpLink, decode, encode, parse_peers

DEFAULT_PORT = 14660
LOST_AFTER_S = 10.0


class FleetNode(Node):
    """Exchange states with the vehicles listed in FLEET_PEERS. No peers: stay idle."""

    def __init__(self) -> None:
        super().__init__('fleet_node')
        self.declare_parameter('rate_hz', 2.0)
        self.vehicle = os.environ.get('VEHICLE_NAME') or 'unnamed'
        port = int(os.environ.get('FLEET_PORT') or DEFAULT_PORT)
        peers = parse_peers(os.environ.get('FLEET_PEERS', ''), port)
        self.health = ModuleHealth(self, 'fleet')
        self.gps: NavSatFix | None = None
        self.heading_rad: float | None = None
        self.speed_mps = 0.0
        self.last_heard: dict[str, float] = {}
        self.link: UdpLink | None = None
        if not peers:
            self.health.ok('no peers configured')
            return

        self.link = UdpLink(port, peers)
        self.peer_pub = self.create_publisher(PeerState, '/fleet/peers', 10)
        self.create_subscription(NavSatFix, '/frontseat/gps', self._on_gps, 10)
        self.create_subscription(Float64, '/frontseat/heading', self._on_heading, 10)
        self.create_subscription(Float64, '/frontseat/speed', self._on_speed, 10)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        rate = max(0.1, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / rate, self._send, clock=steady)
        self.create_timer(0.1, self._receive, clock=steady)
        self.create_timer(1.0, self._check_peers, clock=steady)
        self.get_logger().info(f'Fleet link to {len(peers)} peer(s) on UDP {port}.')

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.gps = msg

    def _on_heading(self, msg: Float64) -> None:
        self.heading_rad = math.radians(float(msg.data))

    def _on_speed(self, msg: Float64) -> None:
        self.speed_mps = float(msg.data)

    def _send(self) -> None:
        if self.gps is None or self.heading_rad is None or self.link is None:
            return
        report = PeerReport(
            self.vehicle,
            self.gps.latitude,
            self.gps.longitude,
            self.heading_rad,
            self.speed_mps,
        )
        for error in self.link.send(encode(report)):
            self.get_logger().warning(error, throttle_duration_sec=30.0)

    def _receive(self) -> None:
        if self.link is None:
            return
        for data in self.link.receive():
            report = decode(data)
            if report is None or report.vehicle == self.vehicle:
                continue
            if report.vehicle not in self.last_heard:
                self.health.event('peer_heard', vehicle=report.vehicle)
            self.last_heard[report.vehicle] = time.monotonic()
            msg = PeerState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.vehicle = report.vehicle
            msg.latitude = report.lat
            msg.longitude = report.lon
            msg.heading_rad = report.heading_rad
            msg.speed_mps = report.speed_mps
            self.peer_pub.publish(msg)

    def _check_peers(self) -> None:
        now = time.monotonic()
        lost = [v for v, t in self.last_heard.items() if now - t > LOST_AFTER_S]
        for vehicle in lost:
            del self.last_heard[vehicle]
            self.health.event('peer_lost', vehicle=vehicle)
        self.health.ok(f'{len(self.last_heard)} peer(s) heard', peers=sorted(self.last_heard))


def main(args: list[str] | None = None) -> None:
    """Spin the fleet link until shutdown."""
    rclpy.init(args=args)
    node = FleetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.link is not None:
            node.link.close()
        node.destroy_node()
        rclpy.try_shutdown()
