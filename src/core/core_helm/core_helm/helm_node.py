"""ROS node of the helm: one command path from every source to the gateway.

Sources publish on /helm/setpoint. The executive engages the helm on /helm/engage and names
the source that may drive on /helm/grant. Only this node publishes /helm/output. It keeps
the gateway's lease alive, so if it dies the vehicle stops taking commands.
"""

import math
import time

import rclpy
from core_common.health import OK, WARN, ModuleHealth
from core_interfaces.msg import HelmGrant, HelmStatus, PeerState, Setpoint
from core_interfaces.srv import Engage
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from core_helm.collision import CollisionConfig, CollisionFilter, Peer
from core_helm.model import Fence, Limits, Output, Vehicle
from core_helm.rules import IDLE, Config, Helm, Request

SOURCE = 'core_helm'
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class HelmNode(Node):
    """Own the path to the gateway."""

    def __init__(self) -> None:
        super().__init__('helm_node')
        defaults = Config()
        limits = Limits()
        collision = CollisionConfig()
        self.declare_parameter('setpoint_timeout_s', defaults.setpoint_timeout_s)
        self.declare_parameter('grant_timeout_s', defaults.grant_timeout_s)
        self.declare_parameter('stale_stop_s', defaults.stale_stop_s)
        self.declare_parameter('gps_timeout_s', defaults.gps_timeout_s)
        self.declare_parameter('max_speed_mps', limits.max_speed_mps)
        self.declare_parameter('max_yaw_rate_rps', limits.max_yaw_rate_rps)
        self.declare_parameter('collision_enabled', True)
        self.declare_parameter('safety_radius_m', collision.safety_radius_m)
        self.declare_parameter('horizon_s', collision.horizon_s)
        self.declare_parameter('clear_s', collision.clear_s)
        self.declare_parameter('peer_timeout_s', collision.peer_timeout_s)
        self.declare_parameter('cruise_speed_mps', collision.cruise_speed_mps)

        def number(name: str) -> float:
            return float(self.get_parameter(name).value)

        avoidance = None
        if bool(self.get_parameter('collision_enabled').value):
            avoidance = CollisionFilter(
                CollisionConfig(
                    safety_radius_m=number('safety_radius_m'),
                    horizon_s=number('horizon_s'),
                    clear_s=number('clear_s'),
                    peer_timeout_s=number('peer_timeout_s'),
                    cruise_speed_mps=number('cruise_speed_mps'),
                )
            )
        self.helm = Helm(
            Config(
                setpoint_timeout_s=number('setpoint_timeout_s'),
                grant_timeout_s=number('grant_timeout_s'),
                stale_stop_s=number('stale_stop_s'),
                gps_timeout_s=number('gps_timeout_s'),
                cruise_speed_mps=number('cruise_speed_mps'),
            ),
            Limits(number('max_speed_mps'), number('max_yaw_rate_rps')),
            avoidance,
        )
        self.gps: NavSatFix | None = None
        self.gps_time = -math.inf
        self.heading_rad: float | None = None
        self.speed_mps = 0.0
        self.mode = ''
        self.peers: dict[str, tuple[PeerState, float]] = {}
        self._reported: tuple[str, str, str, str] = ('', '', '', '')

        self.output_pub = self.create_publisher(Setpoint, '/helm/output', 10)
        self.status_pub = self.create_publisher(HelmStatus, '/helm/status', LATCHED)
        self.health = ModuleHealth(self, 'helm')
        self.create_subscription(Setpoint, '/helm/setpoint', self._on_setpoint, 10)
        self.create_subscription(HelmGrant, '/helm/grant', self._on_grant, 10)
        self.create_subscription(NavSatFix, '/frontseat/gps', self._on_gps, 10)
        self.create_subscription(Float64, '/frontseat/heading', self._on_heading, 10)
        self.create_subscription(Float64, '/frontseat/speed', self._on_speed, 10)
        self.create_subscription(String, '/frontseat/mode', self._on_mode, 10)
        self.create_subscription(PeerState, '/fleet/peers', self._on_peer, 10)
        self.create_service(Engage, '/helm/engage', self._on_engage)

        # Steady clock: a wall-clock step would stall timers on the ROS clock.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.1, self._tick, clock=steady)
        self.create_timer(1.0, self._publish_status, clock=steady)
        self.get_logger().info('Helm online.')

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self.gps = msg
        self.gps_time = time.monotonic()

    def _on_heading(self, msg: Float64) -> None:
        self.heading_rad = math.radians(float(msg.data))

    def _on_speed(self, msg: Float64) -> None:
        self.speed_mps = float(msg.data)

    def _on_mode(self, msg: String) -> None:
        self.mode = msg.data

    def _on_peer(self, msg: PeerState) -> None:
        self.peers[msg.vehicle] = (msg, time.monotonic())

    def _on_grant(self, msg: HelmGrant) -> None:
        self.helm.on_grant(msg.source, time.monotonic())

    def _on_setpoint(self, msg: Setpoint) -> None:
        self.helm.on_setpoint(
            Request(
                msg.source,
                msg.mode,
                msg.latitude,
                msg.longitude,
                msg.speed_mps,
                msg.yaw_rate_rps,
                time.monotonic(),
            )
        )

    def _vehicle(self) -> Vehicle:
        gps = self.gps
        return Vehicle(
            gps.latitude if gps else None,
            gps.longitude if gps else None,
            self.heading_rad,
            self.speed_mps,
            self.mode == 'GUIDED',
            time.monotonic() - self.gps_time,
        )

    def _on_engage(self, request: Engage.Request, response: Engage.Response) -> Engage.Response:
        if not request.engage:
            self.helm.disengage(f'stopped by {request.requester or "request"}')
            response.success = True
            response.message = 'disengaged'
            self._publish_status()
            return response
        limits = None
        if request.max_speed_mps > 0.0 or request.max_yaw_rate_rps > 0.0:
            base = self.helm.default_limits
            limits = Limits(
                request.max_speed_mps or base.max_speed_mps,
                request.max_yaw_rate_rps or base.max_yaw_rate_rps,
            )
        fence = Fence(
            request.fence_latitude,
            request.fence_longitude,
            request.fence_radius_m,
            request.fence_abort_margin_m,
        )
        response.success, response.message = self.helm.engage(
            fence, limits, self._vehicle(), time.monotonic()
        )
        if not response.success:
            self.get_logger().warning(f'Engage refused: {response.message}')
        self._publish_status()
        return response

    def _peer_list(self) -> list[Peer]:
        now = time.monotonic()
        return [
            Peer(m.vehicle, m.latitude, m.longitude, m.heading_rad, m.speed_mps, now - t)
            for m, t in self.peers.values()
        ]

    def _tick(self) -> None:
        output = self.helm.tick(time.monotonic(), self._vehicle(), self._peer_list())
        self._publish_output(output)
        if self._status_key() != self._reported:
            self._publish_status()

    def _publish_output(self, output: Output) -> None:
        msg = Setpoint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = SOURCE
        msg.mode = output.mode
        msg.latitude = output.lat
        msg.longitude = output.lon
        msg.speed_mps = output.speed_mps
        msg.yaw_rate_rps = output.yaw_rate_rps
        self.output_pub.publish(msg)

    def _status_key(self) -> tuple[str, str, str, str]:
        helm = self.helm
        return helm.state, helm.grant, helm.override, helm.reason

    def _publish_status(self) -> None:
        helm = self.helm
        key = self._status_key()
        if key != self._reported:
            level = WARN if helm.override else OK
            self.health.event(
                'helm', level=level, state=key[0], source=key[1], override=key[2], reason=key[3]
            )
            self._reported = key
        msg = HelmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = helm.state
        msg.granted_source = helm.grant
        msg.active_source = helm.active_source
        msg.override = helm.override
        msg.reason = helm.reason
        msg.guided = self.mode == 'GUIDED'
        if helm.fence is not None:
            msg.fence_latitude = helm.fence.lat
            msg.fence_longitude = helm.fence.lon
            msg.fence_radius_m = helm.fence.radius_m
        self.status_pub.publish(msg)
        if helm.state == IDLE:
            self.health.ok(f'idle ({helm.reason})' if helm.reason else 'idle')
        elif helm.override:
            self.health.warn(f'{helm.state}: {helm.override}')
        else:
            self.health.ok(f'{helm.state}: {helm.active_source}')


def main(args: list[str] | None = None) -> None:
    """Spin the helm until shutdown."""
    rclpy.init(args=args)
    node = HelmNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
