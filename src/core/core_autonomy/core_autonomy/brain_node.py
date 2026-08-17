"""
Autonomy Brain — waypoint missions and a safety-gated command relay.

The brain decides where the boat goes. It does one of three things, selected by
the ``control_source`` parameter:

  * ``mission``          fly a waypoint list from a JSON file, then stop
  * ``payload_waypoint`` relay position targets published by a payload
  * ``payload_velocity`` relay velocity targets published by a payload

The first covers ordinary survey work with no code to write. The other two are
the extension point: your algorithm lives in a payload, in its own repository
and its own container, and publishes where it wants the boat to go. Either way
the brain applies the protections that every source needs and none should have
to reimplement:

  * **Explicit enable.** Relaying is off until an operator turns it on. GUIDED
    alone is not enough — an operator may select GUIDED to drive the boat
    themselves from a ground station.
  * **Geofence.** Commands that would take the boat outside a circular
    boundary are refused, and relaying stops entirely if the boat leaves it.
  * **Staleness.** A payload that stops publishing stops the boat, rather than
    leaving its last command latched in the autopilot.
  * **Lease keepalive.** A heartbeat at 10 Hz. If this node dies, the gateway
    revokes the autonomy lease within two seconds.

Commands never reach the vehicle directly from here. Everything is published to
the gateway, which enforces its own checks — sender identity, the GUIDED flight
mode, and the heartbeat — before any of it becomes MAVLink.

Extending this is expected, and the intended way is a payload rather than a
change here: anything publishing to ``/payload/waypoints/waypoint`` inherits
the whole safety envelope for free.
"""

import json
import math
import queue
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from core_interfaces.msg import Command

from geometry_msgs.msg import Point, TransformStamped, Twist
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import ColorRGBA, Float64, String
from std_srvs.srv import SetBool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Pure geometry, kept ROS-free in its own module so it can be tested and
# reused without a ROS 2 environment.
from core_autonomy.geodesy import gps_to_local_ned, haversine_m  # noqa: E402

# Waypoint files baked into the brain image at build time, so a mission stays
# runnable when the /workspace bind mount is absent.
_BUNDLED_WAYPOINT_DIR = Path('/opt/blueboat/waypoints')


class BrainNode(Node):
    """Keeps the autonomy lease alive and relays payload commands, safely."""

    _VIZ_ROOT_FRAME_ID = 'map'
    _VIZ_FRAME_ID = 'geofence_ned'
    _VIZ_BOAT_ARROW_LEN_M = 4.0

    def __init__(self):
        super().__init__('brain_node')
        self.get_logger().info('Initializing autonomy brain...')

        # ==================================================================
        # Parameters
        # ==================================================================

        # Geofence centre. Left unset, the first GPS fix after startup becomes
        # the centre — so the boat is always fenced relative to where it
        # actually launched, with no configuration at all. Set these explicitly
        # when the survey area is known in advance.
        self.declare_parameter('geofence_home_lat', float('nan'))
        self.declare_parameter('geofence_home_lon', float('nan'))
        self.declare_parameter('geofence_radius_m', 100.0)

        # A payload command older than this is treated as absent. Set it to a
        # few times the payload's publishing period: long enough to ride out a
        # slow inference frame, short enough that a crashed payload stops the
        # boat promptly.
        self.declare_parameter('payload_command_timeout_sec', 2.0)

        # What drives the boat. Exactly one source at a time — forwarding a
        # position target and a velocity target together would hand the
        # autopilot two conflicting instructions.
        #
        #   'mission'          follow waypoint_file, then stop
        #   'payload_waypoint' relay /payload/waypoints/waypoint (NavSatFix)
        #   'payload_velocity' relay /payload/cmd_vel            (Twist)
        self.declare_parameter('control_source', 'mission')

        # Waypoint mission. The file is JSON; see deployment/waypoints/ for
        # examples and scripts/generate-waypoints.py to make your own.
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('waypoint_reached_tolerance_m', 4.0)
        # Repeat the pattern instead of stopping at the last point. Useful for
        # a monitoring station that should keep covering the same area.
        self.declare_parameter('mission_loop', False)

        # ==================================================================
        # State
        # ==================================================================
        self.autonomy_enabled = False
        self.current_gps: NavSatFix | None = None
        self.current_heading_rad: float | None = None
        self._frontseat_mode: str | None = None

        self._geofence_home: tuple[float, float] | None = None
        self._out_of_bounds_logged = False
        self._rejected_target_logged = False
        self._last_status: str | None = None

        self.payload_waypoint: NavSatFix | None = None
        self.payload_waypoint_time: float | None = None
        self.payload_twist: Twist | None = None
        self.payload_twist_time: float | None = None

        self.mission_waypoints: list[tuple[float, float]] = []
        self.mission_index = 0

        self._viz_markers_active = False

        # ==================================================================
        # Publishers
        # ==================================================================
        # Commands to the gateway. Nothing else in the stack may publish here:
        # the gateway rejects any Command whose source_node is not
        # 'core_autonomy'.
        self.cmd_pub = self.create_publisher(Command, '/system/brain/command', 10)
        self.vel_pub = self.create_publisher(Twist, '/system/brain/cmd_vel', 10)

        # Monitoring surface, consumed by the telemetry logger, Foxglove and
        # any remote dashboard.
        self.status_pub = self.create_publisher(String, '/system/brain/status', 10)
        self.stats_pub = self.create_publisher(String, '/system/container_stats', 10)
        self.viz_pub = self.create_publisher(
            MarkerArray, '/system/brain/visualization', 1)
        self.viz_tf_broadcaster = StaticTransformBroadcaster(self)

        # ==================================================================
        # Services
        # ==================================================================
        # std_srvs/SetBool rather than a custom type, so it can be called from
        # any ROS 2 installation without building this repository's interfaces.
        self._enable_srv = self.create_service(
            SetBool, '/system/enable_autonomy', self._on_enable_autonomy)

        # ==================================================================
        # Subscribers
        # ==================================================================
        # Vehicle telemetry, republished from MAVLink by the gateway.
        self.create_subscription(NavSatFix, '/frontseat/gps', self._on_gps, 10)
        self.create_subscription(Float64, '/frontseat/heading', self._on_heading, 10)
        self.create_subscription(String, '/frontseat/mode', self._on_frontseat_mode, 10)

        # Payload inputs. Both optional: a stack with no payloads simply never
        # receives a message here.
        self.create_subscription(
            NavSatFix, '/payload/waypoints/waypoint', self._on_payload_waypoint, 10)
        self.create_subscription(
            Twist, '/payload/cmd_vel', self._on_payload_twist, 10)

        # ==================================================================
        # Timers
        # ==================================================================
        # 10 Hz. This rate is set by the gateway's two-second lease timeout: it
        # must be fast enough that a few dropped ticks cannot revoke the lease.
        self.create_timer(0.1, self._control_loop)
        self.create_timer(0.5, self._publish_visualization)

        # `docker stats` takes seconds to return, so collection runs on a
        # worker thread and the result is handed back through this queue to be
        # published from the executor thread. rclpy publishers are not
        # thread-safe, and a stalled executor would drop the heartbeat the
        # gateway watchdog depends on — monitoring must never break control.
        self._stats_queue: queue.SimpleQueue = queue.SimpleQueue()
        self.create_timer(5.0, self._publish_container_stats)

        self._publish_static_viz_frame()
        self.get_logger().info(
            'Autonomy brain online. Relaying is DISABLED — enable it with: '
            'ros2 service call /system/enable_autonomy std_srvs/srv/SetBool '
            '"{data: true}"')

    # ======================================================================
    # Enable / disable
    # ======================================================================

    def _on_enable_autonomy(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """Turn command relaying on or off.

        Enabling is refused unless the operator has already placed ArduPilot in
        GUIDED. Requiring both is deliberate: GUIDED alone may mean the
        operator intends to drive the boat themselves from a ground station,
        and a payload publishing waypoints would otherwise take over silently.
        """
        if not request.data:
            self._set_autonomy(False, 'operator request')
            response.success = True
            response.message = 'Autonomy relaying disabled.'
            return response

        if self._frontseat_mode != 'GUIDED':
            response.success = False
            response.message = (
                f'Refused: ArduPilot must be in GUIDED (current: '
                f'{self._frontseat_mode!r}). Select GUIDED on the RC '
                'transmitter or in the ground station first.')
            self.get_logger().error(response.message)
            return response

        # Anchor the geofence now if it has not been configured or captured.
        if self._geofence_home is None:
            self._capture_geofence_home()

        if self._geofence_home is None:
            response.success = False
            response.message = (
                'Refused: no geofence centre. Waiting for a valid GPS fix, or '
                'set geofence_home_lat and geofence_home_lon explicitly.')
            self.get_logger().error(response.message)
            return response

        # A waypoint mission is loaded at enable time, not at startup, so that
        # editing the file and re-enabling is the whole edit cycle.
        if self._control_source() == 'mission':
            self.mission_waypoints = self._load_waypoints()
            self.mission_index = 0
            if not self.mission_waypoints:
                response.success = False
                response.message = (
                    'Refused: control_source is "mission" but no waypoints '
                    f'loaded (waypoint_file='
                    f'{str(self.get_parameter("waypoint_file").value)!r}). '
                    'Relative paths resolve under /workspace.')
                self.get_logger().error(response.message)
                return response

            outside = [
                index for index, (lat, lon) in enumerate(self.mission_waypoints, start=1)
                if not self._target_is_in_bounds(lat, lon)]
            if outside:
                # Refuse rather than start and abort part-way: a pattern that
                # does not fit its fence is a configuration mistake, and
                # finding out halfway through a survey wastes the run.
                response.success = False
                response.message = (
                    f'Refused: {len(outside)} of {len(self.mission_waypoints)} '
                    f'waypoints lie outside the {float(self.get_parameter("geofence_radius_m").value):.0f} m '
                    'geofence (first is number '
                    f'{outside[0]}). Widen geofence_radius_m or move the pattern.')
                self.get_logger().error(response.message)
                return response

        self._out_of_bounds_logged = False
        self._rejected_target_logged = False
        self._set_autonomy(True, 'operator request')
        response.success = True
        response.message = 'Autonomy relaying enabled.'
        return response

    def _control_source(self) -> str:
        source = str(self.get_parameter('control_source').value).strip().lower()
        if source not in ('mission', 'payload_waypoint', 'payload_velocity'):
            self.get_logger().warn(
                f'Unknown control_source {source!r}; falling back to "mission".')
            return 'mission'
        return source

    def _load_waypoints(self) -> list[tuple[float, float]]:
        """Load the waypoint file.

        Accepts either ``{"waypoints": [...]}`` or a bare JSON list. Each item
        needs ``lat``/``latitude`` and ``lon``/``longitude``; anything else in
        the object — indices, local coordinates, generator metadata — is
        ignored, so files produced by external survey planners usually load
        unmodified.
        """
        file_path = str(self.get_parameter('waypoint_file').value).strip()
        if not file_path:
            return []

        given = Path(file_path)
        if given.is_absolute():
            candidates = [given]
        else:
            # The bind-mounted repository first, so an edited file wins over
            # the copy baked into the image at build time.
            candidates = [Path('/workspace') / given,
                          _BUNDLED_WAYPOINT_DIR / given.name]

        try:
            path = next((p for p in candidates if p.is_file()), None)
            if path is None:
                tried = ', '.join(str(p) for p in candidates)
                raise FileNotFoundError(f'no file found (tried: {tried})')
            with path.open('r', encoding='utf-8') as stream:
                data = json.load(stream)
            items = data.get('waypoints', []) if isinstance(data, dict) else data
            waypoints = []
            for item in items:
                lat = item.get('lat', item.get('latitude'))
                lon = item.get('lon', item.get('longitude'))
                if lat is None or lon is None:
                    continue
                waypoints.append((float(lat), float(lon)))
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {path}.')
            return waypoints
        except Exception as exc:  # noqa: BLE001 — a bad file must not kill the node
            self.get_logger().error(f'Failed to load waypoint file: {exc}')
            return []

    def _set_autonomy(self, enabled: bool, reason: str) -> None:
        if self.autonomy_enabled != enabled:
            state = 'ENABLED' if enabled else 'DISABLED'
            self.get_logger().info(f'Autonomy relaying {state} ({reason}).')
        self.autonomy_enabled = enabled

    def _capture_geofence_home(self) -> None:
        """Set the geofence centre from parameters, or from the current fix."""
        lat = float(self.get_parameter('geofence_home_lat').value)
        lon = float(self.get_parameter('geofence_home_lon').value)

        if not (math.isnan(lat) or math.isnan(lon)):
            self._geofence_home = (lat, lon)
            self.get_logger().info(
                f'Geofence centred on the configured position '
                f'({lat:.6f}, {lon:.6f}), radius '
                f'{float(self.get_parameter("geofence_radius_m").value):.0f} m.')
            return

        if not self._gps_is_valid():
            return

        self._geofence_home = (self.current_gps.latitude, self.current_gps.longitude)
        self.get_logger().info(
            f'Geofence centred on the first GPS fix '
            f'({self._geofence_home[0]:.6f}, {self._geofence_home[1]:.6f}), '
            f'radius {float(self.get_parameter("geofence_radius_m").value):.0f} m.')

    # ======================================================================
    # Subscriber callbacks
    # ======================================================================

    def _gps_is_valid(self) -> bool:
        """True when the current fix is a real position rather than a placeholder.

        ArduPilot publishes GLOBAL_POSITION_INT before the EKF has converged,
        with latitude and longitude at zero. Anchoring a geofence on that would
        put the boundary in the Gulf of Guinea, so both the fix status stamped
        by the gateway and the null-island sentinel are checked — the sentinel
        because a frontseat that never sends GPS_RAW_INT would otherwise leave
        the status permanently unset.
        """
        if self.current_gps is None:
            return False
        if self.current_gps.status.status == NavSatStatus.STATUS_NO_FIX:
            return False
        return not (abs(self.current_gps.latitude) < 1e-7
                    and abs(self.current_gps.longitude) < 1e-7)

    def _on_gps(self, msg: NavSatFix) -> None:
        self.current_gps = msg
        if self._geofence_home is None:
            self._capture_geofence_home()

    def _on_heading(self, msg: Float64) -> None:
        self.current_heading_rad = math.radians(float(msg.data))

    def _on_frontseat_mode(self, msg: String) -> None:
        previous = self._frontseat_mode
        self._frontseat_mode = msg.data
        # Leaving GUIDED is the operator taking back control. Drop the enable
        # flag so that returning to GUIDED later does not silently resume
        # relaying a payload the operator has stopped thinking about.
        if previous == 'GUIDED' and msg.data != 'GUIDED' and self.autonomy_enabled:
            self._set_autonomy(False, 'operator left GUIDED')

    def _on_payload_waypoint(self, msg: NavSatFix) -> None:
        self.payload_waypoint = msg
        self.payload_waypoint_time = time.monotonic()

    def _on_payload_twist(self, msg: Twist) -> None:
        self.payload_twist = msg
        self.payload_twist_time = time.monotonic()

    # ======================================================================
    # Safety checks
    # ======================================================================

    def _distance_from_home_m(self, lat: float, lon: float) -> float | None:
        if self._geofence_home is None:
            return None
        return haversine_m(lat, lon, self._geofence_home[0], self._geofence_home[1])

    def _boat_is_out_of_bounds(self) -> bool:
        # An invalid fix is not evidence of leaving the fence — treating it as
        # such would stop the boat every time GPS briefly dropped out.
        if not self._gps_is_valid():
            return False
        distance = self._distance_from_home_m(
            self.current_gps.latitude, self.current_gps.longitude)
        if distance is None:
            return False
        return distance > max(0.0, float(self.get_parameter('geofence_radius_m').value))

    def _target_is_in_bounds(self, lat: float, lon: float) -> bool:
        distance = self._distance_from_home_m(lat, lon)
        if distance is None:
            return False
        return distance <= max(0.0, float(self.get_parameter('geofence_radius_m').value))

    def _command_is_fresh(self, stamp: float | None) -> bool:
        if stamp is None:
            return False
        timeout = max(0.1, float(self.get_parameter('payload_command_timeout_sec').value))
        return (time.monotonic() - stamp) <= timeout

    # ======================================================================
    # Command publishing
    # ======================================================================

    def _publish_heartbeat(self) -> None:
        """Publish the keepalive the gateway's watchdog counts on.

        ``CMD_MODE`` carries no authority — ArduPilot's flight mode belongs to
        the operator. The message exists so the gateway can tell a live brain
        from a dead one.
        """
        cmd = Command()
        cmd.source_node = 'core_autonomy'
        cmd.command_type = Command.CMD_MODE
        cmd.string_param = 'ENABLED' if self.autonomy_enabled else 'DISABLED'
        self.cmd_pub.publish(cmd)

    def _publish_waypoint(self, lat: float, lon: float) -> None:
        cmd = Command()
        cmd.source_node = 'core_autonomy'
        cmd.command_type = Command.CMD_WAYPOINT
        cmd.float_params = [float(lat), float(lon)]
        self.cmd_pub.publish(cmd)

    # ======================================================================
    # Control loop
    # ======================================================================

    def _control_loop(self) -> None:
        """10 Hz: heartbeat, safety checks, then relay one command."""
        self._publish_heartbeat()

        if not self.autonomy_enabled:
            # Publish a zero Twist so a velocity latched in the autopilot from
            # a previous run is actively flushed. Simply not publishing would
            # leave the boat driving on its last command.
            self.vel_pub.publish(Twist())
            self._publish_status('disabled')
            return

        # The boat itself has left the fence. Stop relaying entirely: the
        # payload that drove it there is the last thing that should be trusted
        # to drive it back.
        if self._boat_is_out_of_bounds():
            if not self._out_of_bounds_logged:
                self.get_logger().warn(
                    'Boat is outside the geofence. Relaying disabled — take '
                    'manual control via RC transmitter or ground station.')
                self._out_of_bounds_logged = True
            self._set_autonomy(False, 'geofence')
            self.vel_pub.publish(Twist())
            self._publish_status('geofence_stop')
            return

        source = self._control_source()
        if source == 'mission':
            self._follow_mission()
        elif source == 'payload_velocity':
            self._relay_velocity()
        else:
            self._relay_waypoint()

    def _follow_mission(self) -> None:
        """Fly the loaded waypoint list, advancing on arrival.

        The current waypoint is re-published every tick rather than sent once.
        A dropped message therefore costs nothing — the autopilot is still
        driving to the last target it received, and the next tick repeats it.
        """
        if self.mission_index >= len(self.mission_waypoints):
            if bool(self.get_parameter('mission_loop').value):
                self.mission_index = 0
                self.get_logger().info('Mission complete — looping.')
            else:
                self.get_logger().info(
                    f'Mission complete: {len(self.mission_waypoints)} waypoints '
                    'reached. Relaying disabled.')
                self._set_autonomy(False, 'mission complete')
                self._publish_status('mission_complete')
                return

        lat, lon = self.mission_waypoints[self.mission_index]
        self._publish_waypoint(lat, lon)
        self._publish_status('mission')

        if not self._gps_is_valid():
            return
        tolerance = float(self.get_parameter('waypoint_reached_tolerance_m').value)
        distance = haversine_m(
            self.current_gps.latitude, self.current_gps.longitude, lat, lon)
        # The 0.5 m floor stops a mis-set tolerance of zero from stalling the
        # mission forever on GPS noise alone.
        if distance <= max(0.5, tolerance):
            self.mission_index += 1
            self.get_logger().info(
                f'Waypoint {self.mission_index}/{len(self.mission_waypoints)} reached.')

    def _relay_waypoint(self) -> None:
        """Forward the payload's position target, if it is fresh and in bounds."""
        if not self._command_is_fresh(self.payload_waypoint_time):
            # Nothing to forward. The autopilot holds its last position target,
            # which for a waypoint means it stops at a known place rather than
            # continuing indefinitely.
            self._publish_status('waiting_for_payload')
            return

        lat = self.payload_waypoint.latitude
        lon = self.payload_waypoint.longitude

        if not self._target_is_in_bounds(lat, lon):
            # Refuse the individual command rather than shutting down: a
            # payload that briefly proposes a bad target should be corrected,
            # not treated as a vehicle emergency.
            if not self._rejected_target_logged:
                distance = self._distance_from_home_m(lat, lon) or 0.0
                self.get_logger().warn(
                    f'Refusing payload target {distance:.0f} m from the '
                    'geofence centre — outside the boundary. Holding.')
                self._rejected_target_logged = True
            self._publish_status('target_rejected')
            return

        self._rejected_target_logged = False
        self._publish_waypoint(lat, lon)
        self._publish_status('relaying_waypoint')

    def _relay_velocity(self) -> None:
        """Forward the payload's velocity target, if it is fresh."""
        if not self._command_is_fresh(self.payload_twist_time):
            # A velocity is latched by the autopilot until superseded, so
            # silence is not a stop. Publish zero to actually stop the boat.
            self.vel_pub.publish(Twist())
            self._publish_status('waiting_for_payload')
            return

        self.vel_pub.publish(self.payload_twist)
        self._publish_status('relaying_velocity')

    # ======================================================================
    # Monitoring
    # ======================================================================

    def _publish_status(self, status: str) -> None:
        """Publish the relay status, logging only when it changes.

        Edge-triggered because this is called ten times a second; logging
        unconditionally would bury every other message in the container log.
        """
        if status != self._last_status:
            detail = [f'frontseat_mode={self._frontseat_mode!r}']
            if self.current_gps is not None and self._geofence_home is not None:
                distance = self._distance_from_home_m(
                    self.current_gps.latitude, self.current_gps.longitude)
                radius = float(self.get_parameter('geofence_radius_m').value)
                detail.append(f'dist_from_home_m={distance:.1f}/{radius:.0f}')
            self.get_logger().info(
                f'Status: {self._last_status!r} -> {status!r} [{" ".join(detail)}]')
            self._last_status = status

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _publish_container_stats(self) -> None:
        """Publish the Docker stats snapshot gathered by the worker thread."""
        try:
            msg = String()
            msg.data = self._stats_queue.get_nowait()
            self.stats_pub.publish(msg)
        except queue.Empty:
            pass
        threading.Thread(target=self._collect_stats_async, daemon=True).start()

    def _collect_stats_async(self) -> None:
        try:
            raw = subprocess.check_output(
                ['docker', 'stats', '--no-stream',
                 '--format', '{{.Name}},{{.CPUPerc}},{{.MemUsage}}'],
                timeout=4.0,
            ).decode('utf-8').strip()
            self._stats_queue.put(raw)
        except Exception as exc:  # noqa: BLE001 — monitoring must never kill the node
            self.get_logger().warn(f'Failed to collect Docker stats: {exc}')

    # ======================================================================
    # Visualisation
    # ======================================================================

    def _publish_static_viz_frame(self) -> None:
        """Pin the local frame under 'map' so markers have a parent."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._VIZ_ROOT_FRAME_ID
        tf.child_frame_id = self._VIZ_FRAME_ID
        tf.transform.rotation.w = 1.0
        self.viz_tf_broadcaster.sendTransform(tf)

    def _publish_visualization(self) -> None:
        """Publish the geofence and the boat pose as markers.

        Rendered in Foxglove or RViz. Drawn in local metres centred on the
        geofence, which keeps the numbers readable compared to raw lat/lon.
        """
        if self._geofence_home is None:
            return

        home_lat, home_lon = self._geofence_home
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        def new_marker(marker_id: int, marker_type: int, scale: float) -> Marker:
            marker = Marker()
            marker.header.frame_id = self._VIZ_FRAME_ID
            marker.header.stamp = stamp
            marker.ns = 'core'
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.pose.orientation.w = 1.0
            return marker

        # Geofence, sampled as a closed line strip.
        radius = float(self.get_parameter('geofence_radius_m').value)
        fence = new_marker(0, Marker.LINE_STRIP, 0.5)
        fence.color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.9)
        for i in range(65):
            theta = 2.0 * math.pi * i / 64.0
            fence.points.append(
                Point(x=radius * math.sin(theta), y=radius * math.cos(theta), z=0.0))
        array.markers.append(fence)

        # The boat, as an arrow along its current heading. Green while
        # relaying, grey while idle — so the state is visible at a glance.
        if self.current_gps is not None:
            north, east = gps_to_local_ned(
                self.current_gps.latitude, self.current_gps.longitude,
                home_lat, home_lon)
            boat = new_marker(1, Marker.ARROW, 0.8)
            boat.scale.x = 0.8   # shaft diameter
            boat.scale.y = 1.6   # head diameter
            boat.scale.z = 0.0
            boat.color = (ColorRGBA(r=0.1, g=0.9, b=0.3, a=1.0)
                          if self.autonomy_enabled
                          else ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0))
            heading = self.current_heading_rad or 0.0
            tip = self._VIZ_BOAT_ARROW_LEN_M
            boat.points.append(Point(x=east, y=north, z=0.0))
            boat.points.append(Point(
                x=east + tip * math.sin(heading),
                y=north + tip * math.cos(heading),
                z=0.0))
            array.markers.append(boat)

        # Mission waypoints: still to come in blue, already reached in grey.
        if self.mission_waypoints:
            done = new_marker(3, Marker.POINTS, 1.5)
            done.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7)
            todo = new_marker(4, Marker.POINTS, 1.5)
            todo.color = ColorRGBA(r=0.1, g=0.5, b=1.0, a=0.9)
            for index, (lat, lon) in enumerate(self.mission_waypoints):
                north, east = gps_to_local_ned(lat, lon, home_lat, home_lon)
                point = Point(x=east, y=north, z=0.0)
                (done if index < self.mission_index else todo).points.append(point)
            array.markers.append(done)
            array.markers.append(todo)

        # The payload's current target, when there is a live one.
        if (self.autonomy_enabled
                and self._command_is_fresh(self.payload_waypoint_time)
                and self.payload_waypoint is not None):
            north, east = gps_to_local_ned(
                self.payload_waypoint.latitude, self.payload_waypoint.longitude,
                home_lat, home_lon)
            target = new_marker(2, Marker.SPHERE, 2.0)
            target.color = ColorRGBA(r=0.1, g=0.5, b=1.0, a=0.9)
            target.pose.position = Point(x=east, y=north, z=0.0)
            array.markers.append(target)

        self.viz_pub.publish(array)
        self._viz_markers_active = True


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Brain node interrupted — shutting down.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
