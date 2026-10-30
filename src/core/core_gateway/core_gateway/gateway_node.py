"""Bridge between the backseat and the frontseat autopilot (ArduRover over MAVLink).

Publishes autopilot telemetry on /frontseat/*, forwards the helm's output under the
rules in core_gateway.safety, and revokes the autonomy lease when the helm goes
silent. The backseat never sets the autopilot mode; the operator does.

It also sets the autopilot's home on request (/frontseat/set_home), reports the home
and the link-loss failsafe parameters (/frontseat/home, /frontseat/failsafe), the
autopilot's GPS time (/frontseat/time) and one parameter snapshot (/frontseat/params).
"""

import json
import math
import os
import time

import rclpy
from core_common.geodesy import haversine_m
from core_common.health import ERROR, OK, WARN, ModuleHealth
from core_interfaces.msg import Setpoint
from pymavlink import mavutil
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Temperature, TimeReference
from std_msgs.msg import Float64, Int32MultiArray, String

from core_gateway.params import ParamCollector
from core_gateway.safety import ROVER_MODE_NAME_BY_NUM, AutonomyLease

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# Parameters that decide what the autopilot does when the base-station link drops.
FAILSAFE_PARAMS = ('FS_GCS_ENABLE', 'FS_ACTION', 'FS_GCS_TIMEOUT', 'FS_TIMEOUT')
# A requested home farther than this from the vessel is refused as a typo.
MAX_HOME_DISTANCE_M = 1000.0


class GatewayNode(Node):
    """Sole holder of vehicle-command authority: nothing else may talk to the autopilot."""

    def __init__(self) -> None:
        super().__init__('gateway_node')
        self.get_logger().info('Initializing Gateway...')

        # In sim, abort if the first autopilot HEARTBEAT comes from another IP, so a
        # BlueOS running alongside the SITL cannot be commanded by mistake.
        _expected_source = os.environ.get('GATEWAY_EXPECTED_SOURCE', '').strip()

        # 14550 on the boat (BlueOS default); 14552 in sim, so the real frontseat
        # cannot reach a sim gateway on the same network.
        _port = int(os.environ.get('MAVLINK_PORT', '14550'))
        self.boat = mavutil.mavlink_connection(f'udpin:0.0.0.0:{_port}')

        # Lock onto the flight controller only: BlueOS also forwards heartbeats from
        # mavlink2rest, cameras and GCSes, whose custom_mode 0 reads as MANUAL.
        # SITL reports GROUND_ROVER, the real BlueBoat SURFACE_BOAT.
        _VEHICLE_TYPES = (  # noqa: N806 - local constant
            mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
            mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
        )
        while True:
            msg = self.boat.recv_match(type='HEARTBEAT', blocking=True, timeout=60)
            if msg is None:
                raise RuntimeError(
                    'No MAVLink HEARTBEAT received within 60 s — '
                    'is ArduPilot / SITL running and forwarding to UDP:14550?'
                )
            if (
                msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                and msg.type in _VEHICLE_TYPES
            ):
                self.boat.target_system = msg.get_srcSystem()
                self.boat.target_component = msg.get_srcComponent()
                break

        _connected_ip = getattr(self.boat, 'address', ('unknown', 0))[0]
        if _expected_source:
            if _connected_ip in ('unknown', '0', ''):
                self.get_logger().warning(
                    f'GATEWAY_EXPECTED_SOURCE={_expected_source!r} is set, but '
                    f'connected MAVLink source is {_connected_ip!r}. '
                    'Skipping strict source verification.'
                )
            elif _connected_ip != _expected_source:
                raise RuntimeError(
                    f'Connected to MAVLink source {_connected_ip!r} but '
                    f'GATEWAY_EXPECTED_SOURCE={_expected_source!r}. '
                    'Stop BlueOS / real Navigator before running in sim mode.'
                )

        self.get_logger().info(
            f'MAVLink Connected to Frontseat! (source={_connected_ip}, port={_port})'
        )

        # Speed and yaw rate come from ArduPilot (SITL or hardware), not Gazebo.
        self._request_required_streams()

        self.lease = AutonomyLease()

        self._actual_mode_num = None  # from HEARTBEAT.custom_mode; never written by us

        self.gps_pub = self.create_publisher(NavSatFix, '/frontseat/gps', 10)
        self.heading_pub = self.create_publisher(Float64, '/frontseat/heading', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/frontseat/battery', 10)
        self.temp_pub = self.create_publisher(Temperature, '/frontseat/temperature', 10)
        self.speed_pub = self.create_publisher(Float64, '/frontseat/speed', 10)
        self.yaw_rate_pub = self.create_publisher(Float64, '/frontseat/yaw_rate', 10)
        self.mode_pub = self.create_publisher(String, '/frontseat/mode', 10)
        # servo1_raw..servo8_raw in microseconds.
        self.thruster_pub = self.create_publisher(Int32MultiArray, '/frontseat/thruster_pwm', 10)
        self.home_pub = self.create_publisher(NavSatFix, '/frontseat/home', LATCHED)
        self.failsafe_pub = self.create_publisher(String, '/frontseat/failsafe', LATCHED)
        self.time_pub = self.create_publisher(TimeReference, '/frontseat/time', 10)
        self.params_pub = self.create_publisher(String, '/frontseat/params', LATCHED)
        self.health = ModuleHealth(self, 'gateway')
        self.health.event('mavlink_connected', source=_connected_ip, port=_port)
        self.params = ParamCollector()
        self.boat.mav.param_request_list_send(self.boat.target_system, self.boat.target_component)
        self._reported_mode: str | None = None
        self._last_lat: float | None = None
        self._last_lon: float | None = None
        self._last_alt_m = 0.0
        self._home: tuple[float, float, float] | None = None
        self._failsafe: dict[str, float] = {}

        # GLOBAL_POSITION_INT carries no fix quality, so track it from
        # GPS_RAW_INT. None means none seen yet: report UNKNOWN, do not guess.
        self._gps_fix_type: int | None = None
        self._gps_had_fix = False

        self.output_sub = self.create_subscription(
            Setpoint, '/helm/output', self.output_callback, 10
        )
        self.set_home_sub = self.create_subscription(
            NavSatFix, '/frontseat/set_home', self.set_home_callback, 10
        )

        # Steady clock: a wall-clock step would stall timers on the ROS clock.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.mavlink_timer = self.create_timer(0.1, self.read_mavlink, clock=steady)
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_check, clock=steady)
        # Home and failsafe parameters change rarely (and QGroundControl can
        # change them too), so poll them.
        self.home_timer = self.create_timer(5.0, self._request_home_and_failsafe, clock=steady)
        self._request_home_and_failsafe()
        self.time_timer = self.create_timer(10.0, self._request_system_time, clock=steady)
        self._request_system_time()
        self.params_timer = self.create_timer(1.0, self._publish_params_when_done, clock=steady)

    # =========================================================================
    # MAVLINK -> ROS 2 (TELEMETRY)
    # =========================================================================

    def read_mavlink(self) -> None:
        """Drain the MAVLink socket and republish what arrives onto ROS 2 topics."""
        # Drain all available messages per tick so fast-streaming message types
        # (ATTITUDE, VFR_HUD, etc.) never block GPS from reaching ROS.
        while True:
            msg = self.boat.recv_match(blocking=False)
            if not msg:
                break

            msg_type = msg.get_type()

            if msg_type == 'GPS_RAW_INT':
                self._gps_fix_type = msg.fix_type
                has_fix = msg.fix_type >= 2
                if self._gps_had_fix and not has_fix:
                    self.get_logger().error(
                        f'GPS FIX LOST (fix_type={msg.fix_type}, '
                        f'{msg.satellites_visible} sats). Position is no longer trustworthy.'
                    )
                    self.health.event('gps_fix_lost', level=ERROR, fix_type=msg.fix_type)
                    self.health.error('no GPS fix')
                elif has_fix and not self._gps_had_fix:
                    self.health.event('gps_fix', fix_type=msg.fix_type)
                    self.health.ok()
                self._gps_had_fix = has_fix

            if msg_type == 'GLOBAL_POSITION_INT':
                # The stamp and status let the geofence tell a stale fix from a live one.
                ros_gps = NavSatFix()
                ros_gps.header.stamp = self.get_clock().now().to_msg()
                ros_gps.header.frame_id = 'gps'
                ros_gps.status.status = self._navsat_status()
                ros_gps.status.service = NavSatStatus.SERVICE_GPS
                ros_gps.latitude = msg.lat / 1e7
                ros_gps.longitude = msg.lon / 1e7
                self.gps_pub.publish(ros_gps)
                self._last_lat, self._last_lon = ros_gps.latitude, ros_gps.longitude
                self._last_alt_m = msg.alt / 1000.0

                # hdg is centidegrees; 65535 means unknown.
                if msg.hdg != 65535:
                    ros_heading = Float64()
                    ros_heading.data = msg.hdg / 100.0
                    self.heading_pub.publish(ros_heading)

            # Only the locked autopilot updates the mode. GCS heartbeats, e.g.
            # from a MAVProxy relay, carry custom_mode 0, which reads as MANUAL.
            elif msg_type == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue
                if msg.get_srcSystem() != self.boat.target_system:
                    continue
                if msg.get_srcComponent() != self.boat.target_component:
                    continue
                self._actual_mode_num = int(msg.custom_mode)

                mode_name = ROVER_MODE_NAME_BY_NUM.get(
                    self._actual_mode_num, str(self._actual_mode_num)
                )
                mode_msg = String()
                mode_msg.data = mode_name
                self.mode_pub.publish(mode_msg)
                if mode_name != self._reported_mode:
                    self.health.event('autopilot_mode', mode=mode_name)
                    self._reported_mode = mode_name

            elif msg_type == 'SYS_STATUS':
                ros_batt = BatteryState()
                ros_batt.voltage = msg.voltage_battery / 1000.0

                # Current is in 10mA increments (-1 means no sensor). BatteryState
                # marks an unmeasured value as NaN; its 0.0 default would read as
                # "no current" or "battery empty".
                ros_batt.current = (
                    msg.current_battery / 100.0 if msg.current_battery != -1 else math.nan
                )

                # Remaining is percentage (-1 means no sensor)
                ros_batt.percentage = (
                    msg.battery_remaining / 100.0 if msg.battery_remaining != -1 else math.nan
                )

                self.battery_pub.publish(ros_batt)

            elif msg_type == 'HOME_POSITION':
                home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
                if home != self._home:
                    self.get_logger().info(
                        f'Autopilot home: {home[0]:.7f}, {home[1]:.7f} ({home[2]:.1f} m AMSL)'
                    )
                self._home = home
                ros_home = NavSatFix()
                ros_home.header.stamp = self.get_clock().now().to_msg()
                ros_home.header.frame_id = 'home'
                ros_home.latitude, ros_home.longitude, ros_home.altitude = home
                self.home_pub.publish(ros_home)

            elif msg_type == 'SYSTEM_TIME':
                # Zero until the autopilot has GPS time.
                if msg.time_unix_usec > 0:
                    ref = TimeReference()
                    ref.header.stamp = self.get_clock().now().to_msg()
                    ref.time_ref.sec = int(msg.time_unix_usec // 1_000_000)
                    ref.time_ref.nanosec = int(msg.time_unix_usec % 1_000_000) * 1000
                    ref.source = 'autopilot'
                    self.time_pub.publish(ref)

            elif msg_type == 'PARAM_VALUE':
                name = msg.param_id.rstrip('\x00') if isinstance(msg.param_id, str) else ''
                self.params.add(name, float(msg.param_value), msg.param_count, time.monotonic())
                if name in FAILSAFE_PARAMS:
                    self._failsafe[name] = float(msg.param_value)
                    fs = String()
                    fs.data = json.dumps(self._failsafe)
                    self.failsafe_pub.publish(fs)

            elif msg_type == 'COMMAND_ACK':
                if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
                    accepted = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                    (self.get_logger().info if accepted else self.get_logger().error)(
                        f'Set home {"accepted" if accepted else f"refused (result {msg.result})"}'
                    )
                    self._request_home_and_failsafe()

            elif msg_type == 'SCALED_PRESSURE':
                ros_temp = Temperature()
                ros_temp.header.stamp = self.get_clock().now().to_msg()
                ros_temp.temperature = msg.temperature / 100.0  # MAVLink sends centidegrees
                self.temp_pub.publish(ros_temp)

            # Groundspeed, m/s.
            elif msg_type == 'VFR_HUD':
                ros_speed = Float64()
                ros_speed.data = float(msg.groundspeed)
                self.speed_pub.publish(ros_speed)

            # Body yaw rate, rad/s.
            elif msg_type == 'ATTITUDE':
                ros_yaw_rate = Float64()
                ros_yaw_rate.data = float(msg.yawspeed)
                self.yaw_rate_pub.publish(ros_yaw_rate)

            elif msg_type == 'SERVO_OUTPUT_RAW':
                ros_thrusters = Int32MultiArray()
                ros_thrusters.data = [
                    msg.servo1_raw,
                    msg.servo2_raw,
                    msg.servo3_raw,
                    msg.servo4_raw,
                    msg.servo5_raw,
                    msg.servo6_raw,
                    msg.servo7_raw,
                    msg.servo8_raw,
                ]
                self.thruster_pub.publish(ros_thrusters)

    # =========================================================================
    # ROS 2 -> MAVLINK (COMMAND TRANSLATION)
    # =========================================================================

    def output_callback(self, msg: Setpoint) -> None:
        """Forward the helm's command, but only from the helm and only in GUIDED."""
        now = time.monotonic()
        was_active = self.lease.active
        if not self.lease.on_command(msg.source, now, self._actual_mode_num):
            self.get_logger().warn(
                f"SECURITY REJECT: Unauthorized node '{msg.source}' tried to command the boat!",
                throttle_duration_sec=5.0,
            )
            return
        if self.lease.active and not was_active:
            self.get_logger().info('Autonomy Lease Acquired. State: ACTIVE')
            self.health.event('lease_acquired')

        if msg.mode == Setpoint.MODE_KEEPALIVE:
            return
        # ArduRover auto-enters GUIDED on SET_POSITION_TARGET, so send only when in it.
        if not self.lease.allows_send(now, self._actual_mode_num):
            return
        if msg.mode in (Setpoint.MODE_POSITION, Setpoint.MODE_HOLD):
            self._send_position(msg.latitude, msg.longitude)
        elif msg.mode == Setpoint.MODE_VELOCITY:
            self._send_velocity(msg.speed_mps, msg.yaw_rate_rps)

    def _send_position(self, lat: float, lon: float) -> None:
        self.boat.mav.set_position_target_global_int_send(
            0,
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            3576,
            int(lat * 1e7),
            int(lon * 1e7),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _send_velocity(self, speed_mps: float, yaw_rate_rps: float) -> None:
        """Send a body-frame velocity target.

        vy and vz are 0 on a skid-steer surface vessel. ArduRover's controllers do the
        thruster mixing.
        """
        # Ignores position (bits 0-2), ax/ay (6-7) and yaw (10); uses velocity and
        # yaw_rate. Bit 8 (az) is clear.
        type_mask = 0x04C7

        self.boat.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (0 = auto)
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # body-frame: vx = forward
            type_mask,
            0,
            0,
            0,  # x, y, z (ignored)
            float(speed_mps),
            0.0,
            0.0,  # vx, vy, vz (m/s)
            0,
            0,
            0,  # afx, afy, afz (ignored)
            0,  # yaw (ignored)
            float(yaw_rate_rps),  # yaw_rate (rad/s)
        )

    # =========================================================================
    # WATCHDOG & FAILSAFE
    # =========================================================================

    def set_home_callback(self, msg: NavSatFix) -> None:
        """Set the autopilot's home (MAV_CMD_DO_SET_HOME), where RTL and the link-loss failsafe go.

        Not a motion command: it is accepted in any mode and without the lease.
        NaN altitude means the vessel's current altitude.
        """
        lat, lon = float(msg.latitude), float(msg.longitude)
        if not (math.isfinite(lat) and math.isfinite(lon)) or abs(lat) > 90 or abs(lon) > 180:
            self.get_logger().warn(f'Refusing set_home to ({lat}, {lon}): not a position.')
            return
        if self._last_lat is not None:
            d = haversine_m(lat, lon, self._last_lat, self._last_lon)
            if d > MAX_HOME_DISTANCE_M:
                self.get_logger().warn(
                    f'Refusing set_home {d:.0f} m from the vessel '
                    f'(limit {MAX_HOME_DISTANCE_M:.0f} m).'
                )
                return
        alt = float(msg.altitude) if math.isfinite(msg.altitude) else self._last_alt_m
        self.boat.mav.command_int_send(
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,
            0,
            0.0,  # param1 0: use the given location, not the current one
            0.0,
            0.0,
            0.0,
            round(lat * 1e7),
            round(lon * 1e7),
            alt,
        )
        self.get_logger().info(f'Set home requested: {lat:.7f}, {lon:.7f}')

    def _request_system_time(self) -> None:
        self.boat.mav.command_long_send(
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _publish_params_when_done(self) -> None:
        if not self.params.ready(time.monotonic()):
            return
        msg = String()
        msg.data = json.dumps(self.params.snapshot())
        self.params_pub.publish(msg)
        self.params.published = True
        self.params_timer.cancel()
        self.health.event(
            'autopilot_params',
            level=WARN if self.params.missing else OK,
            count=len(self.params.values),
            missing=self.params.missing,
        )

    def _request_home_and_failsafe(self) -> None:
        self.boat.mav.command_long_send(
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        for name in FAILSAFE_PARAMS:
            self.boat.mav.param_request_read_send(
                self.boat.target_system, self.boat.target_component, name.encode('ascii'), -1
            )

    def watchdog_check(self) -> None:
        """Revoke the autonomy lease when the helm stops sending keepalives."""
        # The mode is left alone; the operator takes over by RC.
        if not self.lease.tick(time.monotonic()):
            self.get_logger().error(
                'FAILSAFE: helm keepalive lost. Autonomy lease revoked. '
                'No further commands will be forwarded. '
                'Take manual control via RC transmitter.'
            )
            self.health.event('lease_revoked', level=ERROR)
            self.health.warn('lease revoked')

    def _navsat_status(self) -> int:
        """Map the autopilot's GPS fix type onto a NavSatStatus constant.

        ArduPilot fix_type: 0-1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
        """
        fix_type = self._gps_fix_type
        if fix_type is None:
            return NavSatStatus.STATUS_UNKNOWN
        if fix_type >= 5:
            return NavSatStatus.STATUS_GBAS_FIX
        if fix_type == 4:
            return NavSatStatus.STATUS_SBAS_FIX
        if fix_type >= 2:
            return NavSatStatus.STATUS_FIX
        return NavSatStatus.STATUS_NO_FIX

    def _request_required_streams(self, rate_hz: int = 5) -> None:
        """Ask ArduPilot for the telemetry streams required by the stack.

        Requests:
          - MAV_DATA_STREAM_POSITION      (GLOBAL_POSITION_INT, etc.)
          - MAV_DATA_STREAM_EXTENDED_STATUS (GPS_RAW_INT, for fix quality)
          - MAV_DATA_STREAM_EXTRA1        (VFR_HUD, etc.)
          - MAV_DATA_STREAM_EXTRA2        (ATTITUDE, etc.)
          - MAV_DATA_STREAM_RAW_CONTROLLER (SERVO_OUTPUT_RAW, thruster PWM)
        """
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 'POSITION'),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 'EXTENDED_STATUS'),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 'EXTRA1'),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 'EXTRA2'),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 'RAW_CONTROLLER'),
        ]
        for stream_id, stream_name in streams:
            self.boat.mav.request_data_stream_send(
                self.boat.target_system,
                self.boat.target_component,
                stream_id,
                rate_hz,
                1,  # 1 = start streaming
            )
            self.get_logger().info(
                f'Requested MAV_DATA_STREAM_{stream_name} at {rate_hz} Hz from frontseat.'
            )


def main(args: list[str] | None = None) -> None:
    """Spin the gateway node until shutdown."""
    rclpy.init(args=args)
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
