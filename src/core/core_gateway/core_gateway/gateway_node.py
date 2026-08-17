"""
Core MAVLink Gateway Node

This node acts as the bridge between backseat (payloads) and the frontseat (blueboat).
1. Translates ROS 2 commands into MAVLink packets for the Frontseat (ArduRover).
2. Streams physical telemetry (GPS, Heading, Speed, Yaw Rate, Battery) to the ROS 2 network.
3. Enforces the Hardware Watchdog: if the Brain node dies, revokes the autonomy lease.
4. Acts as a Security Gate, rejecting commands from unauthorized ROS namespaces.
5. Translates Twist velocity commands into MAVLink body-frame velocity requests.

ArduPilot mode is NEVER set by the backseat. GUIDED must be set by the operator
via RC transmitter or GCS. The gateway only reads mode from HEARTBEAT telemetry.
"""

import os
import time
import rclpy
from rclpy.node import Node

# --- ROS 2 Message Types ---
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState, Temperature
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from core_interfaces.msg import Command

from pymavlink import mavutil

# ArduRover mode name → custom_mode integer (MAV_TYPE_GROUND_ROVER = 10).
# pymavlink's set_mode_apm relies on the HEARTBEAT's mav_type being correctly
# identified; if that lookup fails it prints "Unknown mode" and no-ops.
# Using explicit integers avoids that fragility entirely.
_ROVER_MODE = {
    'MANUAL': 0,
    'ACRO': 1,
    'STEERING': 3,
    'HOLD': 4,
    'LOITER': 5,
    'FOLLOW': 6,
    'SIMPLE': 7,
    'AUTO': 10,
    'RTL': 11,
    'SMART_RTL': 12,
    'GUIDED': 15,
}
_ROVER_MODE_NAME_BY_NUM = {value: key for key, value in _ROVER_MODE.items()}


class GatewayNode(Node):
    def __init__(self):
        super().__init__('gateway_node')
        self.get_logger().info("Initializing Gateway...")

        # --- ROS Parameters ---
        # Auto-arm the frontseat when the Brain is active and the desired
        # mode is something other than HOLD. Set false on real-boat
        # deployments where arming must be operator-gated.
        self.declare_parameter('auto_arm', True)

        # --- MAVLink Connection ---
        # GATEWAY_EXPECTED_SOURCE (env var): if set, the gateway aborts at
        # startup when the first vehicle HEARTBEAT does not come from this IP.
        # Use in sim mode (GATEWAY_EXPECTED_SOURCE=$LAPTOP_IP) so the gateway
        # fails fast rather than silently arming the real Navigator if BlueOS
        # is running alongside the SITL.
        _expected_source = os.environ.get('GATEWAY_EXPECTED_SOURCE', '').strip()

        # MAVLINK_PORT (env var): UDP port to listen on for MAVLink traffic.
        # Default 14550 (real boat / BlueOS Navigator HAT default).
        # Set to 14552 in sim mode (start-simulated-mission.sh) so the real
        # frontseat Pi, which sends to 14550, can never reach this gateway
        # even when both are on the same network.
        _port = int(os.environ.get('MAVLINK_PORT', '14550'))
        self.boat = mavutil.mavlink_connection(f'udpin:0.0.0.0:{_port}')

        # Wait for the autopilot's HEARTBEAT specifically. The BlueOS MAVLink
        # router forwards every component's heartbeats to this endpoint —
        # mavlink2rest (sysid 1, comp 194, ONBOARD_CONTROLLER), the camera
        # manager (comps 100-105, CAMERA) and GCSes all share the stream, and
        # their heartbeats carry custom_mode = 0. Locking onto one of those
        # makes the gateway report MANUAL forever and address commands to the
        # wrong component, so require the flight controller's component id
        # (AUTOPILOT1) and a rover/boat vehicle type (SITL reports
        # GROUND_ROVER, the real BlueBoat SURFACE_BOAT).
        _VEHICLE_TYPES = (mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                          mavutil.mavlink.MAV_TYPE_GROUND_ROVER)
        while True:
            msg = self.boat.recv_match(type='HEARTBEAT', blocking=True, timeout=60)
            if msg is None:
                raise RuntimeError(
                    'No MAVLink HEARTBEAT received within 60 s — '
                    'is ArduPilot / SITL running and forwarding to UDP:14550?')
            if (msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                    and msg.type in _VEHICLE_TYPES):
                self.boat.target_system = msg.get_srcSystem()
                self.boat.target_component = msg.get_srcComponent()
                break

        _connected_ip = getattr(self.boat, 'address', ('unknown', 0))[0]
        if _expected_source:
            if _connected_ip in ('unknown', '0', ''):
                self.get_logger().warning(
                    f"GATEWAY_EXPECTED_SOURCE={_expected_source!r} is set, but "
                    f"connected MAVLink source is {_connected_ip!r}. "
                    "Skipping strict source verification.")
            elif _connected_ip != _expected_source:
                raise RuntimeError(
                    f"Connected to MAVLink source {_connected_ip!r} but "
                    f"GATEWAY_EXPECTED_SOURCE={_expected_source!r}. "
                    "Stop BlueOS / real Navigator before running in sim mode.")

        self.get_logger().info(
            f"MAVLink Connected to Frontseat! (source={_connected_ip}, port={_port})")

        # Request telemetry streams needed by payloads and the brain:
        # - POSITION:        GLOBAL_POSITION_INT for GPS + heading
        # - EXTRA1:          VFR_HUD for groundspeed
        # - EXTRA2:          ATTITUDE for IMU yaw rate
        # - EXTENDED_STATUS: GPS_RAW_INT for the satellite fix quality
        # This keeps frontseat/speed and frontseat/yaw_rate sourced from
        # ArduPilot (SITL or hardware), not from any simulator physics.
        self._request_required_streams()

        # --- Internal State ---
        self.autonomy_active = False
        self.last_heartbeat_time = time.time()

        # Latest GPS fix quality, from GPS_RAW_INT.fix_type. GLOBAL_POSITION_INT
        # carries no fix quality of its own and is published even before the EKF
        # has a position — as (0, 0). Stamping every NavSatFix with the real fix
        # status lets the brain and every payload tell a position from a
        # placeholder, instead of each having to guess.
        self._gps_fix_type = 0

        # Arming state reconciliation. `_actual_armed` is driven by the
        # SAFETY_ARMED bit in incoming HEARTBEATs; `_desired_armed` is set
        # by mode transitions (True when entering any autonomy mode, False
        # only on watchdog failsafe). The 1 Hz watchdog retries arm requests
        # until the two agree, which covers SITL prearm checks that can take
        # a few seconds to pass after EKF init.
        self._actual_armed = False
        self._desired_armed = False
        self._last_arm_attempt_t = 0.0
        self._arm_confirmed_logged = False  # one-shot: suppress repeat "armed" messages
        # `_actual_mode_num` is read from HEARTBEAT.custom_mode and used to
        # gate waypoint/velocity forwarding (GUIDED-only) and lease acquisition.
        # The backseat never writes the mode register — ArduPilot mode is set
        # exclusively by the operator via RC transmitter or GCS.
        self._actual_mode_num = None

        # --- Publishers (Telemetry)---
        self.gps_pub = self.create_publisher(NavSatFix, '/frontseat/gps', 10)
        self.heading_pub = self.create_publisher(Float64, '/frontseat/heading', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/frontseat/battery', 10)
        self.temp_pub = self.create_publisher(Temperature, '/frontseat/temperature', 10)
        self.speed_pub = self.create_publisher(Float64, '/frontseat/speed', 10)
        self.yaw_rate_pub = self.create_publisher(Float64, '/frontseat/yaw_rate', 10)
        # Current ArduPilot mode name published on every HEARTBEAT for subscribers.
        self.mode_pub = self.create_publisher(String, '/frontseat/mode', 10)

        # --- Subscribers (Commands) ---
        self.cmd_sub = self.create_subscription(
            Command, '/system/brain/command', self.cmd_callback, 10)

        # Velocity subscriber — Brain publishes muxed Twist here in speed mode.
        # We translate this to MAVLink body-frame velocity via
        # SET_POSITION_TARGET_LOCAL_NED so the ArduRover PID handles execution.
        self.vel_sub = self.create_subscription(
            Twist, '/system/brain/cmd_vel', self.vel_callback, 10)

        # --- Timers ---
        # 10Hz loop to read incoming telemetry from the flight controller
        self.mavlink_timer = self.create_timer(0.1, self.read_mavlink)
        # 1Hz loop to check if the Brain node is still alive
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_check)

    # =========================================================================
    # MAVLINK -> ROS 2 (TELEMETRY)
    # =========================================================================

    def read_mavlink(self):
        # Drain all available messages per tick so fast-streaming message types
        # (ATTITUDE, VFR_HUD, etc.) never block GPS from reaching ROS.
        while True:
            msg = self.boat.recv_match(blocking=False)
            if not msg:
                break

            msg_type = msg.get_type()

            # 1. Track GPS fix quality (see _gps_fix_type above).
            if msg_type == 'GPS_RAW_INT':
                self._gps_fix_type = int(msg.fix_type)

            # 2. Parse GPS and Heading
            elif msg_type == 'GLOBAL_POSITION_INT':
                ros_gps = NavSatFix()
                ros_gps.header.stamp = self.get_clock().now().to_msg()
                ros_gps.header.frame_id = 'gps'
                ros_gps.latitude = msg.lat / 1e7
                ros_gps.longitude = msg.lon / 1e7
                ros_gps.altitude = msg.alt / 1000.0
                # fix_type follows GPS_FIX_TYPE: 0 no GPS, 1 no fix, 2 2D,
                # 3 3D, 4 DGPS, 5/6 RTK. Anything below 2D is not a position.
                ros_gps.status.status = (
                    NavSatStatus.STATUS_FIX if self._gps_fix_type >= 2
                    else NavSatStatus.STATUS_NO_FIX)
                ros_gps.status.service = NavSatStatus.SERVICE_GPS
                self.gps_pub.publish(ros_gps)

                # Publish Heading (msg.hdg is in centidegrees, up to 35999. 65535 means invalid/unknown)
                if msg.hdg != 65535:
                    ros_heading = Float64()
                    ros_heading.data = msg.hdg / 100.0
                    self.heading_pub.publish(ros_heading)

            # 3. Track frontseat arming state and mode from HEARTBEAT.
            #
            # When the gateway connects via a MAVProxy relay (e.g. remote SITL
            # in SIM_PI_YOLO mode), MAVProxy also forwards its own GCS-type
            # HEARTBEAT alongside the vehicle's.  GCS HEARTBEATs carry
            # custom_mode = 0, which maps to MANUAL in ArduRover and would
            # spuriously revoke the autonomy lease.  Skip any HEARTBEAT whose
            # MAV_TYPE is GCS so only vehicle telemetry updates state.
            elif msg_type == 'HEARTBEAT':
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue
                if msg.get_srcSystem() != self.boat.target_system:
                    continue
                if msg.get_srcComponent() != self.boat.target_component:
                    continue
                self._actual_armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._actual_mode_num = int(msg.custom_mode)

                # Publish current ArduPilot mode so brain and other nodes can read it.
                mode_name = _ROVER_MODE_NAME_BY_NUM.get(
                    self._actual_mode_num, str(self._actual_mode_num))
                mode_msg = String()
                mode_msg.data = mode_name
                self.mode_pub.publish(mode_msg)

            # 4. Parse Battery State
            elif msg_type == 'SYS_STATUS':
                ros_batt = BatteryState()
                ros_batt.voltage = msg.voltage_battery / 1000.0

                # Current is in 10mA increments (-1 means no sensor)
                if msg.current_battery != -1:
                    ros_batt.current = msg.current_battery / 100.0

                # Remaining is percentage (-1 means no sensor)
                if msg.battery_remaining != -1:
                    ros_batt.percentage = msg.battery_remaining / 100.0

                self.battery_pub.publish(ros_batt)

            # 5. Parse Temperature
            elif msg_type == 'SCALED_PRESSURE':
                ros_temp = Temperature()
                ros_temp.header.stamp = self.get_clock().now().to_msg()
                ros_temp.temperature = msg.temperature / 100.0  # MAVLink sends centidegrees
                self.temp_pub.publish(ros_temp)

            # 6. Parse groundspeed (m/s, scalar magnitude)
            elif msg_type == 'VFR_HUD':
                ros_speed = Float64()
                ros_speed.data = float(msg.groundspeed)
                self.speed_pub.publish(ros_speed)

            # 7. Parse body yaw rate (rad/s, IMU-derived)
            elif msg_type == 'ATTITUDE':
                ros_yaw_rate = Float64()
                ros_yaw_rate.data = float(msg.yawspeed)
                self.yaw_rate_pub.publish(ros_yaw_rate)

    # =========================================================================
    # ROS 2 -> MAVLINK (COMMAND TRANSLATION)
    # =========================================================================

    def cmd_callback(self, msg):
        # 1. Security Check
        if msg.source_node != "core_autonomy":
            self.get_logger().warn(f"SECURITY REJECT: Unauthorized node '{msg.source_node}' tried to command the boat!")
            return

        # 2. Lease Renewal — brain keepalive; lease only counts while in GUIDED
        # (operator has authorized autonomous operation).
        self.last_heartbeat_time = time.time()
        if not self.autonomy_active and self._actual_mode_num == _ROVER_MODE['GUIDED']:
            self.get_logger().info("Autonomy Lease Acquired. State: ACTIVE")
            self.autonomy_active = True

        # 3. Arming follows the brain's own enable flag, carried in the
        # heartbeat's string_param. Arming merely on lease acquisition would
        # spin up a vehicle whose operator selected GUIDED intending to drive
        # it themselves. Requiring the brain to be actively relaying means
        # nothing moves until someone asks for it twice: once by selecting
        # GUIDED, once by enabling autonomy.
        #
        # The watchdog retries until ArduPilot's prearm checks pass. Set the
        # auto_arm parameter false to keep arming strictly operator-driven.
        #
        # CMD_MODE itself commands nothing: ArduPilot's flight mode is written
        # exclusively by the operator via RC or a ground station. The message
        # exists to prove the brain is alive and to report what it intends.
        if msg.command_type == Command.CMD_MODE:
            self._desired_armed = (msg.string_param == 'ENABLED')

        # 4. MAVLink translation
        elif msg.command_type == Command.CMD_WAYPOINT:
            # Only forward waypoints when the frontseat is actually in GUIDED;
            # ArduRover can auto-enter GUIDED on receiving SET_POSITION_TARGET,
            # which would override the operator in unexpected ways.
            if self._actual_mode_num != _ROVER_MODE['GUIDED']:
                return
            lat_int = int(msg.float_params[0] * 1e7)
            lon_int = int(msg.float_params[1] * 1e7)

            self.boat.mav.set_position_target_global_int_send(
                0, self.boat.target_system, self.boat.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                3576, lat_int, lon_int, 0,
                0, 0, 0, 0, 0, 0, 0, 0)

    # =========================================================================
    # TWIST -> MAVLINK (VELOCITY CONTROL)
    # =========================================================================

    def vel_callback(self, msg: Twist):
        """Translate a Twist from the Brain's velocity mux into a MAVLink
        body-frame velocity command.

        Uses SET_POSITION_TARGET_LOCAL_NED with:
          - type_mask = 0b0000_11_0_111_000_111 (= 0x0DC7)
            Bits set → ignore position (xyz), acceleration (ax/ay/az),
            and yaw angle.  Bits clear → use vx, vy, vz, yaw_rate.
          - coordinate_frame = MAV_FRAME_BODY_NED so vx is forward
            along the hull, not geographic north.
          - vx = msg.linear.x  (m/s forward)
          - vy = 0             (no lateral thrust on skid-steer)
          - vz = 0             (surface vessel)
          - yaw_rate = msg.angular.z  (rad/s)

        The ArduRover firmware's internal PID loops handle the actual
        thruster mixing to achieve the requested velocity and yaw rate.
        """
        if not self.autonomy_active:
            # Don't send velocity if no valid heartbeat lease
            return

        if self._actual_mode_num != _ROVER_MODE['GUIDED']:
            # ArduRover can auto-enter GUIDED on receiving SET_POSITION_TARGET,
            # overriding the operator. Only send velocity when already in GUIDED.
            return

        # Type mask: use vx, vy, vz, yaw_rate — ignore everything else
        # Bit layout (LSB → MSB):
        #   0-2: ignore pos x,y,z  = 0b111 = 7
        #   3-5: USE vel x,y,z     = 0b000 = 0
        #   6-8: ignore accel      = 0b111 = 7 → shifted = 0x01C0
        #   9:   ignore force       = 0b0
        #  10:   ignore yaw         = 0b1   → shifted = 0x0400
        #  11:   USE yaw_rate       = 0b0
        # Result = 0b0000_10_0_111_000_111 = 0x04C7
        type_mask = 0x04C7

        self.boat.mav.set_position_target_local_ned_send(
            0,                                             # time_boot_ms (0 = auto)
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,            # body-frame: vx = forward
            type_mask,
            0, 0, 0,                                       # x, y, z (ignored)
            float(msg.linear.x), 0.0, 0.0,                 # vx, vy, vz (m/s)
            0, 0, 0,                                       # afx, afy, afz (ignored)
            0,                                              # yaw (ignored)
            float(msg.angular.z),                           # yaw_rate (rad/s)
        )

    # =========================================================================
    # HARDWARE WATCHDOG & FAILSAFE
    # =========================================================================

    def watchdog_check(self):
        # 1) Brain heartbeat failsafe. If the brain dies while holding an
        # autonomy lease, revoke the lease and stop forwarding commands.
        # ArduPilot mode is NOT changed — the operator must take over via RC.
        if self.autonomy_active and (time.time() - self.last_heartbeat_time > 2.0):
            self.get_logger().error(
                "FAILSAFE: Brain Heartbeat Lost! Autonomy lease revoked. "
                "No further waypoint/velocity commands will be forwarded. "
                "Take manual control via RC transmitter.")
            self.autonomy_active = False
            # Stop *requesting* arming. This does not disarm: the vehicle stays
            # armed so ArduPilot can keep holding station against wind and
            # current. Disarming here would leave the boat drifting freely with
            # no autonomy running and no operator expecting it to move.
            self._desired_armed = False
            return

        # 2) Arming reconciliation. If the Brain is driving and wants us
        # armed, keep retrying until ArduPilot's prearm checks agree. A
        # fresh SITL usually passes prearm within a few seconds of EKF
        # convergence, but on a real boat this can take longer — so we
        # just keep asking at 1 Hz until the HEARTBEAT base_mode shows
        # MAV_MODE_FLAG_SAFETY_ARMED. If prearm is failing, the operator
        # will see the reason in the SITL/MAVProxy console or the
        # STATUSTEXT stream; we do not surface those here to avoid
        # double-logging.
        if not self.get_parameter('auto_arm').value:
            return
        if self._desired_armed and not self._actual_armed:
            self._arm_confirmed_logged = False
            now = time.time()
            if (now - self._last_arm_attempt_t) >= 1.0:
                self._send_arm_disarm(True)
                self._last_arm_attempt_t = now
                self.get_logger().info(
                    "Arm requested — retrying until ArduPilot prearm checks pass.")
        elif self._desired_armed and self._actual_armed and not self._arm_confirmed_logged:
            self.get_logger().info("ArduPilot armed — autonomy ready.")
            self._arm_confirmed_logged = True

    def _request_required_streams(self, rate_hz: int = 5):
        """Ask ArduPilot for the telemetry streams required by the stack.

        Requests:
          - MAV_DATA_STREAM_POSITION        (GLOBAL_POSITION_INT, etc.)
          - MAV_DATA_STREAM_EXTRA1          (VFR_HUD, etc.)
          - MAV_DATA_STREAM_EXTRA2          (ATTITUDE, etc.)
          - MAV_DATA_STREAM_EXTENDED_STATUS (GPS_RAW_INT, etc.)
        """
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 'POSITION'),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 'EXTRA1'),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 'EXTRA2'),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 'EXTENDED_STATUS'),
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
                f'Requested MAV_DATA_STREAM_{stream_name} at {rate_hz} Hz from frontseat.')

    def _send_arm_disarm(self, arm: bool) -> None:
        """Fire a MAV_CMD_COMPONENT_ARM_DISARM (idempotent on ArduRover)."""
        self.boat.mav.command_long_send(
            self.boat.target_system,
            self.boat.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0, 0, 0, 0, 0, 0,
        )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GatewayNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
