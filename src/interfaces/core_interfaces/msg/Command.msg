# A command from the backseat autonomy layer to the MAVLink gateway.
#
# This is the ONLY message type that can move the vehicle from ROS 2. The
# gateway rejects any Command whose source_node is not "core_autonomy", so a
# payload cannot reach the autopilot even though the topic is visible to it.

# --- Validation ---------------------------------------------------------
# Name of the sending node. Must be "core_autonomy"; anything else is logged
# and dropped by the gateway's security check.
string source_node
# Optional sender timestamp (nanoseconds). Unused by the gateway; available
# for payloads and log analysis.
uint64 timestamp

# --- Command types ------------------------------------------------------
# CMD_MODE           Heartbeat / lease keepalive. Carries no authority: the
#                    ArduPilot flight mode is set only by the operator via RC
#                    or ground station. string_param holds the brain's own
#                    intent for logging.
# CMD_WAYPOINT       Navigate to float_params = [latitude, longitude].
#                    Forwarded only while the frontseat is in GUIDED.
# CMD_ARM            Reserved. Arming is handled by the gateway's own
#                    reconciliation loop; no node publishes this today.
# CMD_EMERGENCY_HOLD Reserved for an explicit operator-initiated stop.
uint8 CMD_MODE = 1
uint8 CMD_ARM = 2
uint8 CMD_WAYPOINT = 3
uint8 CMD_EMERGENCY_HOLD = 4

uint8 command_type

# --- Payload ------------------------------------------------------------
string string_param
float64[] float_params
