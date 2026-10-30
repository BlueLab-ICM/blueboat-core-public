# One motion request. Modules publish it on /helm/setpoint. The helm publishes the one
# command it chose on /helm/output, which only the gateway reads.

uint8 MODE_KEEPALIVE = 0  # no motion, renews the autonomy lease (helm output only)
uint8 MODE_POSITION = 1   # drive to latitude, longitude
uint8 MODE_VELOCITY = 2   # speed_mps forward, yaw_rate_rps about the vertical
uint8 MODE_HOLD = 3       # keep station at latitude, longitude

std_msgs/Header header
string source             # the requesting module; must hold the helm grant
uint8 mode
float64 latitude          # degrees
float64 longitude         # degrees
float64 speed_mps
float64 yaw_rate_rps
