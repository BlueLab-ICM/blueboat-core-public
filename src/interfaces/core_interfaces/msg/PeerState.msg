# Latest known state of another vehicle, from the fleet link.

std_msgs/Header header    # stamp = receipt time on this vehicle
string vehicle
float64 latitude          # degrees
float64 longitude         # degrees
float64 heading_rad       # NED, 0 = north, clockwise positive
float64 speed_mps
