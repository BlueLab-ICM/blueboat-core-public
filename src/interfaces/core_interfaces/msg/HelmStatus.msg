# Helm state, latched and republished at 1 Hz.

string STATE_IDLE = idle            # not engaged, sends keepalives only
string STATE_ENGAGED = engaged      # relaying the granted source
string STATE_HOLDING = holding      # engaged, holding position
string STATE_AVOIDING = avoiding    # engaged, the collision filter is steering

std_msgs/Header header
string state
string granted_source
string active_source      # the source whose command went out on the last tick
string override           # the rule that replaced the request, empty when none did
string reason             # why the last engagement ended, or why the helm refused
bool guided               # the autopilot is in GUIDED
float64 fence_latitude
float64 fence_longitude
float64 fence_radius_m
