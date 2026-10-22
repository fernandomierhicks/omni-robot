# The catch path's filter (omni_catch.tracker), odom frame, at header.stamp (the last frame it used)
std_msgs/Header header
bool tracking
bool ready                    # enough frames to plan on
uint32 frames
float64[6] x                  # position, velocity
float64[36] covariance
# where it crosses the funnel's rim height (at tilt 0), for the views
float64[3] crossing
float64 t_go
float64 crossing_sigma
