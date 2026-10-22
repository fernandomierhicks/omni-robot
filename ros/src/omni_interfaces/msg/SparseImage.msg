# A synthetic camera frame from the sim (omni_catch.frames.SparseFrame): what differs from the plain background
std_msgs/Header header        # stamp = exposure; frame_id = the camera
uint32 width
uint32 height
uint32[] ball                 # flat pixel indices (y * width + x) the color threshold calls ball
uint32[] clutter              # more ball-colored pixels that are not the ball
uint32 seed                   # the background noise's
