# What the Jetson asks of the MCU (protocol.h CmdBody). Times are on the Jetson's clock; the link node converts them.
uint8 REQ_DISARM=0
uint8 REQ_AUTO=1
uint8 DRIVE_IDLE=0
uint8 DRIVE_VEL_BODY=1
uint8 DRIVE_GOTO=2
std_msgs/Header header
uint8 req_mode
bool kid
uint8 drive_mode
bool hold_yaw                 # VEL_BODY: with wz = 0, hold the heading
float32 a                     # VEL_BODY vx | GOTO x (odom)
float32 b                     # VEL_BODY vy | GOTO y
float32 c                     # VEL_BODY wz | GOTO yaw
float32 v_end
float32 a_cap
float32 speed_cap
float64 t_brake               # s, Jetson clock; 0 = none: then coast_s, brake at a_brake, hold
float32 coast_s
float32 a_brake
float32 tilt
float64 t_tilt                # s, Jetson clock; 0 = none: then tilt_after
float32 tilt_after
float32 fly_rpm
uint16 fire_seq
