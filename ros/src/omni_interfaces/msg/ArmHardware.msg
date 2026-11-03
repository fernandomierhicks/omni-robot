# What the arm driver knows of the arm and its servo bus (design.md 11), 5 Hz and on every change of motion.
std_msgs/Header header
bool connected           # the bus is open and every servo answered the ping
bool torque
bool ok                  # every servo answered the last read
string port
string[] joint_names
uint8[] missing          # ids that didn't answer
float64[] position       # rad, measured
float64[] goal           # rad, commanded (empty when the torque is off)
float64[] load           # fraction of each servo's torque limit
uint8[] temp_c
float64 volts
bool tool_out            # the tool is outside the bumper or high (the base runs with the arm-out caps)
uint32 motion_id         # the id of the last ArmMotion taken
bool moving              # a MOVE is playing
