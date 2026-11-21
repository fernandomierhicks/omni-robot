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
float64[] current        # A each (empty: the servos don't report current)
float64[] external       # N m: the outside torque on each joint (design.md 4.5; 0 inside the friction band at rest)
bool[] strained          # the strain detector's
float64[3] tool_force    # N, base_link: the outside torques as one force at the tool point
string load_model        # the load model in use: "nominal" or the calibration's source
uint32 reflexes          # how many times the reflex has stopped the arm
string reflex            # the last reflex's joints and torques ("" none)
bool reflex_active       # the reflex holds the arm until the next command
string[] servo_notes     # what connect changed in the servos' settings (P / D, protections) or couldn't check
