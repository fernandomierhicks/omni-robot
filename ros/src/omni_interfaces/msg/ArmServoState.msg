# The arm driver's every cycle (omni_cpp arm_driver, design.md 4.5): each servo's telemetry and what the load model
# makes of it. Joint order: /arm/hw's joint_names; the jaw's load fields are 0 (the model covers the five arm joints).
std_msgs/Header header
float64[] position       # rad: the joints' true angles as the load model knows them (encoder + zero + sag)
float64[] position_raw   # rad: the encoders' (LeRobot's calibration)
float64[] velocity       # rad/s: the servos' speed register
float64[] goal           # rad: commanded (true angles)
float64[] goal_velocity  # rad/s, rad/s2: the played motion's (0 holding)
float64[] goal_acceleration
float64[] duty           # Present Load: the PWM duty, -1..1 (est, VERIFY)
float64[] current        # A (signed; empty when the servos don't report it)
float64[] volts
uint8[] temp_c
uint8[] status           # each servo's error bits (Feetech status, 65)
float64[] gravity        # N m: the links' gravity on each joint
float64[] motor          # N m: what the servo's motor gives (duty or current model)
float64[] external       # N m: the outside torque (0 inside the friction band at rest)
float64[] band           # N m: the band's half width (0 moving)
float64[] filtered       # N m: the strain detector's low-passed outside torque
bool[] strained
float64[3] tool_force    # N in base_link: the outside torques as one force at the tool point
float64[3] gravity_vec   # m/s2 in base_link (the IMU's)
bool guard               # the motion playing is guarded (a strained joint fires the reflex)
