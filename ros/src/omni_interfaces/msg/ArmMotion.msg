# A command to the arm driver (omni_cpp's arm_driver, design.md 11): the one writer is the arm brain (omni_arm's
# arm_node). The driver streams the goals to the servos at its own rate; it plays one motion at a time (the latest wins).
uint8 MOVE=0       # play the waypoints
uint8 HOLD=1       # torque on, hold where the arm is now (the motion ends)
uint8 RELAX=2      # torque off: the arm is limp (the SO-101 has no brakes)
uint8 QUINTIC=0    # each segment a quintic 0..1 (zero speed and acceleration at both ends): omni_arm/motion.py's Motion
uint8 LINEAR=1     # straight lines between the waypoints (a FollowJointTrajectory goal)
uint8 GUARD_ON=0
uint8 GUARD_OFF=1

std_msgs/Header header
uint8 command
uint8 interpolation
uint32 id                # echoed in /arm/hw's motion_id once the driver has taken the command
float64[] times          # one per waypoint, absolute seconds on the ROS clock (the first is the start)
float64[] waypoints      # n x 6 joint angles in rad, row-major, in /arm/hw's joint order; the first row is where the
                         # arm is: the driver plays from its own goal there (so a move that starts while another plays
                         # carries on from the goal with no jump)
uint8 guard              # GUARD_ON (0): a strained joint stops the motion and backs it off (the driver's reflex),
                         # and holding still the driver trims out the servos' droop; GUARD_OFF: neither, the servos raw
                         # (the arm load calibration's own holds and sweeps)
