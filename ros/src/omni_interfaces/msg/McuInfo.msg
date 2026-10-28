# The MCU's INFO (firmware/lib/omni_fw/protocol.h InfoBody), 10 Hz: what firmware runs, the RC receiver, the drive
# motors' temperatures
std_msgs/Header header
uint16 protocol               # the firmware's PROTOCOL_VERSION (omni_mcu.protocol.PROTOCOL_VERSION on this side)
string build                  # the firmware's build id (firmware/fw_build_id.py: a hash of lib/omni_fw)
bool rc_ok                    # the RC receiver's link is up
bool rc_manual                # its switch asks for MANUAL (the MCU then takes the sticks)
float32[8] rc                 # channels, -1..1
uint8[4] motor_temp_c         # protocol 3: each DM-2325's winding (rotor) temperature, deg C
uint8[4] driver_temp_c        # and its driver's (MOSFETs)
