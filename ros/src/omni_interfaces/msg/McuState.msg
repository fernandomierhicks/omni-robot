# The MCU's STATE (firmware/lib/omni_fw/protocol.h StateBody), 200 Hz, stamped on the Jetson's clock (time sync)
std_msgs/Header header        # frame_id "odom"
uint64 t_mcu_us               # the MCU's own clock
string mode                   # BOOT DISARMED AUTO MANUAL ESTOP
string profile                # adult / kid
uint8 drive_mode              # 0 idle, 1 velocity, 2 goto, 3 coasting / braking after a catch
string feed_state             # IDLE SEAT RUN DONE
string[] faults
uint8[4] dm_err               # drive motor error codes (1 = enabled)
float32 x                     # odometry (wheels + BNO085 heading), odom frame
float32 y
float32 yaw
float32 vx                    # body twist
float32 vy
float32 wz
float32 slip                  # m/s, wheel disagreement
float32 roll                  # BNO085 game rotation vector
float32 pitch
float32 v_cmd_x               # the controller's command, odom frame
float32 v_cmd_y
float32 w_cmd
float32[2] fly_rpm
float32 fly_target_rpm
bool fly_ready
float32 tilt                  # rad, measured
float32 tilt_cmd
bool ball_held                # the catch beam
bool feed_exit_beam
uint16 fire_seq_done
uint16 fire_rejected
float32 v_bus
float64 sync_rtt_ms           # the link's best round trip
