// The hardware boundary of the firmware core (design.md 30). Every millisecond the board layer (src/main.cpp on the
// Teensy, sil/sil_api.cpp in the sim) fills HwInputs from the peripherals, calls Firmware::tick(), and drives the
// peripherals from HwOutputs. Nothing else crosses: the core never touches a register, so the same core runs on the
// Teensy and inside the MuJoCo sim (SIL).
//
// Plain C structs with fixed sizes: sil/omni_fw_sil.py mirrors them with ctypes (and checks sizeof at load).
#pragma once

#include <stdint.h>

#define FW_CAN_MAX 16

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;        // 11-bit standard id
    uint8_t len;
    uint8_t data[8];
    uint8_t _pad[3];
} FwCanFrame;

typedef struct {
    uint64_t t_us;                 // MCU clock, microseconds since boot
    // CAN1 (drive motors, 1 Mbit/s): frames received since the last tick
    uint32_t n_can_rx;
    FwCanFrame can_rx[FW_CAN_MAX];
    // BNO085 over SPI (400 Hz): a new report since the last tick?
    uint32_t imu_new;
    float imu_quat[4];             // game rotation vector, w x y z (yaw starts at 0)
    float imu_gyro[3];             // rad/s, calibrated
    float imu_accel[3];            // m/s^2, with gravity
    // flywheel ESC (bidirectional DShot): a new eRPM frame per channel?
    uint32_t fly_new[2];
    float fly_erpm[2];             // electrical rpm
    // feed motor controller (RoboClaw packet serial): measured wheel speeds
    uint32_t feed_new;
    float feed_speed[2];           // rad/s at the feed wheels
    // tilt servo (STS3250 serial bus): present position
    uint32_t tilt_new;
    float tilt_pos;                // rad, module tilt (+ = toward the thrower)
    // digital inputs
    uint32_t beam_catch;           // 1 = the catch beam is blocked (a ball in the pocket)
    uint32_t beam_feed_exit;       // 1 = the feed-exit beam is blocked
    uint32_t estop;                // 1 = the e-stop chain is open (motor power is off)
    uint32_t bumper;               // 1 = a bumper switch is pressed
    float v_bus;                   // V, battery (ADC through a divider)
    // RC receiver (ELRS / CRSF): link up, channels -1..1
    uint32_t rc_ok;
    float rc[8];
} FwHwInputs;

typedef struct {
    // CAN1 frames to send this tick
    uint32_t n_can_tx;
    FwCanFrame can_tx[FW_CAN_MAX];
    // flywheel ESC: throttle 0..1 (bidirectional DShot 48..2047); 0 = DShot 0 (disarmed, the wheel coasts)
    float fly_throttle[2];
    uint32_t fly_brake;            // 1 = with a zero throttle, brake (AM32 "brake on stop") instead of coasting
    // feed motor controller: wheel speed command, rad/s. 0 = hold the wheel where it stopped (position hold)
    float feed_speed_cmd[2];
    // tilt servo: goal position, rad
    float tilt_cmd;
    uint32_t tilt_enable;
    uint32_t buzzer;               // 1 = sound (start-of-motion chirp, fire warning)
    uint32_t led;                  // status LED pattern id
    // not hardware: the controller's internal state, for the sim's logs and tests
    float dbg_v_cmd[2];            // commanded velocity, odom frame
    float dbg_w_cmd;
    float dbg_wheel_cmd[4];        // rad/s sent to the drive motors
} FwHwOutputs;

#ifdef __cplusplus
}
#endif
