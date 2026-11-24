// The MCU firmware's top level (design.md 4, 14, 30): the safety authority under the robot computer.
//
// Every 1 ms tick: read the hardware (HwInputs), update odometry, decide the mode, run the base controller and the
// module, and write the hardware (HwOutputs). Between ticks, bytes from the robot computer arrive (rx) and leave (tx).
//
// Modes
//   DISARMED   drive motors disabled, flywheels off, feed holding. Boot state; also after the link has been lost and
//              the base has stopped, and after the e-stop is released (the robot computer must ask for AUTO again)
//   AUTO       the robot computer's CMDs drive the base and the module, inside this firmware's limits (the profile's caps;
//              the robot computer can only ask for less)
//   MANUAL     the sticks drive the base directly, without the robot computer's lidar guard: the deliberate fallback.
//              Normally (protocol 10) the RC's drive switch (ch 5) drives through the robot computer: STATE carries
//              the sticks, mode_manager's lidar guard shapes them, they come back as CMDs (AUTO). MANUAL only when the
//              computer can't drive (no link, the fault stop latched) and the switch has been turned off and on again
//              since. Leaving it (the switch, or the RC link lost) reports DISARMED at once but ramps the base to rest
//              with the motors still enabled, then disables them: never a cut at speed
//   ESTOP      the e-stop chain is open: motor power is already off in hardware; everything is reset
// Rules (design.md 14): every stop is ramped; no CMD for CMD_TIMEOUT -> ramped stop, flywheels off; a bumper ->
// ramped stop (only with HAS_BUMPERS: the arm robot has no switches yet); firing needs AUTO, the flywheels at speed and
// no bumper. A CMD must pass cmd_ok() (this protocol version, known enums, finite numbers in range) or it is dropped
// and counted (cmd_rejected): a dropped CMD is no heartbeat. The kid profile turns on at once and off only with the
// base at rest.
// The motor test (DRIVE_WHEELS, AUTO only): each wheel at the speed asked, capped (WHEEL_TEST_MAX) and ramped per
// wheel (at the CMD's a_brake up to WHEEL_TEST_ACCEL_MAX, else WHEEL_TEST_ACCEL); a lost link, a disarm or another drive mode ramps the wheels to rest first, then the base controller resumes.
// The fault stop: in AUTO, a low pack (under BATT_LOW for BATT_LOW_HOLD_US), a motor error or a silent motor -> a
// ramped stop, then DISARMED. A silent motor counts only when the base needs it: while the base controller drives
// (VEL_BODY, GOTO) or while that wheel is being turned (the motor test asking for it, a ramp): armed at rest
// (DRIVE_IDLE, the motor test between pushes) or turning the other wheels, a robot with motors missing tests the ones
// it has, and drives nothing else (STATE reports each silent motor as DM_NO_REPLY). It latches (FAULT_STOP) until the robot computer asks for DISARM (its mode ended) with the pack at
// BATT_LOW + BATT_RESUME or more; only then does a request for AUTO re-arm (which clears the motors' errors). MANUAL is
// not stopped: the RC operator (the drive switch off and on) keeps the sticks to move the robot off.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "base.h"
#include "ak_motor.h"
#include "dm_motor.h"
#include "fw_params_select.h"
#include "hw_io.h"
#include "module.h"
#include "protocol.h"

namespace fw {

enum McuMode : uint8_t { MODE_BOOT = 0, MODE_DISARMED = 1, MODE_AUTO = 2, MODE_MANUAL = 3, MODE_ESTOP = 4 };

enum Fault : uint16_t {
    F_LINK_LOST = 1 << 0,        // no CMD for CMD_TIMEOUT
    F_ESTOP = 1 << 1,
    F_BUMPER = 1 << 2,
    F_MOTOR_ERR = 1 << 3,        // a drive motor reports an error code (over-current, temperature, ...)
    F_MOTOR_SILENT = 1 << 4,     // a drive motor stopped replying
    F_LOW_BATT = 1 << 5,
    F_IMU_SILENT = 1 << 6,
    F_FLY_TELEM = 1 << 7,        // no eRPM telemetry while the flywheels run
    F_FAULT_STOP = 1 << 8,       // the fault stop is latched (LOW_BATT, MOTOR_ERR or MOTOR_SILENT in AUTO)
};

constexpr float BATT_RESUME = 1.2f;              // V over BATT_LOW to re-arm (0.2 V a cell on 6S): a resting low pack
                                                 // recovers less (est)
constexpr uint64_t BATT_LOW_HOLD_US = 1000000;   // under BATT_LOW this long: a low pack, not a current spike's sag (est)
constexpr uint64_t ARM_GRACE_US = 200000;        // after arming, the motors' enable and error clear before they count
// the motor test (DRIVE_WHEELS): each wheel's own speed, for a person at the console with the wheels up. Capped here
// whatever the host asks (and by CMD speed_cap / WHEEL_RADIUS), ramped per wheel; leaving it ramps every wheel to rest
// before the base controller takes over again
constexpr float WHEEL_TEST_MAX = prm::MAX_WHEEL_SPEED;  // rad/s: the motor's own limit, so it is tested at full speed
                                                        // (the arm robot's AK40-10: 37.2 rad/s, 355 rpm: 0.95 of
                                                        // its no-load speed at 24 V, measured Nov 13)
constexpr float WHEEL_TEST_ACCEL = 20.0f;        // rad/s^2 per wheel (est): rest to the arm robot's top in 2.2 s;
                                                 // the default, and every ramp to rest on leaving the test
// protocol 9: the host may ask for another ramp (CMD a_brake), up to this: a step for a bare wheel (its motor's
// current limit sets the real rate: ~3000 rad/s^2 on an AK40-10 with a 140 mm wheel, est), to load the wheel's
// inertia on the bench and compare with the sim
constexpr float WHEEL_TEST_ACCEL_MAX = 2000.0f;

enum BasePhase : uint8_t { PH_TRACK = 0, PH_COAST = 1, PH_BRAKE = 2, PH_HOLD = 3 };

class Firmware {
public:
    void init();
    void tick(const FwHwInputs& in, FwHwOutputs& out);
    // bytes from the robot computer; t_rx_us = when they were read (MCU clock), the CMD's arrival for the time sync's echo
    // (0 = unknown: the next tick's time)
    void rx(const uint8_t* data, size_t n, uint64_t t_rx_us = 0);
    size_t tx(uint8_t* out, size_t cap);

    // read-only views for tests and the sim's logs
    McuMode mode() const { return mode_; }
    uint16_t faults() const { return faults_; }
    const Odometry& odom() const { return odom_; }
    const BaseController& ctl() const { return ctl_; }
    const CmdBody& cmd() const { return cmd_; }

private:
    void handle_can(const FwHwInputs& in, float dt);
    void run_base(float dt, bool auto_ok, const FwHwInputs& in, float* wheel_cmd);
    void run_module(float dt, bool auto_ok, const FwHwInputs& in, FwHwOutputs& out);
    void drive_motors(bool enable, const float* wheel_cmd, FwHwOutputs& out);
    void send_state(const FwHwInputs& in);
    void send_imu(const FwImuSample& s);
    void send_info(const FwHwInputs& in);
    void send_time(uint64_t t, uint8_t kind, uint32_t seq, uint32_t value);
    void time_refs(const FwHwInputs& in, FwHwOutputs& out);
    void push_tx(const uint8_t* d, size_t n);
    Limits limits() const;
    static bool cmd_ok(const CmdBody& c);

    McuMode mode_ = MODE_BOOT;
    uint16_t faults_ = 0;
    uint64_t t_us_ = 0;
    bool first_ = true;

    // link
    FrameDecoder dec_;
    CmdBody cmd_{};
    bool have_cmd_ = false;
    bool cmd_new_ = false;
    uint64_t t_cmd_us_ = 0;
    uint64_t t_cmd_rx_us_ = 0;   // when the last CMD's bytes were read (0: not stamped)
    uint64_t echo_host_ = 0, echo_rx_us_ = 0;
    uint16_t cmd_count_ = 0;
    uint8_t seq_state_ = 0, seq_imu_ = 0, seq_info_ = 0, seq_time_ = 0;
    uint16_t cmd_rejected_ = 0;             // CMDs dropped by cmd_ok()
    uint16_t tick_late_ = 0;                // ticks more than 1.5 ms after the last
    uint32_t tick_max_us_ = 0;              // the longest gap between ticks since the last STATE
    uint8_t profile_ = PROFILE_ADULT;       // the profile in force (kid on at once, off only at rest)

    // time references (design.md 10): the next scheduled edges and their counters
    uint64_t sync_next_ = 0, cam_next_ = 0, cam_first_ = 0;
    uint32_t sync_seq_ = 0, cam_seq_ = 0, cam_k_ = 0;
    uint64_t cam_last_fired_ = 0;
    uint64_t t_state_next_ = 0, t_info_next_ = 0;
    uint8_t txbuf_[4096];
    size_t tx_head_ = 0, tx_tail_ = 0;

    // drive motors
    DmFeedback fb_[prm::N_WHEELS]{};
    bool fb_valid_[prm::N_WHEELS]{};
    uint64_t t_fb_us_[prm::N_WHEELS]{};      // the tick that got each motor's last reply (the silent-motor fault)
    uint64_t t_enable_us_[prm::N_WHEELS]{};
    float wheel_cmd_[prm::N_WHEELS]{};
    bool wheel_enabled_ = false;               // CubeMars: armed (velocity commands) or coasting
    uint32_t ak_next_ = 0;                     // CubeMars: the motor whose turn it is (one frame per tick)
    uint8_t ak_err_[prm::N_WHEELS]{};          // ... each one's last status error (AkErr)
    float wheel_test_[prm::N_WHEELS]{};        // the motor test's ramped wheel speeds (DRIVE_WHEELS), rad/s
    float wheel_cur_[prm::N_WHEELS]{};         // each motor's reported current, A (AK40-10 status; STATE wheel_cur)
    bool wheel_test_on_ = false;               // ... running, or ramping its wheels down after it ended
    void wheel_test_reset();
    bool wheel_silent(int i) const;            // no reply from wheel i's motor for DM_FEEDBACK_TIMEOUT
    bool silent_matters(int i) const;          // wheel i silent would stop the base: driving, or turning that wheel

    // base
    BaseController ctl_;
    Odometry odom_;
    float hold_yaw_ = 0.0f;
    bool hold_yaw_set_ = false;
    BasePhase phase_ = PH_TRACK;
    uint64_t brake_key_ = 0;
    Pose hold_pose_;
    uint64_t t_stopped_us_ = 0;
    uint64_t t_armed_us_ = 0;
    uint64_t t_batt_low_us_ = 0;     // since when the pack reads under BATT_LOW (0: it doesn't)
    bool fault_stop_ = false;
    bool ramp_out_ = false;          // MANUAL just ended: ramping the base to rest before the motors are disabled
    bool rc_rearm_ = false;          // the drive switch was off since the computer stopped driving (MANUAL may start)
    bool bumper_ = false;            // a bumper is pressed (always false without HAS_BUMPERS)

    // module
    FlywheelLoop fly_[2];
    uint64_t t_fly_telem_us_ = 0;
    Feed feed_;
    uint16_t fire_seq_done_ = 0;
    uint16_t fire_rejected_ = 0;
    bool fire_init_ = false;
    float fly_target_ = 0.0f;    // rad/s
    bool fly_ready_ = false;
    float tilt_cmd_ = 0.0f;
    float tilt_meas_ = 0.0f;
    float feed_speed_meas_[2]{};
    uint64_t t_imu_us_ = 0;
    float last_v_bus_ = 0.0f;
};

}  // namespace fw
