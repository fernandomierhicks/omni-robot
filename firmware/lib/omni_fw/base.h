// The base: X4 omni kinematics, the jerk-limited, direction-aware velocity controller and the braking-curve goto
// (a port of the sim's omni_control.BaseController, design.md 3 and 20), and wheel + IMU odometry (design.md 7, 27).
#pragma once

#include "fw_params.h"
#include "fw_types.h"

namespace fw {

struct Pose {
    float x = 0.0f, y = 0.0f, yaw = 0.0f;
};

// the limits a mode allows (fw_params.h: ADULT, KID); the Jetson may only ask for less (speed_cap)
struct Limits {
    float max_speed, max_accel, max_jerk, max_yaw_rate, max_yaw_accel, brake_margin, accel_scale, fly_rpm_max;
};

class BaseController {
public:
    Vec2 v_cmd, a_cmd;          // odom frame
    float w_cmd = 0.0f;
    float max_speed = 0.0f, max_accel = 0.0f, max_jerk = 0.0f, max_yaw_rate = 0.0f, max_yaw_accel = 0.0f;
    float brake_margin = 0.8f, accel_scale = 1.0f;

    void reset();
    void set_limits(const Limits& l, float speed_cap);
    float accel_limit(Vec2 u_world, float yaw) const;
    float worst_accel() const;
    float top_speed(Vec2 u_world, float yaw) const;
    // wheel speed commands (rad/s) into out[N_WHEELS]
    void velocity_step(float dt, Vec2 v_des, float w_des, float yaw, float a_cap, float* out);
    void goto_step(float dt, const Pose& pose, const Pose& target, Vec2 v_ff, float v_end, float* out);
};

void wheel_speeds(float vx, float vy, float wz, float* out);       // body twist -> wheels (J)

class Odometry {
public:
    Pose pose;                  // odom frame: where the robot stood at boot, x forward
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;     // body twist from the wheels
    float slip = 0.0f;          // m/s: how far the four wheels disagree with any rigid motion
    float roll = 0.0f, pitch = 0.0f, gyro_z = 0.0f;
    bool imu_ok = false;

    void reset();
    // wheel angles from the motors' feedback (rad, wrapping at +/-p_max); valid[i] = a new reply this tick.
    // t_us[i] (design.md 10): when motor i measured it, MCU clock; the speed is the angle step over the real interval
    // between two samples, not over the ticks (null, or an unusable interval: the ticks')
    void on_wheels(float dt, const float* pos, const bool* valid, float p_max, const uint64_t* t_us = nullptr);
    void on_imu(const float* quat_wxyz, const float* gyro);
    void integrate(float dt);

private:
    float last_pos_[prm::N_WHEELS] = {};
    float last_dt_[prm::N_WHEELS] = {};
    uint64_t last_t_[prm::N_WHEELS] = {};
    bool have_pos_[prm::N_WHEELS] = {};
    float w_[prm::N_WHEELS] = {};
    float imu_yaw0_ = 0.0f, imu_yaw_ = 0.0f, imu_unwrap_ = 0.0f, imu_last_ = 0.0f;
    bool imu_init_ = false;
};

}  // namespace fw
