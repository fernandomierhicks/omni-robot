// Which way the robot faces while Nav2 drives it (the console's "Driving" choice). C++ port of
// omni_bringup/heading.py, whose notes say why: agnostic passes Nav2's turn rate through, forward turns the face
// toward the direction of travel, spinning spins at spin_rate; Nav2's velocity is kept in the odom frame and turned
// back into the body frame at our own rate with the yaw predicted for the middle of the firmware's hold.
#pragma once

#include <cmath>
#include <optional>
#include <string>

namespace omni {

constexpr double kPi = 3.14159265358979323846;

struct HeadingConfig {
    std::string mode = "agnostic";
    double spin_rate = 120.0 * (kPi / 180.0);  // rad/s, spinning (negative: clockwise)
    double turn_max = 90.0 * (kPi / 180.0);    // rad/s, forward's top turn rate
    double yaw_accel = 3.2;                    // rad/s^2, as the velocity smoother's
    double gain = 3.0;                         // 1/s, forward: turn rate per rad of heading error
    double lead = 0.03;                        // s from here to the firmware acting on it
    double period = 0.02;                      // s between our commands
    double handback_lo = 0.4, handback_hi = 0.9;   // m of path left: Nav2's turn rate at the first, ours from the second
    double slow_lo = 0.08, slow_hi = 0.2;      // m/s, forward: Nav2's turn rate at the first, ours from the second
    double stale = 0.3;                        // s without Nav2's command: stop publishing
    double plan_stale = 3.0;                   // s without a plan: the goal distance is unknown
};

bool heading_mode_ok(const std::string& m);

struct Cmd3 { double vx, vy, wz; };

class HeadingShaper {
public:
    explicit HeadingShaper(HeadingConfig c = {}) : cfg(std::move(c)) {}

    void on_odom(double t, double x, double y, double yaw, double wz);
    void on_plan(double t, double length);
    // Nav2's command (body frame): what to publish now
    Cmd3 on_cmd(double t, double vx, double vy, double wz);
    bool active(double t) const;
    std::optional<double> goal_distance(double t) const;
    // the shaped command (body frame) at time t; none once Nav2's command is stale
    std::optional<Cmd3> step(double t);

    HeadingConfig cfg;
    double t_out = NAN;          // when the last turn rate went out

private:
    double yaw_at(double t) const;
    double target_rate(double t) const;
    bool has_odom_ = false, has_cmd_ = false, has_plan_ = false, has_xy_ = false;
    double odom_t_ = 0, odom_yaw_ = 0, odom_wz_ = 0;
    double travelled_ = 0, x_ = 0, y_ = 0;
    double plan_t_ = 0, plan_len_ = 0, plan_travelled_ = 0;
    double vox_ = 0, voy_ = 0, w_nav_ = 0, t_cmd_ = 0, w_out_ = 0;
};

}  // namespace omni
