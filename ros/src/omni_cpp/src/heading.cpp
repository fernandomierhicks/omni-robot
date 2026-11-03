#include "omni_cpp/heading.hpp"

#include <algorithm>

namespace omni {

namespace {

double wrap(double a) {                        // Python's (a + pi) % (2 pi) - pi
    double m = std::fmod(a + kPi, 2.0 * kPi);
    if (m < 0) m += 2.0 * kPi;
    return m - kPi;
}

double ramp(double x, double lo, double hi) { return std::min(1.0, std::max(0.0, (x - lo) / (hi - lo))); }

}  // namespace

bool heading_mode_ok(const std::string& m) { return m == "agnostic" || m == "forward" || m == "spinning"; }

void HeadingShaper::on_odom(double t, double x, double y, double yaw, double wz) {
    if (has_xy_) travelled_ += std::hypot(x - x_, y - y_);
    x_ = x, y_ = y, has_xy_ = true;
    odom_t_ = t, odom_yaw_ = yaw, odom_wz_ = wz, has_odom_ = true;
}

void HeadingShaper::on_plan(double t, double length) {
    plan_t_ = t, plan_len_ = length, plan_travelled_ = travelled_, has_plan_ = true;
}

Cmd3 HeadingShaper::on_cmd(double t, double vx, double vy, double wz) {
    if (cfg.mode == "agnostic" || !has_odom_) {
        has_cmd_ = false;
        return {vx, vy, wz};
    }
    if (!active(t)) w_out_ = wz, t_out = t;    // a new drive: take over from Nav2's rate, then ramp
    double yaw = yaw_at(t), c = std::cos(yaw), s = std::sin(yaw);
    vox_ = c * vx - s * vy, voy_ = s * vx + c * vy;
    w_nav_ = wz;
    t_cmd_ = t, has_cmd_ = true;
    return *step(t);
}

double HeadingShaper::yaw_at(double t) const { return odom_yaw_ + odom_wz_ * (t - odom_t_); }

std::optional<double> HeadingShaper::goal_distance(double t) const {
    if (!has_plan_ || t - plan_t_ > cfg.plan_stale) return std::nullopt;
    return std::max(0.0, plan_len_ - (travelled_ - plan_travelled_));
}

bool HeadingShaper::active(double t) const {
    return cfg.mode != "agnostic" && has_cmd_ && t - t_cmd_ <= cfg.stale;
}

double HeadingShaper::target_rate(double t) const {
    auto d = goal_distance(t);
    double k = d ? ramp(*d, cfg.handback_lo, cfg.handback_hi) : 0.0;
    double mine;
    if (cfg.mode == "forward") {
        k *= ramp(std::hypot(vox_, voy_), cfg.slow_lo, cfg.slow_hi);
        if (k <= 0.0) return w_nav_;
        double e = wrap(std::atan2(voy_, vox_) - yaw_at(t));
        // proportional, and never faster than it can stop at the heading with yaw_accel
        double w = std::min({cfg.turn_max, cfg.gain * std::fabs(e), std::sqrt(2.0 * cfg.yaw_accel * std::fabs(e))});
        mine = std::copysign(w, e);
    } else {
        mine = cfg.spin_rate;
    }
    return w_nav_ + k * (mine - w_nav_);
}

std::optional<Cmd3> HeadingShaper::step(double t) {
    if (!active(t)) return std::nullopt;
    double w_t = target_rate(t);
    double a = cfg.yaw_accel * std::min(0.1, std::max(0.0, t - t_out));
    w_out_ += std::min(a, std::max(-a, w_t - w_out_));
    t_out = t;
    double yaw = yaw_at(t) + w_out_ * (cfg.lead + 0.5 * cfg.period), c = std::cos(yaw), s = std::sin(yaw);
    return Cmd3{c * vox_ + s * voy_, -s * vox_ + c * voy_, w_out_};
}

}  // namespace omni
