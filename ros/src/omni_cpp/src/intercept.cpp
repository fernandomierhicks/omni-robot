#include "omni_cpp/intercept.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace omni {

bool SafetyZone::clear_point(double qx, double qy) const {
    double lim = robot_radius + margin;
    if (std::fabs(qx - cx) > half_x - lim || std::fabs(qy - cy) > half_y - lim) return false;
    for (auto& p : people)
        if (!(std::hypot(qx - p.x, qy - p.y) >= p.r + lim)) return false;
    return true;
}

double min_time(double u0, double d, double v_f, double a, double v_max) {
    if (d <= 0) return 0.0;
    double vp = std::sqrt(a * d + (u0 * u0 + v_f * v_f) / 2);
    if (vp < std::max(u0, v_f))                 // can't shed (or gain) the speed in the distance: one-sided ramp
        return u0 + v_f > 1e-9 ? 2 * d / (u0 + v_f) : std::numeric_limits<double>::infinity();
    if (vp <= v_max) return (vp - u0) / a + (vp - v_f) / a;
    double d_ramp = (2 * std::pow(v_max, 2.0) - std::pow(u0, 2.0) - std::pow(v_f, 2.0)) / (2 * a);
    return (v_max - u0) / a + (v_max - v_f) / a + (d - d_ramp) / v_max;
}

double min_arrival_speed(double u0, double d, double T, double a, double v_max) {
    if (min_time(u0, d, 0.0, a, v_max) <= T) return 0.0;
    if (min_time(u0, d, v_max, a, v_max) > T) return v_max;
    double lo = 0.0, hi = v_max;
    for (int k = 0; k < 30; ++k) {
        double mid = 0.5 * (lo + hi);
        if (min_time(u0, d, mid, a, v_max) <= T) hi = mid;
        else lo = mid;
    }
    return hi;
}

bool path_clear(const SafetyZone& z, const double p0[2], const double p_c[2], const double v_f[2], double coast,
                double a_ball, double step) {
    double sp = std::sqrt(v_f[0] * v_f[0] + v_f[1] * v_f[1]);
    double q_end[2] = {p_c[0], p_c[1]};
    if (!(sp < 1e-9)) {
        double k = coast + sp / (2 * a_ball);
        q_end[0] = p_c[0] + v_f[0] * k, q_end[1] = p_c[1] + v_f[1] * k;
    }
    const double* seg[2][2] = {{p0, p_c}, {p_c, q_end}};
    for (auto& s : seg) {
        const double *a = s[0], *b = s[1];
        double dx = b[0] - a[0], dy = b[1] - a[1];
        int n = std::max(2, (int)std::ceil(std::sqrt(dx * dx + dy * dy) / step) + 1);
        for (int k = 0; k < n; ++k)
            if (!z.clear_point(a[0] + dx * k / (n - 1), a[1] + dy * k / (n - 1))) return false;
    }
    return true;
}

InterceptChoice choose_intercept(const double p0[2], const double v0[2], const double p_c[2], double T, double a,
                                 double v_max, const SafetyZone& zone, double coast, double a_ball, bool moving) {
    double dx = p_c[0] - p0[0], dy = p_c[1] - p0[1];
    double d = std::sqrt(dx * dx + dy * dy);
    if (d < 0.01 || T <= 0.02) return {0.0, 1.0, 0.0, true, false};
    double hx = dx / d, hy = dy / d;
    double u0 = std::max(0.0, v0[0] * hx + v0[1] * hy);
    double v_f = min_arrival_speed(u0, d, T, a, v_max);
    bool feasible = min_time(u0, d, v_f, a, v_max) <= T;
    if (v_f <= 0.0) return {0.0, hx, hy, feasible, false};
    if (!moving) return {0.0, hx, hy, false, false};
    double vv[2] = {v_f * hx, v_f * hy};
    if (path_clear(zone, p0, p_c, vv, coast, a_ball)) return {v_f, hx, hy, feasible, false};
    double lo = 0.0, hi = v_f;                         // fastest end speed the gate allows
    for (int k = 0; k < 20; ++k) {
        double mid = 0.5 * (lo + hi);
        double vm[2] = {mid * hx, mid * hy};
        if (path_clear(zone, p0, p_c, vm, coast, a_ball)) lo = mid;
        else hi = mid;
    }
    return {lo, hx, hy, false, true};
}

}  // namespace omni
