#include "omni_cpp/robot_config.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace omni {

double np_interp(double x, const std::vector<double>& xp, const std::vector<double>& fp) {
    size_t n = xp.size();
    if (n == 0) throw std::invalid_argument("np_interp: empty table");
    if (x > xp[n - 1]) return fp[n - 1];
    if (x < xp[0]) return fp[0];
    if (n == 1 || x == xp[n - 1]) return fp[n - 1];
    size_t j = (size_t)(std::upper_bound(xp.begin(), xp.end(), x) - xp.begin()) - 1;     // xp[j] <= x < xp[j + 1]
    if (xp[j] == x) return fp[j];
    double slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
    double r = slope * (x - xp[j]) + fp[j];
    if (std::isnan(r)) r = slope * (x - xp[j + 1]) + fp[j + 1];
    return r;
}

RobotConfig::RobotConfig(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("robot config: can't read " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    raw = Json::parse(ss.str());
    const Json &b = raw["base"], &mo = raw["module"], &la = raw["launcher"], &ba = raw["ball"];
    base_link_height = b["base_link_height"].num();
    robot_radius = b["robot_radius"].num();
    J = b["J"].rows();
    max_wheel_speed = b["max_wheel_speed"].num();
    max_speed = b["max_speed"].num();
    brake_margin = b["brake_margin"].num();
    accel_table = b["accel_table"].rows();
    max_accel = b["max_accel"].num();
    kid_max_speed = b["kid"]["max_speed"].num();
    kid_accel_scale = b["kid"]["accel_scale"].num();
    auto lim = mo["tilt_limits"].nums();
    tilt_lo = lim.at(0), tilt_hi = lim.at(1);
    for (double d : mo["tilt_deg"].nums()) tilt_rad.push_back(d * (kPiC / 180.0));         // np.radians
    for (auto& r : mo["rim"].rows()) rim_x.push_back(r.at(0)), rim_z.push_back(r.at(1));
    for (auto& r : mo["exit"].rows()) exit_x.push_back(r.at(0)), exit_z.push_back(r.at(1));
    funnel_rim_radius = mo["funnel_rim_radius"].num();
    launch_angle = mo["launch_angle_deg"].num() * (kPiC / 180.0);                          // math.radians
    cal_tilts = la["tilt_rad"].nums();
    cal_w = la["w"].rows();
    cal_v = la["v"].rows();
    cal_de = la["d_elev"].rows();
    ball_mass = ba["mass"].num();
    ball_radius = ba["radius"].num();
    ball_cd = ba["drag_cd"].num();
    air_density = ba["air_density"].num();
    air_viscosity = ba["air_viscosity"].num();
    const Json& cams = raw["cameras"];
    for (size_t k = 0; k < cams.size(); ++k) {
        const Json& c = cams[k];
        CameraConfig cc;
        cc.name = c.get("name").is_null() ? "" : c["name"].str();
        auto p = c["pos"].nums();
        cc.pos = {p.at(0), p.at(1), p.at(2)};
        auto R = c["R"].rows();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) cc.R[3 * i + j] = R.at(i).at(j);
        cc.fovy_deg = c["fovy_deg"].num();
        cc.width = (int)c["width"].integer();
        cc.height = (int)c["height"].integer();
        cameras.push_back(cc);
    }
}

std::vector<double> RobotConfig::rim_center(double tilt) const {
    return {np_interp(tilt, tilt_rad, rim_x), np_interp(tilt, tilt_rad, rim_z)};
}

std::vector<double> RobotConfig::exit_point(double tilt) const {
    return {np_interp(tilt, tilt_rad, exit_x), np_interp(tilt, tilt_rad, exit_z)};
}

template <typename F>
double RobotConfig::blend(double tilt, F fn) const {
    size_t n = cal_tilts.size();
    if (n == 1) return fn(0);
    long ss = (long)(std::lower_bound(cal_tilts.begin(), cal_tilts.end(), tilt) - cal_tilts.begin());  // searchsorted
    long k = std::min<long>(std::max<long>(ss - 1, 0), (long)n - 2);
    double f = std::min(1.0, std::max(0.0, (tilt - cal_tilts[k]) / (cal_tilts[k + 1] - cal_tilts[k])));
    return (1 - f) * fn(k) + f * fn(k + 1);
}

double RobotConfig::speed_for(double v_exit, double tilt) const {
    return blend(tilt, [&](long k) { return np_interp(v_exit, cal_v[k], cal_w[k]); });
}

double RobotConfig::elevation_at(double v_exit, double tilt) const {
    return launch_angle - tilt + blend(tilt, [&](long k) { return np_interp(v_exit, cal_v[k], cal_de[k]); });
}

double RobotConfig::tilt_for(double v_exit, double elevation) const {
    double tilt = launch_angle - elevation;
    for (int i = 0; i < 3; ++i)
        tilt = launch_angle - elevation + blend(tilt, [&](long k) { return np_interp(v_exit, cal_v[k], cal_de[k]); });
    return tilt;
}

double RobotConfig::plan_accel(bool kid) const {
    double a = !accel_table.empty() ? *std::min_element(accel_table[1].begin(), accel_table[1].end()) : max_accel;
    return a * (kid ? kid_accel_scale : 1.0);
}

double RobotConfig::top_speed(double ux, double uy, double yaw, bool kid) const {
    double c = std::cos(yaw), s = std::sin(yaw);
    double bx = c * ux + s * uy, by = -s * ux + c * uy;
    double n = std::sqrt(bx * bx + by * by);
    double cap = kid ? kid_max_speed : max_speed;
    if (n < 1e-9) return cap;
    double wx = bx / n, wy = by / n, k = 0;
    for (auto& row : J) k = std::max(k, std::fabs(row[0] * wx + row[1] * wy));
    return std::min(cap, max_wheel_speed / k);
}

}  // namespace omni
