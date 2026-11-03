// The robot as the catch code knows it (design.md 30): C++ port of omni_catch/robot_config.py, reading the same
// config/robot_v11.json (written by simulation/v11 sil/robot_export.py). test/test_cores.py checks the two.
#pragma once

#include <string>
#include <vector>

#include "omni_cpp/catch_math.hpp"
#include "omni_cpp/json_value.hpp"

namespace omni {

// np.interp(x, xp, fp): the ends held, linear between
double np_interp(double x, const std::vector<double>& xp, const std::vector<double>& fp);

struct CameraConfig {
    std::string name;
    V3 pos;
    M3 R;                       // columns: the camera's axes in base_link (it looks along -z)
    double fovy_deg = 0;
    int width = 0, height = 0;
};

class RobotConfig {
public:
    RobotConfig() = default;
    explicit RobotConfig(const std::string& path);      // throws on a missing or malformed file

    std::vector<double> rim_center(double tilt) const;   // funnel rim center (x, z)
    std::vector<double> exit_point(double tilt) const;   // the launch exit (x, z)
    double speed_for(double v_exit, double tilt) const;  // flywheel rad/s for an exit speed
    double elevation_at(double v_exit, double tilt) const;
    double tilt_for(double v_exit, double elevation) const;
    double plan_accel(bool kid = false) const;
    double top_speed(double ux, double uy, double yaw, bool kid = false) const;

    Json raw;
    double base_link_height = 0, robot_radius = 0, max_wheel_speed = 0, max_speed = 0, brake_margin = 0,
           max_accel = 0;
    std::vector<std::vector<double>> J, accel_table;
    double kid_max_speed = 0, kid_accel_scale = 1;
    double tilt_lo = 0, tilt_hi = 0;
    std::vector<double> tilt_rad, rim_x, rim_z, exit_x, exit_z;
    double funnel_rim_radius = 0, launch_angle = 0;
    std::vector<double> cal_tilts;
    std::vector<std::vector<double>> cal_w, cal_v, cal_de;
    double ball_mass = 0, ball_radius = 0, ball_cd = 0, air_density = 0, air_viscosity = 0;
    std::vector<CameraConfig> cameras;

private:
    template <typename F>
    double blend(double tilt, F fn) const;
};

}  // namespace omni
