// Ball flight with air drag (design.md 30): C++ port of omni_catch/flight.py (its notes say why each part exists);
// test/test_cores.py checks the two against each other. RK4 on plain doubles, as the Python does.
#pragma once

#include <optional>

#include "omni_cpp/catch_math.hpp"

namespace omni {

struct Crossing {
    V3 pos;                 // where the ball crosses the plane going down
    V3 vel;
    double t;               // s from now
};

class BallFlight {
public:
    BallFlight() = default;
    BallFlight(double mass, double radius, double drag_cd, double air_density = 1.204, double air_viscosity = 1.8e-5,
               double gravity = 9.81);

    V3 accel(const V3& v) const;
    std::optional<Crossing> crossing_state(const V3& pos, const V3& vel, double z_plane, double dt = 0.004,
                                           double t_max = 4.0) const;
    double apex(const V3& pos, const V3& vel, double dt = 0.004) const;
    // where the ball last rose through z_release, integrating back: (xyz, seconds before now)
    std::optional<std::pair<V3, double>> release_point(const V3& pos, const V3& vel, double z_release,
                                                       double dt = 0.004, double t_max = 3.0) const;
    double solve_launch_speed(const V3& exit_pos, double ux, double uy, double elevation, const V3& target) const;

    double k_quad = 0, k_lin = 0, gz = 9.81;

private:
    void acc(double vx, double vy, double vz, double& ax, double& ay, double& az) const {
        double k = k_lin + k_quad * std::sqrt(vx * vx + vy * vy + vz * vz);
        ax = -k * vx, ay = -k * vy, az = -gz - k * vz;
    }
    double horizontal_range(const V3& pos, const V3& vel, double z_plane) const;
};

}  // namespace omni
