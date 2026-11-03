// Moving-catch intercept planner with the clear-path gate (design.md 30): C++ port of omni_catch/intercept.py (its
// notes explain the planner and the two versions it beat). test/test_cores.py checks the two.
#pragma once

#include <vector>

namespace omni {

struct Person {
    double x, y, r;
};

struct SafetyZone {
    double half_x = 2.5, half_y = 2.0;
    std::vector<Person> people;
    double robot_radius = 0.21, margin = 0.20;
    double cx = 0.0, cy = 0.0;
    bool clear_point(double qx, double qy) const;
};

double min_time(double u0, double d, double v_f, double a, double v_max);
double min_arrival_speed(double u0, double d, double T, double a, double v_max);
bool path_clear(const SafetyZone& z, const double p0[2], const double p_c[2], const double v_f[2], double coast,
                double a_ball, double step = 0.05);

struct InterceptChoice {
    double v_f;
    double dx, dy;              // unit direction of travel at the meeting point
    bool feasible, gated;
};

InterceptChoice choose_intercept(const double p0[2], const double v0[2], const double p_c[2], double T, double a,
                                 double v_max, const SafetyZone& zone, double coast, double a_ball, bool moving = true);

}  // namespace omni
