#include "omni_cpp/flight.hpp"

#include <algorithm>

namespace omni {

BallFlight::BallFlight(double mass, double radius, double drag_cd, double air_density, double air_viscosity,
                       double gravity) {
    double area = kPiC * (radius * radius);
    k_quad = air_density * area * drag_cd / 2 / mass;
    k_lin = 6 * kPiC * air_viscosity * radius / mass;
    gz = gravity;
}

V3 BallFlight::accel(const V3& v) const {
    V3 a;
    acc(v[0], v[1], v[2], a[0], a[1], a[2]);
    return a;
}

std::optional<Crossing> BallFlight::crossing_state(const V3& pos, const V3& vel, double z_plane, double dt,
                                                   double t_max) const {
    double x = pos[0], y = pos[1], z = pos[2], vx = vel[0], vy = vel[1], vz = vel[2];
    double t = 0.0, h = dt / 2;
    while (t < t_max) {
        double a1x, a1y, a1z, a2x, a2y, a2z, a3x, a3y, a3z, a4x, a4y, a4z;
        acc(vx, vy, vz, a1x, a1y, a1z);
        double v2x = vx + h * a1x, v2y = vy + h * a1y, v2z = vz + h * a1z;
        acc(v2x, v2y, v2z, a2x, a2y, a2z);
        double v3x = vx + h * a2x, v3y = vy + h * a2y, v3z = vz + h * a2z;
        acc(v3x, v3y, v3z, a3x, a3y, a3z);
        double v4x = vx + dt * a3x, v4y = vy + dt * a3y, v4z = vz + dt * a3z;
        acc(v4x, v4y, v4z, a4x, a4y, a4z);
        double nx = x + dt / 6 * (vx + 2 * v2x + 2 * v3x + v4x);
        double ny = y + dt / 6 * (vy + 2 * v2y + 2 * v3y + v4y);
        double nz = z + dt / 6 * (vz + 2 * v2z + 2 * v3z + v4z);
        double nvz = vz + dt / 6 * (a1z + 2 * a2z + 2 * a3z + a4z);
        double nvx = vx + dt / 6 * (a1x + 2 * a2x + 2 * a3x + a4x);
        double nvy = vy + dt / 6 * (a1y + 2 * a2y + 2 * a3y + a4y);
        if (nvz < 0 && z >= z_plane && z_plane > nz) {
            double f = (z - z_plane) / (z - nz);
            return Crossing{{x + f * (nx - x), y + f * (ny - y), z_plane},
                            {vx + f * (nvx - vx), vy + f * (nvy - vy), vz + f * (nvz - vz)}, t + f * dt};
        }
        x = nx, y = ny, z = nz, vx = nvx, vy = nvy, vz = nvz, t = t + dt;
    }
    return std::nullopt;
}

double BallFlight::apex(const V3& pos, const V3& vel, double dt) const {
    double z = pos[2], vx = vel[0], vy = vel[1], vz = vel[2];
    double top = z;
    while (vz > 0) {
        double ax, ay, az, bx, by, bz;
        acc(vx, vy, vz, ax, ay, az);
        double v2x = vx + dt / 2 * ax, v2y = vy + dt / 2 * ay, v2z = vz + dt / 2 * az;
        acc(v2x, v2y, v2z, bx, by, bz);
        z += dt * v2z;
        vx = vx + dt * bx, vy = vy + dt * by, vz = vz + dt * bz;
        top = std::max(top, z);
    }
    return top;
}

std::optional<std::pair<V3, double>> BallFlight::release_point(const V3& pos, const V3& vel, double z_release,
                                                               double dt, double t_max) const {
    double x = pos[0], y = pos[1], z = pos[2], vx = vel[0], vy = vel[1], vz = vel[2];
    double t = 0.0;
    while (t < t_max) {
        double ax, ay, az, bx, by, bz;
        acc(vx, vy, vz, ax, ay, az);
        double mx = vx - dt / 2 * ax, my = vy - dt / 2 * ay, mz = vz - dt / 2 * az;
        acc(mx, my, mz, bx, by, bz);
        double nx = x - dt * mx, ny = y - dt * my, nz = z - dt * mz;
        double nvx = vx - dt * bx, nvy = vy - dt * by, nvz = vz - dt * bz;
        if (nvz > 0 && z >= z_release && z_release > nz) {
            double f = (z - z_release) / (z - nz);
            return std::make_pair(V3{x + f * (nx - x), y + f * (ny - y), z_release}, t + f * dt);
        }
        x = nx, y = ny, z = nz, vx = nvx, vy = nvy, vz = nvz, t = t + dt;
    }
    return std::nullopt;
}

double BallFlight::horizontal_range(const V3& pos, const V3& vel, double z_plane) const {
    auto hit = crossing_state(pos, vel, z_plane);
    if (!hit) return 0.0;
    return std::hypot(hit->pos[0] - pos[0], hit->pos[1] - pos[1]);
}

double BallFlight::solve_launch_speed(const V3& exit_pos, double ux, double uy, double elevation,
                                      const V3& target) const {
    double dx = target[0] - exit_pos[0], dy = target[1] - exit_pos[1];
    double dist = std::sqrt(dx * dx + dy * dy);
    double n = std::sqrt(ux * ux + uy * uy);
    ux /= n, uy /= n;
    double ce = std::cos(elevation), se = std::sin(elevation);
    double lo = 0.0, hi = 25.0;
    for (int k = 0; k < 30; ++k) {
        double mid = 0.5 * (lo + hi);
        if (horizontal_range(exit_pos, {ux * mid * ce, uy * mid * ce, mid * se}, target[2]) < dist)
            lo = mid;
        else
            hi = mid;
    }
    return 0.5 * (lo + hi);
}

}  // namespace omni
