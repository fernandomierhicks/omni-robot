#include "omni_cpp/drive_guard.hpp"

#include <algorithm>
#include <cmath>

namespace omni {

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double NORMAL_BAND = 0.01;     // m: the points the circle meets within this of the first set the normal
}

std::vector<GuardPoint> guard_scan_points(const std::vector<float>& ranges, double angle_min, double angle_inc,
                                          double range_min, double range_max, double max_gap, bool full_turn,
                                          int* n_filled) {
    const int n = (int)ranges.size();
    std::vector<GuardPoint> out;
    if (n_filled) *n_filled = 0;
    if (n == 0 || angle_inc == 0.0) return out;
    out.reserve(n);
    auto ok = [&](int i) {
        float r = ranges[i];
        return std::isfinite(r) && r >= range_min && r <= range_max;
    };
    auto put = [&](int i, double r) {
        double a = angle_min + angle_inc * i;
        out.push_back({r * std::cos(a), r * std::sin(a)});
    };
    int first_ok = -1;
    for (int i = 0; i < n; ++i)
        if (ok(i)) {
            put(i, ranges[i]);
            if (first_ok < 0) first_ok = i;
        }
    if (first_ok < 0) return out;                          // nothing at all: no scan to speak of
    const int max_run = (int)std::floor(max_gap / std::abs(angle_inc));
    // walk the empty runs, starting from a return so a wrapped run is seen whole
    int span = full_turn ? n : n - first_ok;
    int k = 0;
    while (k < span) {
        int i = (first_ok + k) % n;
        if (ok(i)) {
            ++k;
            continue;
        }
        int start = k;                                     // a run of empty bins begins at k
        while (k < span && !ok((first_ok + k) % n)) ++k;
        int len = k - start;
        if (k >= span && !full_turn) break;                // runs off the scan's end: no right flank
        if (len > max_run) continue;                       // open space
        int left = (first_ok + start - 1 + n) % n, right = (first_ok + k) % n;
        double r = std::min(ranges[left], ranges[right]) * std::cos(0.5 * (len + 1) * std::abs(angle_inc));
        for (int j = start; j < k; ++j) put((first_ok + j) % n, r);
        if (n_filled) *n_filled += len;
    }
    return out;
}

double DriveGuard::free_distance(double ux, double uy, double* nx, double* ny) const {
    const double R = cfg.radius, R2 = R * R, inner = R - cfg.self_margin;
    auto travel = [&](const GuardPoint& p, double* d) {   // how far the circle goes along u before it meets p
        double r2 = p.x * p.x + p.y * p.y;
        if (r2 < inner * inner) return false;              // the robot's own structure
        double along = p.x * ux + p.y * uy;
        if (along <= 0.0) return false;                    // behind the way: moving away never touches it
        double lat = p.x * uy - p.y * ux;
        if (lat * lat >= R2) return false;                 // passes beside
        *d = std::max(along - std::sqrt(R2 - lat * lat), 0.0);   // 0: already at the rim (contact)
        return true;
    };
    double best = 1e9, d;
    for (const GuardPoint& p : pts_)
        if (travel(p, &d)) best = std::min(best, d);
    if (nx && ny) {
        *nx = ux, *ny = uy;
        if (best < 1e8) {
            // the contact normal, centre to point where the circle stops, averaged over the way's points within
            // NORMAL_BAND of the circle there: a wall's points either side of the touch (symmetric about it) give the
            // wall's own normal
            double cx = best * ux, cy = best * uy, sx = 0.0, sy = 0.0;
            for (const GuardPoint& p : pts_) {
                double dx = p.x - cx, dy = p.y - cy, l = std::hypot(dx, dy);
                if (l > 1e-9 && l <= R + NORMAL_BAND && travel(p, &d)) sx += dx / l, sy += dy / l;
            }
            double l = std::hypot(sx, sy);
            if (l > 1e-9) *nx = sx / l, *ny = sy / l;
        }
    }
    return best;
}

double DriveGuard::allowed_speed(double free) const {
    const double a = cfg.decel, tau = cfg.latency;
    double d = free - cfg.buffer;
    double law = d > 0.0 ? -a * tau + std::sqrt(a * a * tau * tau + 2.0 * a * d) : 0.0;
    double touch = cfg.buffer > 0.0 ? cfg.touch_speed * std::clamp(free / cfg.buffer, 0.0, 1.0) : cfg.touch_speed;
    return std::max(law, touch);
}

DriveGuard::Way DriveGuard::way(double vx, double vy, double wz) const {
    Way w;
    double v = std::hypot(vx, vy);
    if (v < 1e-9) return w;
    double ux = vx / v, uy = vy / v;
    // a turn sweeps the way's heading over the stopping time: the way is fixed in the body, which turns by wz * t,
    // so in today's frame it heads wz * t further round (an arc approximated by its headings' straight ways)
    double t_stop = cfg.latency + v / cfg.decel;
    double lim = cfg.max_fan_deg * PI / 180.0;
    double sweep = std::clamp(wz * t_stop, -lim, lim);
    int n = std::abs(sweep) > 1e-3 ? std::max(1, (int)std::ceil(std::abs(sweep) / (10.0 * PI / 180.0))) : 0;
    for (int k = 0; k <= n; ++k) {
        double th = n ? sweep * k / n : 0.0;
        double c = std::cos(th), s = std::sin(th), nx, ny;
        double f = free_distance(c * ux - s * uy, s * ux + c * uy, &nx, &ny);
        if (f < w.free) w.free = f, w.nx = nx, w.ny = ny;
    }
    return w;
}

bool DriveGuard::safe(double vx, double vy, double wz) const {
    double v = std::hypot(vx, vy);
    if (v < 1e-9) return true;
    Way w = way(vx, vy, wz);
    return w.free >= 1e8 || v <= allowed_speed(w.free) + 1e-9;
}

GuardResult DriveGuard::limit(double vx, double vy, double wz) const {
    GuardResult out;
    out.vx = vx, out.vy = vy, out.wz = wz;
    double v = std::hypot(vx, vy);
    out.asked = v;
    if (v < 1e-4) return out;
    const Way w = way(vx, vy, wz);
    out.free = w.free;
    if (w.free >= 1e8) return out;
    out.limit = allowed_speed(w.free);
    if (v <= out.limit) return out;
    out.limited = true;
    double cx = vx, cy = vy;
    if (cfg.slide) {
        // keep the part along what it would hit, cut the part into it as far as the law needs; a second obstacle
        // (a corner) takes another pass
        Way c = w;
        for (int pass = 0; pass < 3; ++pass) {
            if (pass) {
                c = way(cx, cy, wz);
                if (c.free >= 1e8 || std::hypot(cx, cy) <= allowed_speed(c.free)) break;
            }
            double vn = cx * c.nx + cy * c.ny;              // > 0: into it
            if (vn <= 0.0) break;
            double tx = cx - vn * c.nx, ty = cy - vn * c.ny;
            if (std::hypot(tx, ty) < 1e-3 * std::hypot(cx, cy)) break;   // square on: nothing to slide along
            if (!safe(tx, ty, wz)) {                        // the slide itself meets something: drop the part into
                cx = tx, cy = ty;                           // this one, the next pass cuts the slide
                continue;
            }
            double lo = 0.0, hi = 1.0;                      // the share of the part into it that stays
            for (int i = 0; i < 12; ++i) {
                double m = 0.5 * (lo + hi);
                (safe(tx + m * vn * c.nx, ty + m * vn * c.ny, wz) ? lo : hi) = m;
            }
            cx = tx + hi * vn * c.nx, cy = ty + hi * vn * c.ny;   // just over the line: the cap below lands on it
            break;
        }
    }
    // what is left, capped along its own way (no slide: the asked way, the direction kept)
    double s = std::hypot(cx, cy);
    if (s > 1e-9) {
        Way f = way(cx, cy, wz);
        if (f.free < 1e8) {
            double lim = allowed_speed(f.free);
            if (s > lim) cx *= lim / s, cy *= lim / s;
        }
    }
    out.vx = cx, out.vy = cy;
    return out;
}

}  // namespace omni
