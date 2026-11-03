#include "omni_cpp/scan.hpp"

#include <algorithm>
#include <cmath>

namespace omni {

namespace {

constexpr double PI = 3.14159265358979323846;
const double INF = std::numeric_limits<double>::infinity();

double pymod(double a, double b) {             // numpy's mod: the sign of the divisor
    double m = std::fmod(a, b);
    return (m != 0 && ((m < 0) != (b < 0))) ? m + b : m;
}

double interp(double x, const std::vector<double>& xp, const std::vector<double>& fp) {   // numpy's interp
    if (x <= xp.front()) return fp.front();
    if (x >= xp.back()) return fp.back();
    size_t j = std::upper_bound(xp.begin(), xp.end(), x) - xp.begin() - 1;
    double s = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
    return s * (x - xp[j]) + fp[j];
}

double quantile(std::vector<double> v, double q) {   // numpy's default (linear)
    std::sort(v.begin(), v.end());
    double pos = q * (double)(v.size() - 1);
    size_t i = (size_t)std::floor(pos);
    double f = pos - (double)i;
    return i + 1 < v.size() ? v[i] + f * (v[i + 1] - v[i]) : v[i];
}

// least squares about the means (scan_time._line): raw stamps (~1.7e9 s) would lose ~1e-8 s of the slope
void polyfit1(const std::vector<double>& x, const std::vector<double>& y, double& slope, double& icpt) {
    double xm = 0, ym = 0;
    for (size_t i = 0; i < x.size(); ++i) xm += x[i], ym += y[i];
    xm /= (double)x.size(), ym /= (double)x.size();
    double sxy = 0, sxx = 0;
    for (size_t i = 0; i < x.size(); ++i) sxy += (x[i] - xm) * (y[i] - ym), sxx += (x[i] - xm) * (x[i] - xm);
    slope = sxy / sxx;
    icpt = ym - slope * xm;
}

}  // namespace

std::vector<bool> floor_mask(const std::vector<double>& a, const std::vector<double>& r, const LidarCal& cal,
                             double roll, double pitch, double margin) {
    double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
    std::vector<bool> out(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(r[i])) continue;
        double d0 = std::cos(a[i]), d1 = std::sin(a[i]), d2 = cal.tan_tilt * std::cos(a[i] - cal.azimuth);
        double w0 = cp * d0 + sp * sr * d1 + sp * cr * d2;       // Ry(pitch) Rx(roll), laser frame = base axes
        double w1 = cr * d1 - sr * d2;
        double w2 = -sp * d0 + cp * sr * d1 + cp * cr * d2;
        double slope = w2 / std::hypot(w0, w1);
        double r_floor = slope < -1e-4 ? cal.height / std::max(-slope, 1e-4) : INF;
        out[i] = r[i] >= r_floor - margin;
    }
    return out;
}

std::vector<double> sample_times(size_t n, double stamp, double time_increment, int order) {
    std::vector<double> t(n);
    for (size_t i = 0; i < n; ++i) t[i] = stamp + (double)(order < 0 ? n - 1 - i : i) * time_increment;
    return t;
}

std::vector<double> deskew(const std::vector<double>& a, const std::vector<double>& r, const std::vector<double>& ts,
                           const std::function<Pose2(double)>& pose_at, double lx, double ly, double t_ref,
                           double angle_min, double inc, size_t n_out, int n_key) {
    double t0 = *std::min_element(ts.begin(), ts.end()), t1 = *std::max_element(ts.begin(), ts.end());
    double lo = std::min(t0, t_ref), hi = std::max(t1, t_ref);
    std::vector<double> tk(n_key), kx(n_key), ky(n_key), kyaw(n_key);
    double step = (hi - lo) / (n_key - 1);
    for (int j = 0; j < n_key; ++j) {
        tk[j] = j == n_key - 1 ? hi : lo + j * step;
        Pose2 p = pose_at(tk[j]);
        kx[j] = p.x, ky[j] = p.y, kyaw[j] = p.yaw;
    }
    const std::vector<double> raw = kyaw;
    for (int j = 1; j < n_key; ++j) {          // numpy's unwrap
        double d = raw[j] - raw[j - 1];
        double dm = pymod(d + PI, 2 * PI) - PI;
        if (dm == -PI && d > 0) dm = PI;
        kyaw[j] = kyaw[j - 1] + (std::fabs(d) < PI ? d : dm);
    }
    double rx = interp(t_ref, tk, kx), ry = interp(t_ref, tk, ky), ryaw = interp(t_ref, tk, kyaw);
    double c0 = std::cos(ryaw), s0 = std::sin(ryaw);
    std::vector<double> out(n_out, INF);
    for (size_t i = 0; i < r.size(); ++i) {
        if (!std::isfinite(r[i])) continue;
        double px = interp(ts[i], tk, kx), py = interp(ts[i], tk, ky), pyaw = interp(ts[i], tk, kyaw);
        double bx = lx + r[i] * std::cos(a[i]), by = ly + r[i] * std::sin(a[i]);   // base frame at the sample's time
        double c = std::cos(pyaw), s = std::sin(pyaw);
        double wx = px + c * bx - s * by, wy = py + s * bx + c * by;              // odom frame
        double qx = c0 * (wx - rx) + s0 * (wy - ry) - lx;                         // laser at t_ref
        double qy = -s0 * (wx - rx) + c0 * (wy - ry) - ly;
        double aa = pymod(std::atan2(qy, qx) - angle_min, 2 * PI);
        long k = (long)std::nearbyint(aa / inc) % (long)n_out;
        out[k] = std::min(out[k], std::hypot(qx, qy));
    }
    return out;
}

double ScanClock::update(double stamp, double scan_time) {
    if (!has_last_) {
        k_ = 0;
    } else {
        double T0 = T != 0 ? T : (scan_time > 0 ? scan_time : 0);
        long dk = T0 != 0 ? (long)std::nearbyint((stamp - t_last_) / T0) : 0;
        if (dk < 1 || dk > 50) {               // out of order, a long gap or no period: start a new line
            hist_.clear();
            k_ = 0;
        } else {
            k_ += dk;
        }
    }
    has_last_ = true;
    t_last_ = stamp;
    hist_.emplace_back(k_, stamp);
    if (hist_.size() > window_) hist_.pop_front();
    if (hist_.size() >= 5) {
        std::vector<double> k, t;
        for (auto& h : hist_) k.push_back((double)h.first), t.push_back(h.second);
        double slope, t0;
        polyfit1(k, t, slope, t0);
        for (int it = 0; it < 2; ++it) {       // the line under the stamps: shift down to their low quantile
            std::vector<double> res(k.size());
            for (size_t i = 0; i < k.size(); ++i) res[i] = t[i] - (t0 + slope * k[i]);
            double med = quantile(res, 0.5);
            std::vector<double> kl, tl;
            for (size_t i = 0; i < k.size(); ++i)
                if (res[i] <= med) kl.push_back(k[i]), tl.push_back(t[i]);
            if (kl.size() >= 3) polyfit1(kl, tl, slope, t0);
        }
        std::vector<double> res(k.size());
        double mean = 0;
        for (size_t i = 0; i < k.size(); ++i) res[i] = t[i] - (t0 + slope * k[i]), mean += res[i];
        mean /= (double)res.size();
        double var = 0;
        for (double v : res) var += (v - mean) * (v - mean);
        t0 += quantile(res, q_);
        T = slope;
        jitter = std::sqrt(var / (double)res.size());
        return t0 + slope * (double)k_ - latency_;
    }
    if (T == 0 && scan_time > 0) T = scan_time;
    return stamp - latency_;
}

}  // namespace omni
