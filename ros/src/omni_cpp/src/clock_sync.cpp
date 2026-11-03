#include "omni_cpp/clock_sync.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "omni_cpp/json.hpp"

namespace omni {

namespace {

const double NaN = std::numeric_limits<double>::quiet_NaN();

int64_t floor_div(int64_t a, int64_t b) {      // Python's //
    int64_t q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

// Weighted least squares y = b + e x. One point, or points over less than 2 s: the offset only, the drift kept at
// its prior (the slope isn't observable yet).
ClockFit line(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& w, double prior) {
    double W = 0, xmin = x[0], xmax = x[0];
    for (size_t i = 0; i < x.size(); ++i) W += w[i], xmin = std::min(xmin, x[i]), xmax = std::max(xmax, x[i]);
    if (x.size() == 1 || xmax - xmin < 2e9) {
        double s = 0;
        for (size_t i = 0; i < x.size(); ++i) s += w[i] * (y[i] - prior * x[i]);
        return {s / W, prior};
    }
    double xm = 0, ym = 0;
    for (size_t i = 0; i < x.size(); ++i) xm += w[i] * x[i], ym += w[i] * y[i];
    xm /= W, ym /= W;
    double sxy = 0, sxx = 0;
    for (size_t i = 0; i < x.size(); ++i) sxy += w[i] * (x[i] - xm) * (y[i] - ym), sxx += w[i] * (x[i] - xm) * (x[i] - xm);
    double e = sxy / sxx;
    return {ym - e * xm, e};
}

double median(std::vector<double> v) {         // numpy's: the mean of the middle two for an even count
    size_t n = v.size(), h = n / 2;
    std::nth_element(v.begin(), v.begin() + h, v.end());
    double hi = v[h];
    if (n % 2) return hi;
    return 0.5 * (hi + *std::max_element(v.begin(), v.begin() + h));
}

}  // namespace

ClockSync::ClockSync(const ClockSyncConfig& c)
    : echo_rms(NaN), pulse_rms(NaN),
      echo_window_ns_((int64_t)(c.echo_window_s * 1e9)), pulse_window_ns_((int64_t)(c.pulse_window_s * 1e9)),
      period_ns_((int64_t)(c.pulse_period_s * 1e9)), edge_latency_ns_(c.edge_latency_ns),
      refit_ns_((int64_t)(c.refit_s * 1e9)) {}

void ClockSync::add(int64_t t1, int64_t t2_us, int64_t t3_us, int64_t t4) {
    if (!t1 || !t2_us || t4 < t1) return;
    int64_t t2 = t2_us * 1000, t3 = t3_us * 1000;
    int64_t rtt = (t4 - t1) - (t3 - t2);
    if (rtt < 0) return;
    int64_t mcu_mid = (t2 + t3) / 2, host_mid = (t1 + t4) / 2;
    if (!m0_) reset(mcu_mid, host_mid);
    else if (std::fabs(mcu_to_host_ns((double)mcu_mid) - (double)host_mid) > 1e9)
        reset(mcu_mid, host_mid);              // the MCU rebooted (its clock restarted) or the host's jumped
    int64_t key = floor_div(host_mid, 1000000000);
    auto it = bins_.find(key);
    if (it == bins_.end() || rtt < it->second.rtt) bins_[key] = {rtt, mcu_mid, host_mid};
    t_last_host_ = t4;
    int64_t old = floor_div(t4 - echo_window_ns_, 1000000000);
    bins_.erase(bins_.begin(), bins_.lower_bound(old));
    int64_t best = std::numeric_limits<int64_t>::max();
    for (auto& kv : bins_) best = std::min(best, kv.second.rtt);
    rtt_ns = best;
    if (!t_refit_ || t4 - *t_refit_ >= refit_ns_ || !echo_fit) {
        t_refit_ = t4;
        fit_echo();
        pair();
    }
}

void ClockSync::add_mcu_pulse(int64_t seq, int64_t t_mcu_us) {
    ++n_pulses;
    mcu_pulses_.emplace_back(seq, t_mcu_us * 1000);
    while (mcu_pulses_.size() > 16) mcu_pulses_.pop_front();
    pair();
}

void ClockSync::add_host_edge(int64_t t_host_ns) {
    ++n_edges;
    edges_.push_back(t_host_ns - edge_latency_ns_);
    t_last_host_ = std::max(t_last_host_.value_or(0), t_host_ns);
    while (edges_.size() > 16) edges_.pop_front();
    pair();
}

void ClockSync::reset(int64_t mcu_ns, int64_t host_ns) {
    if (m0_) ++n_resets;
    m0_ = mcu_ns, h0_ = host_ns;
    bins_.clear();
    pairs_.clear();
    echo_fit.reset();
    pulse_fit.reset();
}

void ClockSync::fit_echo() {
    if (bins_.empty()) return;
    std::vector<double> x, y, w;
    for (auto& kv : bins_) {
        const Bin& b = kv.second;
        int64_t dx = b.mcu_mid - *m0_;
        x.push_back((double)dx);
        y.push_back((double)(b.host_mid - *h0_ - dx));   // what the clocks disagree by, beyond rate 1
        double h = std::max(b.rtt * 0.5, 1e3);           // +/- rtt / 2
        w.push_back(1.0 / (h * h));
    }
    ClockFit f = line(x, y, w, echo_fit ? echo_fit->e : 0.0);
    double ss = 0;
    for (size_t i = 0; i < x.size(); ++i) {
        double r = y[i] - (f.b + f.e * x[i]);
        ss += r * r;
    }
    echo_fit = f;
    echo_rms = std::sqrt(ss / x.size());
    n_echo_used = (int)x.size();
}

void ClockSync::pair() {
    // match each MCU pulse with the host edge nearest its predicted time (within a fifth of the period)
    if (!m0_ || mcu_pulses_.empty() || edges_.empty()) return;
    int64_t tol = std::max(period_ns_ / 5, (int64_t)20000000);
    bool changed = false;
    auto pulses = mcu_pulses_;
    for (auto& [seq, t_m] : pulses) {
        if (edges_.empty()) break;
        double pred = mcu_to_host_ns((double)t_m);
        if (std::isnan(pred)) continue;
        auto best = std::min_element(edges_.begin(), edges_.end(), [&](int64_t a, int64_t b) {
            return std::fabs((double)a - pred) < std::fabs((double)b - pred);
        });
        if (std::fabs((double)*best - pred) <= (double)tol) {
            pairs_.push_back({t_m, *best, seq});
            mcu_pulses_.erase(std::find(mcu_pulses_.begin(), mcu_pulses_.end(), std::make_pair(seq, t_m)));
            edges_.erase(best);
            ++n_paired;
            changed = true;
        }
    }
    if (!edges_.empty() && t_last_host_)       // edges nobody claimed (noise on the line, a lost event)
        while (!edges_.empty() && edges_.front() < edges_.back() - 3 * period_ns_) edges_.pop_front();
    if (changed) {
        int64_t last = pairs_.back().h;
        while (!pairs_.empty() && pairs_.front().h < last - pulse_window_ns_) pairs_.pop_front();
        fit_pulses();
    }
}

void ClockSync::fit_pulses() {
    size_t n = pairs_.size();
    std::vector<double> x(n), y(n);
    for (size_t i = 0; i < n; ++i) {
        int64_t dx = pairs_[i].m - *m0_;
        x[i] = (double)dx;
        y[i] = (double)(pairs_[i].h - *h0_ - dx);
    }
    std::vector<bool> keep(n, true);
    double prior = pulse_fit ? pulse_fit->e : (echo_fit ? echo_fit->e : 0.0);
    auto fit_kept = [&](const std::vector<bool>& k) {
        std::vector<double> xs, ys;
        for (size_t i = 0; i < n; ++i) if (k[i]) xs.push_back(x[i]), ys.push_back(y[i]);
        return line(xs, ys, std::vector<double>(xs.size(), 1.0), prior);
    };
    ClockFit f = fit_kept(keep);
    for (int it = 0; it < 3; ++it) {           // a GPIO stamp is only ever late: drop the late tail
        std::vector<double> r(n), rk;
        for (size_t i = 0; i < n; ++i) {
            r[i] = y[i] - (f.b + f.e * x[i]);
            if (keep[i]) rk.push_back(r[i]);
        }
        double s = 2e3;
        if (rk.size() > 4) {
            double m = median(rk);
            for (double& v : rk) v = std::fabs(v - m);
            s = 1.4826 * median(rk);
        }
        std::vector<bool> nk(n);
        size_t cnt = 0;
        for (size_t i = 0; i < n; ++i) {
            nk[i] = r[i] <= std::max(3.0 * s, 2e3) && r[i] >= -std::max(6.0 * s, 1e4);
            cnt += nk[i];
        }
        if (cnt < 2 || nk == keep) break;
        keep = nk;
        f = fit_kept(keep);
    }
    double ss = 0;
    int used = 0;
    for (size_t i = 0; i < n; ++i)
        if (keep[i]) {
            double r = y[i] - (f.b + f.e * x[i]);
            ss += r * r;
            ++used;
        }
    n_rejected = (int)n - used;
    pulse_fit = f;
    pulse_rms = std::sqrt(ss / used);
    n_pulse_used = used;
}

std::string ClockSync::mode() const {
    if (pulse_fit && n_pulse_used >= 3 && t_last_host_ && !pairs_.empty() &&
        *t_last_host_ - pairs_.back().h < 5 * period_ns_)
        return "pulse";
    return echo_fit ? "echo" : "none";
}

const std::optional<ClockFit>& ClockSync::active() const { return mode() == "pulse" ? pulse_fit : echo_fit; }

double ClockSync::mcu_to_host_ns(double t_mcu_ns) const {
    const auto& f = active();
    if (!f) return NaN;
    double x = t_mcu_ns - (double)*m0_;
    return (double)*h0_ + x + f->b + f->e * x;
}

double ClockSync::host_to_mcu_ns(double t_host_ns) const {
    const auto& f = active();
    if (!f) return NaN;
    return (double)*m0_ + (t_host_ns - (double)*h0_ - f->b) / (1.0 + f->e);
}

double ClockSync::to_host_s(int64_t t_mcu_us) const {
    if (!m0_) return NaN;
    // the difference from the reference first, in integers: no rounding of a large clock value
    const auto& f = active();
    if (!f) return NaN;
    int64_t dx = t_mcu_us * 1000 - *m0_;
    return ((double)(*h0_ + dx) + f->b + f->e * (double)dx) * 1e-9;
}

int64_t ClockSync::to_mcu_us(double t_host_s) const {
    if (!m0_ || !active()) return 0;
    return (int64_t)std::nearbyint(host_to_mcu_ns(t_host_s * 1e9) / 1000.0);
}

std::optional<int64_t> ClockSync::offset_ns() const {
    if (!m0_ || !active()) return std::nullopt;
    int64_t t = t_last_host_ ? *t_last_host_ : *h0_;
    return (int64_t)std::nearbyint(host_to_mcu_ns((double)t) - (double)t);
}

std::string ClockSync::status_json(const std::string& source) const {
    std::string m = mode();
    const auto& f = active();
    auto off = offset_ns();
    JsonObject j;
    j.string("mode", m).number("offset_us", off ? std::optional<double>(*off / 1000.0) : std::nullopt, 1);
    if (f) j.number("drift_ppm", -f->e / (1.0 + f->e) * 1e6, 3);
    else j.null("drift_ppm");
    if (f) j.number("rms_us", (m == "pulse" ? pulse_rms : echo_rms) / 1000.0, 2);
    else j.null("rms_us");
    j.number("rtt_us", rtt_ns ? std::optional<double>(*rtt_ns / 1000.0) : std::nullopt, 1)
        .integer("pulses", n_pulse_used).integer("echoes", n_echo_used).integer("rejected", n_rejected)
        .integer("resets", n_resets).integer("unpaired_edges", (int64_t)edges_.size());
    if (pulse_fit && echo_fit && t_last_host_) {
        double x = host_to_mcu_ns((double)*t_last_host_) - (double)*m0_;
        j.number("echo_minus_pulse_us",
                 ((echo_fit->b + echo_fit->e * x) - (pulse_fit->b + pulse_fit->e * x)) / 1000.0, 1);
    }
    if (!source.empty()) j.string("source", source);
    return j.str();
}

}  // namespace omni
