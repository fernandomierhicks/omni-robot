#include "omni_cpp/rplidar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace omni::rplidar {

namespace {

const double NaN = std::numeric_limits<double>::quiet_NaN();

int64_t wrap_q6(int64_t a) {
    if (a < 0) a += 360 << 6;
    if (a >= 360 << 6) a -= 360 << 6;
    return a;
}

void varbitscale(int64_t scaled, int64_t& value, int& lvl) {
    static const int64_t T[5][3] = {{3328, 4, 1 << 14}, {1792, 3, 1 << 12}, {1280, 2, 1 << 11}, {512, 1, 1 << 9}, {0, 0, 0}};
    for (const auto& r : T)
        if (scaled >= r[0]) {
            value = r[2] + ((scaled - r[0]) << r[1]);
            lvl = (int)r[1];
            return;
        }
    value = 0, lvl = 0;
}

int64_t signed10(uint32_t v) {
    v &= 0x3FF;
    return (v & 0x200) ? (int64_t)v - 0x400 : (int64_t)v;
}

uint32_t le32(const uint8_t* p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

double enc_deg(int64_t raw_q16) { return std::fmod((double)raw_q16 / 65536.0, 360.0); }

double median(std::vector<double> v) {
    if (v.empty()) return NaN;
    size_t h = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + (long)h, v.end());
    return v[h];
}

// least squares y = c + s x about the means
bool line(const std::vector<double>& x, const std::vector<double>& y, double& s, double& c) {
    size_t n = x.size();
    if (n < 2) return false;
    double xm = 0, ym = 0;
    for (size_t i = 0; i < n; ++i) xm += x[i], ym += y[i];
    xm /= (double)n, ym /= (double)n;
    double sxy = 0, sxx = 0;
    for (size_t i = 0; i < n; ++i) sxy += (x[i] - xm) * (y[i] - ym), sxx += (x[i] - xm) * (x[i] - xm);
    if (sxx <= 0) return false;
    s = sxy / sxx;
    c = ym - s * xm;
    return true;
}

}  // namespace

std::vector<uint8_t> request(uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> p{0xA5, cmd};
    if (payload.empty()) return p;
    p.push_back((uint8_t)payload.size());
    p.insert(p.end(), payload.begin(), payload.end());
    uint8_t cs = 0;
    for (uint8_t b : p) cs ^= b;
    p.push_back(cs);
    return p;
}

bool descriptor(const uint8_t* d, size_t n, Descriptor& out) {
    if (n < 7 || d[0] != 0xA5 || d[1] != 0x5A) return false;
    uint32_t v = le32(d + 2);
    out.size = v & 0x3FFFFFFF;
    out.mode = (uint8_t)(v >> 30);
    out.type = d[6];
    return true;
}

std::string model_name(uint8_t model) {
    int major = model >> 4, sub = model & 0xF;
    if (major >= 1 && major <= 3) return "A" + std::to_string(major) + "M" + std::to_string(sub);
    char b[16];
    std::snprintf(b, sizeof b, "model 0x%02X", model);
    return b;
}

Format format(uint8_t t) {
    switch (t) {
        case ANS_STANDARD: return {t, 1, 5, false};
        case ANS_EXPRESS: return {t, 32, 84, true};
        case ANS_ULTRA: return {t, 96, 132, true};
    }
    throw std::invalid_argument("rplidar: answer type " + std::to_string(t) + " isn't one this driver reads");
}

// -- Decoder -----------------------------------------------------------------------------------------------------------

Decoder::Decoder(uint8_t ans_type, double period_s) : fmt(format(ans_type)), period_(period_s) {}

void Decoder::feed(const uint8_t* d, size_t n, double t, std::vector<Sample>& out, std::vector<Anchor>& anchors) {
    buf_.insert(buf_.end(), d, d + n);
    size_t i = 0, N = (size_t)fmt.bytes;
    while (buf_.size() - i >= N) {
        const uint8_t* c = buf_.data() + i;
        if (fmt.type == ANS_STANDARD) {
            int s = c[0] & 1;
            if (s == ((c[0] >> 1) & 1) || !(c[1] & 1)) {
                ++bad, gap_ = true, ++i;
                continue;
            }
            standard(c, t, out, anchors);
            i += N;
            continue;
        }
        if ((c[0] >> 4) != 0xA || (c[1] >> 4) != 0x5) {
            ++bad, gap_ = true, ++i;
            continue;
        }
        uint8_t cs = 0;
        for (size_t k = 2; k < N; ++k) cs ^= c[k];
        if (cs != ((c[0] & 0xF) | ((c[1] & 0xF) << 4))) {
            ++bad, gap_ = true, ++i;
            continue;
        }
        capsule(c, t, out, anchors);
        i += N;
    }
    buf_.erase(buf_.begin(), buf_.begin() + (long)i);
}

void Decoder::standard(const uint8_t* c, double t, std::vector<Sample>& out, std::vector<Anchor>& anchors) {
    Sample x;
    x.angle = x.enc = ((c[1] >> 1) | (c[2] << 7)) / 64.0;
    x.mm = (c[3] | (c[4] << 8)) / 4.0;
    x.quality = c[0] >> 2;
    x.start = c[0] & 1;
    bool fresh = !any_;
    int64_t n = next_n_;
    if (any_ && gap_) {            // its angles are the parallax-corrected ones: a lost sample can't be counted
        fresh = true, ++breaks;
        n += std::max<int64_t>(0, std::llround((t - t_last_) / period_) - 1);
    }
    anchors.push_back({n + 1, t, fresh, out.size()});
    x.n = n;
    out.push_back(x);
    next_n_ = n + 1, t_last_ = t, any_ = true, gap_ = false, ++packets;
}

void Decoder::capsule(const uint8_t* c, double t, std::vector<Sample>& out, std::vector<Anchor>& anchors) {
    int start = c[2] | (c[3] << 8);
    double a = (start & 0x7FFF) / 64.0;
    bool fresh = false;
    int64_t n = next_n_;
    double delta = std::fmod(a - a_last_ + 360.0, 360.0);
    if (!any_ || (start & 0x8000)) {             // the scan's first capsule: nothing before it to pair or count
        fresh = true, has_prev_ = false;
    } else if (gap_ || (adv_ > 0 && delta > 1.5 * adv_) || t - t_last_ > 0.5) {
        // bytes skipped, whole capsules missing, or no data for 0.5 s (where the angle may alias): how many went?
        double dt = t - t_last_, by_time = dt / (fmt.K * period_);
        int64_t g = std::max<int64_t>(1, std::llround(by_time));
        bool ok = false;
        if (adv_ > 0 && dt < 0.5) {
            double best = 1e18, g_real = 0;
            for (int j = 0; j < 4; ++j) {         // whole turns lost besides: the arrival times choose
                double gj = (delta + 360.0 * j) / adv_;
                if (std::fabs(gj - by_time) < best) best = std::fabs(gj - by_time), g_real = gj;
            }
            g = std::llround(g_real);
            ok = g >= 1 && std::fabs(g_real - (double)g) < 0.3 && std::fabs((double)g - by_time) < 0.5 * 360.0 / adv_;
            g = std::max<int64_t>(1, g);
        }
        n += (g - 1) * fmt.K;
        if (ok) {
            lost += g - 1;
            if (g > 1) has_prev_ = false;        // its pair is gone; the angles can't be interpolated across
        } else {
            fresh = true, has_prev_ = false, ++breaks;
        }
    } else {
        adv_ = adv_ > 0 ? 0.9 * adv_ + 0.1 * delta : delta;
    }
    if (has_prev_) decode(prev_.data(), c, prev_n_, out);
    anchors.push_back({n + fmt.K, t, fresh, out.size()});
    prev_.assign(c, c + fmt.bytes);
    prev_n_ = n, has_prev_ = true;
    next_n_ = n + fmt.K, a_last_ = a, t_last_ = t, any_ = true, gap_ = false, ++packets;
}

void Decoder::decode(const uint8_t* prev, const uint8_t* cur, int64_t n0, std::vector<Sample>& out) const {
    // omni_console/rplidar.py's _CapsuleParser._angles and the two decoders, number for number (the SDK's)
    const int K = fmt.K;
    int64_t a_cur = (int64_t)((cur[2] | (cur[3] << 8)) & 0x7FFF) << 2;
    int64_t a_prev = (int64_t)((prev[2] | (prev[3] << 8)) & 0x7FFF) << 2;
    int64_t diff = a_cur - a_prev;
    if (a_prev > a_cur) diff += 360 << 8;
    int64_t raw = a_prev << 8, inc = (diff << 8) / K;
    const int64_t full = (int64_t)360 << 16;
    int64_t n = n0;
    auto emit = [&](int64_t ang_q6, double mm) {
        Sample s;
        s.n = n++;
        s.enc = enc_deg(raw);
        s.angle = ang_q6 / 64.0;
        s.mm = mm;
        s.quality = mm != 0 ? 47 : 0;
        s.start = ((raw + inc) % full) < inc;
        out.push_back(s);
    };
    if (fmt.type == ANS_EXPRESS) {
        for (int pos = 0; pos < 16; ++pos) {
            const uint8_t* o = prev + 4 + 5 * pos;
            int d1 = o[0] | (o[1] << 8), d2 = o[2] | (o[3] << 8), off = o[4];
            const int pairs[2][2] = {{d1, (off & 0xF) | ((d1 & 3) << 4)}, {d2, (off >> 4) | ((d2 & 3) << 4)}};
            for (const auto& p : pairs) {
                emit(wrap_q6((raw - ((int64_t)p[1] << 13)) >> 10), (double)(p[0] >> 2));
                raw += inc;
            }
        }
        return;
    }
    static const int64_t OFF0 = (int64_t)(7.5 * 3.1415926535 * (1 << 16) / 180.0);
    static const int64_t OFF8 = (int64_t)(8 * 3.1415926535 * (1 << 16) / 180);
    uint32_t cab[33];
    for (int k = 0; k < 32; ++k) cab[k] = le32(prev + 4 + 4 * k);
    cab[32] = le32(cur + 4);
    for (int pos = 0; pos < 32; ++pos) {
        uint32_t x = cab[pos];
        int64_t major, major2;
        int lvl1, lvl2;
        varbitscale(x & 0xFFF, major, lvl1);
        varbitscale(cab[pos + 1] & 0xFFF, major2, lvl2);
        int64_t p1 = signed10(x >> 12), p2 = signed10(x >> 22);
        int64_t base1 = major, base2 = major2;
        if (!major && major2) base1 = major2, lvl1 = lvl2;
        int64_t d[3] = {major * 4,
                        (p1 == -512 || p1 == 511) ? 0 : (p1 * ((int64_t)1 << lvl1) + base1) * 4,
                        (p2 == -512 || p2 == 511) ? 0 : (p2 * ((int64_t)1 << lvl2) + base2) * 4};
        for (int64_t dq2 : d) {
            int64_t off = OFF0;
            if (dq2 >= 50 * 4) {
                int64_t k2 = 98361 / dq2;
                off = OFF8 - (k2 << 6) - (k2 * k2 * k2) / 98304;
            }
            emit(wrap_q6((raw - (int64_t)((double)(off * 180) / 3.14159265)) >> 10), std::max<int64_t>(dq2, 0) / 4.0);
            raw += inc;
        }
    }
}

// -- SampleClock -------------------------------------------------------------------------------------------------------

SampleClock::SampleClock(double period_s, double delay_s, double window_s, double bucket_s, double fit_min_s)
    : period(period_s), nominal_(period_s), delay_(delay_s), window_(window_s), bucket_(bucket_s), fit_min_(fit_min_s) {}

void SampleClock::add(const Anchor& a) {
    if (!has_ || a.fresh || a.t < t_last_ || a.t - t_last_ > 0.5) {     // a new line
        has_ = true;
        n0_ = a.n_end, t0_ = a.t, c_ = 0;
        b_.clear();
        period = nominal_;
        ++lines;
    }
    t_last_ = a.t;
    int64_t x = a.n_end - n0_;
    double y = a.t - t0_;
    int64_t k = (int64_t)std::floor(y / bucket_);
    auto low = [this](const Bucket& b) { return b.y - (double)b.x * nominal_; };
    if (b_.empty() || b_.back().k != k) {
        b_.push_back({k, x, y});
    } else if (y - (double)x * nominal_ < low(b_.back())) {
        b_.back() = {k, x, y};
    } else {
        return;                                   // nothing lower: the line stands
    }
    while (b_.front().y < y - window_) b_.pop_front();
    fit();
}

void SampleClock::fit() {
    std::vector<double> x, y;
    for (const auto& b : b_) x.push_back((double)b.x), y.push_back(b.y);
    double s = nominal_, c = 0;
    bool fitted = false;
    if (b_.size() >= 10 && b_.back().y - b_.front().y >= fit_min_ && line(x, y, s, c)) {
        for (int it = 0; it < 2; ++it) {         // the line under the points: refit on the lower half
            std::vector<double> r(x.size());
            for (size_t i = 0; i < x.size(); ++i) r[i] = y[i] - (c + s * x[i]);
            double med = median(r);
            std::vector<double> xl, yl;
            for (size_t i = 0; i < x.size(); ++i)
                if (r[i] <= med) xl.push_back(x[i]), yl.push_back(y[i]);
            if (xl.size() >= 3) line(xl, yl, s, c);
        }
        fitted = std::fabs(s / nominal_ - 1.0) < 1e-3;   // a slope off by > 1000 ppm is a bad fit, not the clock
    }
    if (!fitted) s = nominal_;
    double lo = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < x.size(); ++i) lo = std::min(lo, y[i] - s * x[i]);
    std::vector<double> r(x.size());
    for (size_t i = 0; i < x.size(); ++i) r[i] = y[i] - s * x[i] - lo;
    period = s, c_ = lo;
    spread = median(r);
}

double SampleClock::time(int64_t n) const {
    if (!has_) return NaN;
    return t0_ + c_ + period * (double)(n + 1 - n0_) - delay_;
}

double SampleClock::residual(const Anchor& a) const {
    return a.t - (t0_ + c_ + period * (double)(a.n_end - n0_));
}

// -- TurnBuilder -------------------------------------------------------------------------------------------------------

TurnBuilder::TurnBuilder(int bins_) : bins(bins_), inc_deg_(360.0 / bins_) {}

void TurnBuilder::reset() {
    started_ = false;
    p_max_ = -1;
    t_.clear(), p_.clear(), ang_.clear(), mm_.clear(), n_.clear(), line_.clear();
}

bool TurnBuilder::add(const Sample& s, double t, Turn& out, int line) {
    // the bearing the encoder angle stands for: the device reports bearings off its encoder (Ultra: the SDK's -8 deg
    // plus parallax; Express: its offset field; Standard: none), so the turns follow the typical offset, and only near
    // returns' parallax crosses the seam
    double d = std::fmod(s.angle - s.enc + 540.0, 360.0) - 180.0;
    if (!has_off_) off_ = off_turn_ = d;
    off_ += 0.001 * (d - off_);                   // over a few turns; each turn uses the value it began with
    has_off_ = true;
    double p = std::fmod(std::fmod(s.enc + off_turn_ - inc_deg_ / 2, 360.0) + 360.0, 360.0);   // progress past the cut
    bool done = false;
    if (started_ && !t_.empty() && t - t_.back() > MAX_GAP_S) {    // data lost mid-turn: the turn can't be fitted
        started_ = false;
        t_.clear(), p_.clear(), ang_.clear(), mm_.clear(), n_.clear(), line_.clear();
        p_max_ = -1;
    }
    if (p < 90 && p_max_ > 270) {                 // the head came round past the cut
        if (started_) done = finish(out);
        started_ = true;
        p = std::fmod(std::fmod(p - off_turn_ + off_, 360.0) + 360.0, 360.0);
        off_turn_ = off_;
        t_.clear(), p_.clear(), ang_.clear(), mm_.clear(), n_.clear(), line_.clear();
        p_max_ = p;
    } else if (p_max_ < 0 || (p > p_max_ && p - p_max_ < 90)) {   // forward (not a noisy step back across the cut)
        p_max_ = p;
    }
    if (started_ && std::isfinite(t)) {
        t_.push_back(t), p_.push_back(p), ang_.push_back(s.angle), mm_.push_back(s.mm);
        n_.push_back(s.n), line_.push_back(line);
    }
    return done;
}

bool TurnBuilder::finish(Turn& out) {
    size_t m = t_.size();
    if (m < 10) return false;
    if (retime)
        for (size_t i = 0; i < m; ++i) {
            double r = retime(n_[i], line_[i]);
            if (std::isfinite(r)) t_[i] = r;
        }
    std::vector<double> x(m), y(m);
    for (size_t i = 0; i < m; ++i) x[i] = p_[i] / inc_deg_ - 0.5, y[i] = t_[i] - t_[0];
    double dt, c;
    if (!line(x, y, dt, c) || !(dt > 0)) return false;
    double ss = 0;
    for (size_t i = 0; i < m; ++i) ss += (y[i] - (c + dt * x[i])) * (y[i] - (c + dt * x[i]));
    out = Turn{};
    out.stamp = t_[0] + c;
    out.time_increment = dt;
    out.scan_time = dt * bins;
    out.samples = (int)m;
    out.fit_rms = std::sqrt(ss / (double)m);
    out.ranges.assign((size_t)bins, std::numeric_limits<float>::quiet_NaN());
    for (size_t i = 0; i < m; ++i) {
        double phi = std::fmod(std::fmod(360.0 - ang_[i], 360.0) + 360.0, 360.0);     // counter-clockwise
        long b = std::lround(phi / inc_deg_) % bins;
        if (std::fabs((double)(bins - 1 - b) - x[i]) > bins / 2.0) {                // across the seam
            ++out.dropped;
            continue;
        }
        float r = mm_[i] > 0 ? (float)(mm_[i] / 1000.0) : std::numeric_limits<float>::infinity();
        float& cell = out.ranges[(size_t)b];
        if (std::isnan(cell) || r < cell) cell = r;
    }
    ++turns;
    dropped += out.dropped;
    return true;
}

// -- LidarStream -------------------------------------------------------------------------------------------------------

LidarStream::LidarStream(const StreamConfig& c)
    : cfg(c), dec(c.ans_type, c.period_s),
      clock(c.period_s, format(c.ans_type).bytes * 10.0 / c.baud + c.latency_s, c.window_s), turns(c.bins) {
    turns.reset();
    turns.retime = [this](int64_t n, int line) {
        return line == clock.lines ? clock.time(n) : std::numeric_limits<double>::quiet_NaN();
    };
}

void LidarStream::set_latency(double latency_s) {
    cfg.latency_s = latency_s;
    clock.set_delay(dec.fmt.bytes * 10.0 / cfg.baud + latency_s);
}

std::vector<Turn> LidarStream::feed(const uint8_t* d, size_t n, double t) {
    samples.clear(), times.clear(), anchors.clear();
    dec.feed(d, n, t, samples, anchors);
    std::vector<Turn> done;
    Turn out;
    size_t j = 0;
    auto take = [&](size_t upto) {
        for (; j < upto; ++j) {
            double ts = clock.time(samples[j].n);
            times.push_back(ts);
            if (turns.add(samples[j], ts, out, clock.lines)) done.push_back(out);
        }
    };
    for (const auto& a : anchors) {
        take(a.at);
        clock.add(a);
    }
    take(samples.size());
    return done;
}

}  // namespace omni::rplidar
