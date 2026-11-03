#include "omni_cpp/detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace omni {

bool is_ball_px(int r, int g, int b, const DetectorConfig& c) {
    int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
    int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    int chroma = mx - mn;
    float min_val = (float)c.min_val, min_sat = (float)c.min_sat;
    if (!((float)mx >= min_val * 255.0f)) return false;
    if (!((float)chroma >= min_sat * (float)(mx > 1 ? mx : 1))) return false;
    float cz = (float)(chroma > 1 ? chroma : 1);
    float h;
    if (mx == r) {
        h = 60.0f * ((float)(g - b) / cz);
        h = std::fmod(h, 360.0f);
        if (h < 0.0f) h += 360.0f;
    } else if (mx == g) {
        h = 60.0f * ((float)(b - r) / cz + 2.0f);
    } else {
        h = 60.0f * ((float)(r - g) / cz + 4.0f);
    }
    return h >= (float)c.hue_lo && h <= (float)c.hue_hi;
}

RgbFrame::RgbFrame(const uint8_t* data, int w, int h, long row, long px, bool bgr)
    : p_(data), row_(row), px_(px), ir_(bgr ? 2 : 0), ig_(1), ib_(bgr ? 0 : 2) {
    width = w, height = h;
}

void RgbFrame::mask(const DetectorConfig& c, int u0, int v0, int u1, int v1, std::vector<uint8_t>& out) const {
    int w = u1 - u0, h = v1 - v0;
    out.assign((size_t)w * h, 0);
    for (int y = 0; y < h; ++y) {
        const uint8_t* q = p_ + (long)(v0 + y) * row_ + (long)u0 * px_;
        for (int x = 0; x < w; ++x, q += px_) out[(size_t)y * w + x] = is_ball_px(q[ir_], q[ig_], q[ib_], c);
    }
}

void RgbFrame::coarse_mask(const DetectorConfig& c, int s, std::vector<uint8_t>& out, int& rows, int& cols) const {
    rows = (height + s - 1) / s, cols = (width + s - 1) / s;
    out.assign((size_t)rows * cols, 0);
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < cols; ++x) {
            const uint8_t* q = p_ + (long)(y * s) * row_ + (long)(x * s) * px_;
            out[(size_t)y * cols + x] = is_ball_px(q[ir_], q[ig_], q[ib_], c);
        }
}

SparseFrame::SparseFrame(int w, int h, std::vector<int64_t> b, std::vector<int64_t> cl)
    : ball(std::move(b)), clutter(std::move(cl)) {
    width = w, height = h;
}

void SparseFrame::mask(const DetectorConfig&, int u0, int v0, int u1, int v1, std::vector<uint8_t>& out) const {
    int w = u1 - u0, h = v1 - v0;
    out.assign((size_t)w * h, 0);
    for (const auto* ids : {&ball, &clutter})
        for (int64_t i : *ids) {
            int64_t y = i / width, x = i % width;
            if (y >= v0 && y < v1 && x >= u0 && x < u1) out[(size_t)(y - v0) * w + (size_t)(x - u0)] = 1;
        }
}

void SparseFrame::coarse_mask(const DetectorConfig&, int s, std::vector<uint8_t>& out, int& rows, int& cols) const {
    rows = (height + s - 1) / s, cols = (width + s - 1) / s;
    out.assign((size_t)rows * cols, 0);
    for (const auto* ids : {&ball, &clutter})
        for (int64_t i : *ids) {
            int64_t y = i / width, x = i % width;
            if (y % s == 0 && x % s == 0 && y / s < rows && x / s < cols) out[(size_t)(y / s) * cols + (size_t)(x / s)] = 1;
        }
}

namespace {

// 4-connected components of an h x w mask, labelled in raster order of their first pixel (scipy.ndimage.label's
// numbering); on(k, x, y) for each pixel in flood order, on(-1, 0, 0) when a component starts (native/blob.cpp)
template <typename F>
int label(const std::vector<uint8_t>& m, int h, int w, std::vector<int32_t>& lab, std::vector<int32_t>& stack, F on) {
    size_t n = (size_t)h * w;
    lab.assign(n, 0);
    if (stack.size() < n) stack.resize(n);
    int k = 0;
    for (size_t i0 = 0; i0 < n; ++i0) {
        if (!m[i0] || lab[i0]) continue;
        ++k;
        size_t top = 0;
        stack[top++] = (int32_t)i0;
        lab[i0] = k;
        on(-1, 0, 0);
        while (top) {
            int32_t i = stack[--top];
            int y = i / w, x = i % w;
            on(k, x, y);
            const int nb[4][2] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
            for (auto& q : nb) {
                if (q[0] < 0 || q[1] < 0 || q[0] >= w || q[1] >= h) continue;
                int32_t j = q[1] * w + q[0];
                if (m[j] && !lab[j]) {
                    lab[j] = k;
                    stack[top++] = j;
                }
            }
        }
    }
    return k;
}

}  // namespace

std::vector<DetBlob> Detector::components(const Frame& fr, double fu0, double fv0, double fu1, double fv1) {
    int u0 = std::max(0, (int)fu0), v0 = std::max(0, (int)fv0);
    int u1 = std::min(fr.width, (int)std::ceil(fu1)), v1 = std::min(fr.height, (int)std::ceil(fv1));
    std::vector<DetBlob> out;
    if (u1 <= u0 || v1 <= v0) return out;
    int w = u1 - u0, h = v1 - v0, W = fr.width, H = fr.height;
    fr.mask(cfg, u0, v0, u1, v1, mask_);
    if (std::find(mask_.begin(), mask_.end(), 1) == mask_.end()) return out;
    double su = 0, sv = 0, suu = 0, suv = 0, svv = 0;
    long cnt = 0;
    bool border = false;
    auto flush = [&]() {
        if (cnt == 0) return;
        double uc = su / cnt, vc = sv / cnt;
        double cuu = suu / cnt - uc * uc, cuv = suv / cnt - uc * vc, cvv = svv / cnt - vc * vc;
        double tr = 0.5 * (cuu + cvv), det = cuu * cvv - cuv * cuv;
        double disc = tr * tr - det;
        double lam0 = tr - std::sqrt(disc > 0 ? disc : 0);
        double b = 2.0 * std::sqrt(lam0 - 1.0 / 12.0 > 1e-6 ? lam0 - 1.0 / 12.0 : 1e-6);
        if (cnt >= cfg.min_blob_px && !border) out.push_back({uc, vc, cfg.size_gain != 1.0 ? b * cfg.size_gain : b, (int)cnt});
        su = sv = suu = suv = svv = 0;
        cnt = 0;
        border = false;
    };
    label(mask_, h, w, lab_, stack_, [&](int k, int x, int y) {
        if (k < 0) {
            flush();
            return;
        }
        double u = x + u0 + 0.5, v = y + v0 + 0.5;
        su += u, sv += v, suu += u * u, suv += u * v, svv += v * v;
        ++cnt;
        if (u < 1 || v < 1 || u > W - 1 || v > H - 1) border = true;
    });
    flush();
    return out;
}

std::vector<CoarseComp> Detector::coarse(const Frame& fr) {
    int rows, cols;
    fr.coarse_mask(cfg, cfg.coarse_stride, mask_, rows, cols);
    std::vector<CoarseComp> out;
    label(mask_, rows, cols, lab_, stack_, [&](int k, int x, int y) {
        if (k < 0) {
            out.push_back({(int)out.size() + 1, 0, 1 << 30, -1, 1 << 30, -1});
            return;
        }
        CoarseComp& o = out.back();
        ++o.size;
        o.x0 = std::min(o.x0, x), o.x1 = std::max(o.x1, x), o.y0 = std::min(o.y0, y), o.y1 = std::max(o.y1, y);
    });
    return out;
}

std::vector<DetBlob> Detector::window(const Frame& fr, const CoarseComp& c) {
    int s = cfg.coarse_stride;
    return components(fr, c.x0 * s - s - 2, c.y0 * s - s - 2, (c.x1 + 1) * s + s + 2, (c.y1 + 1) * s + s + 2);
}

std::optional<DetBlob> Detector::detect(const Frame& fr, const double* pred) {
    if (pred) {
        double pu = pred[0], pv = pred[1], pr = pred[2];
        double half = cfg.window_scale * std::max(pr, 1.0) + cfg.window_px;
        auto comps = components(fr, pu - half, pv - half, pu + half, pv + half);
        if (!comps.empty()) {
            size_t best = 0;
            double bd = 0;
            for (size_t i = 0; i < comps.size(); ++i) {
                double d = std::pow(comps[i].u - pu, 2.0) + std::pow(comps[i].v - pv, 2.0);
                if (i == 0 || d < bd) best = i, bd = d;
            }
            return comps[best];
        }
    }
    ++n_coarse;
    auto cs = coarse(fr);
    if (cs.empty()) return std::nullopt;
    // sorted((size, k), reverse=True)[:max_candidates]: the largest first, ties to the later label
    std::vector<size_t> order(cs.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return cs[a].size != cs[b].size ? cs[a].size > cs[b].size : cs[a].k > cs[b].k;
    });
    std::optional<DetBlob> best;
    for (size_t r = 0; r < order.size() && (int)r < cfg.max_candidates; ++r)
        for (auto& c : window(fr, cs[order[r]]))
            if (!best || c.n > best->n) best = c;
    return best;
}

std::vector<DetBlob> Detector::candidates(const Frame& fr, int n) {
    ++n_coarse;
    auto cs = coarse(fr);
    std::vector<DetBlob> out;
    if (cs.empty()) return out;
    std::vector<size_t> order(cs.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return cs[a].size > cs[b].size; });
    size_t m = std::min(order.size(), (size_t)std::max(n, cfg.max_candidates));
    for (size_t r = 0; r < m; ++r)
        for (auto& c : window(fr, cs[order[r]])) {
            bool far = true;
            for (auto& o : out)
                if (!(std::pow(c.u - o.u, 2.0) + std::pow(c.v - o.v, 2.0) > 4.0)) {
                    far = false;
                    break;
                }
            if (far) out.push_back(c);
        }
    std::stable_sort(out.begin(), out.end(), [](const DetBlob& a, const DetBlob& b) { return a.n > b.n; });
    if ((int)out.size() > n) out.resize(n);
    return out;
}

}  // namespace omni
