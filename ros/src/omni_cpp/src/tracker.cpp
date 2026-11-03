#include "omni_cpp/tracker.hpp"

#include <algorithm>
#include <limits>

namespace omni {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double chi2_999(size_t n) {
    static const double t[] = {0, 10.8, 13.8, 16.3, 18.5, 20.5, 22.5};
    return t[n];
}

V3 head3(const std::vector<double>& x) { return {x[0], x[1], x[2]}; }
V3 tail3(const std::vector<double>& x) { return {x[3], x[4], x[5]}; }

// the ray of pixel (u, v) in a camera, unit length (Rc @ [(u - cx) / f, -(v - cy) / f, -1])
V3 ray(const Cam& c, double u, double v) {
    V3 d = mul(c.Rc, V3{(u - c.cx) / c.f, -(v - c.cy) / c.f, -1.0});
    return d / norm(d);
}

}  // namespace

std::optional<SphereImage> sphere_image(const V3& s, double R, double f) {
    double d = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    if (d <= R * 1.001 || s[2] >= 0) return std::nullopt;
    double cth = -s[2] / d;
    double sa2 = std::pow(R / d, 2.0);
    double den = cth * cth - sa2;
    if (den <= 1e-6) return std::nullopt;
    double rxy = std::hypot(s[0], s[1]);
    double sth = rxy / d;
    double rho = f * sth * cth / den;
    double b = f * std::sqrt(sa2) / std::sqrt(den);
    if (rxy < 1e-12) return SphereImage{0.0, 0.0, b};
    return SphereImage{rho * s[0] / rxy, -rho * s[1] / rxy, b};
}

void propagate(const BallFlight& fl, double q, std::vector<double>& x, Mat& P, double dt) {
    int steps = std::max(1, (int)std::ceil(std::fabs(dt) / 0.01));
    double h = dt / steps;
    double q11 = q * (std::pow(std::fabs(h), 3.0) / 3), q12 = q * (std::pow(h, 2.0) / 2), q22 = q * std::fabs(h);
    Mat Q(6, 6);
    for (int i = 0; i < 3; ++i) Q(i, i) = q11, Q(i, i + 3) = q12, Q(i + 3, i) = q12, Q(i + 3, i + 3) = q22;
    Mat F = Mat::eye(6);
    for (int i = 0; i < 3; ++i) F(i, i + 3) = h;
    for (int k = 0; k < steps; ++k) {
        V3 p = head3(x), v = tail3(x);
        V3 a1 = fl.accel(v);
        V3 a2 = fl.accel(v + h / 2 * a1);
        V3 a3 = fl.accel(v + h / 2 * a2);
        V3 a4 = fl.accel(v + h * a3);
        V3 v_new = v + h / 6 * (a1 + 2 * a2 + 2 * a3 + a4);
        V3 p_new = p + h / 6 * (v + 2 * (v + h / 2 * a1) + 2 * (v + h / 2 * a2) + (v + h * a3));
        double s = std::sqrt(dot(v, v));
        double kk = fl.k_lin + fl.k_quad * s;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double Jij = i == j ? -kk : -0.0;
                if (s > 1e-6) Jij -= fl.k_quad * (v[i] * v[j]) / s;
                F(3 + i, 3 + j) = (i == j ? 1.0 : 0.0) + h * Jij;
            }
        P = F * P * F.t() + Q;
        x = {p_new[0], p_new[1], p_new[2], v_new[0], v_new[1], v_new[2]};
    }
}

ImageTracker::ImageTracker(const BallFlight& fl_, const TrackerConfig& c, double f_px) : fl(fl_), cfg(c) {
    q = std::pow(c.accel_sigma, 2.0);
    reset();
    c0 = c.sigma_c0 ? *c.sigma_c0
                    : std::hypot(0.15, f_px * (c.calib_rot_deg * (kPiC / 180.0)) * 1.2 + f_px * c.calib_f_rel * 0.3);
    size_rel = c.sigma_size_rel ? *c.sigma_size_rel : 1.2 * std::hypot(c.ball_size_rel, c.calib_f_rel) + 0.002;
}

void ImageTracker::reset() {
    has_x = false;
    x.clear();
    P = Mat();
    t = 0;
    n = 0;
    has_cand = false;
    cand_pts.reset();
    miss_streak = 0;
}

std::pair<std::vector<double>, Mat> ImageTracker::state_at(double t_at) const {
    std::vector<double> xx = x;
    Mat PP = P;
    propagate(fl, q, xx, PP, t_at - t);
    return {xx, PP};
}

std::optional<std::vector<double>> ImageTracker::h(const V3& pos, const std::vector<const Cam*>& cams,
                                                   const std::vector<bool>& use_size) const {
    std::vector<double> out;
    for (size_t k = 0; k < cams.size(); ++k) {
        const Cam& c = *cams[k];
        auto img = sphere_image(mul_t(c.Rc, pos - c.c), c.Rb, c.f);
        if (!img) return std::nullopt;
        out.push_back(c.cx + img->du);
        out.push_back(c.cy + img->dv);
        if (use_size[k]) out.push_back(img->b);
    }
    return out;
}

Mat ImageTracker::noise(const std::vector<const Blob*>& blobs, const std::vector<bool>& use_size) const {
    std::vector<double> d;
    for (size_t k = 0; k < blobs.size(); ++k) {
        const Blob& bl = *blobs[k];
        double r = std::max(bl.b_truthy() ? *bl.b : std::sqrt(bl.n_px / kPiC), 1.0);
        double sc = std::hypot(c0 + cfg.sigma_c1 / std::sqrt(r), cfg.ego_sigma * r / 0.037);
        d.push_back(sc * sc);
        d.push_back(sc * sc);
        if (use_size[k]) {
            double sb = cfg.sigma_b0 + cfg.sigma_b1 / std::sqrt(r);
            sb = std::hypot(sb, size_rel * *bl.b);
            d.push_back(sb * sb);
        }
    }
    Mat m((int)d.size(), (int)d.size());
    for (size_t i = 0; i < d.size(); ++i) m((int)i, (int)i) = d[i];
    return m;
}

std::optional<V3> ImageTracker::point(const std::vector<Blob>& blobs, const std::vector<Cam>& cams) const {
    struct Ray {
        V3 c, d;
        std::optional<double> rng;
    };
    std::vector<Ray> rays;
    for (auto& bl : blobs) {
        const Cam& c = cams[bl.cam];
        Ray r{c.c, ray(c, bl.u, bl.v), std::nullopt};
        if (bl.b_truthy()) r.rng = c.Rb * std::sqrt(c.f * c.f + *bl.b * *bl.b) / *bl.b;
        rays.push_back(r);
    }
    bool use_size = cfg.mode != "stereo";
    if (rays.size() >= 2 && (cfg.mode == "fused" || cfg.mode == "stereo")) {
        const Ray &r1 = rays[0], &r2 = rays[1];
        V3 w0 = r1.c - r2.c;
        double a = dot(r1.d, r1.d), bb = dot(r1.d, r2.d), cc = dot(r2.d, r2.d), dd = dot(r1.d, w0),
               ee = dot(r2.d, w0);
        double den = a * cc - bb * bb;
        if (den > 1e-9) {
            double s1 = (bb * ee - cc * dd) / den, s2 = (a * ee - bb * dd) / den;
            V3 q1 = r1.c + s1 * r1.d, q2 = r2.c + s2 * r2.d;
            if (s1 > 0.1 && s2 > 0.1 && norm(q1 - q2) < 0.03)
                if (!use_size || !r1.rng || std::fabs(s1 - *r1.rng) < 0.25 * s1) return 0.5 * (q1 + q2);
        }
        return std::nullopt;
    }
    if (use_size && rays.size() == 1 && rays[0].rng) return rays[0].c + *rays[0].rng * rays[0].d;
    return std::nullopt;
}

Mat ImageTracker::point_cov(const V3& pos, const std::vector<Blob>& blobs, const std::vector<Cam>& cams) const {
    V3 c_mid = cams[blobs[0].cam].c;
    if (blobs.size() > 1) {
        V3 s = c_mid;
        for (size_t k = 1; k < blobs.size(); ++k) s = s + cams[blobs[k].cam].c;
        c_mid = s / (double)blobs.size();
    }
    V3 los = pos - c_mid;
    double rng = norm(los);
    V3 e = los / rng;
    double f = cams[blobs[0].cam].f;
    double mn = kInf;
    for (auto& bl : blobs) mn = std::min(mn, bl.b_truthy() ? *bl.b : 1.0);
    double r_px = std::max(1.0, mn);
    double s_px = c0 + cfg.sigma_c1 / std::sqrt(r_px);
    double s_lat = rng * s_px / f;
    double inv = 0.0;
    if (blobs.size() >= 2 && (cfg.mode == "fused" || cfg.mode == "stereo")) {
        double base = norm(cams[blobs[0].cam].c - cams[blobs[1].cam].c);
        inv += 1.0 / std::pow(rng * rng / (f * base) * std::sqrt(2.0) * s_px, 2.0);
    }
    if (cfg.mode != "stereo" && blobs[0].b_truthy()) {
        double sb = std::hypot(cfg.sigma_b0 + cfg.sigma_b1 / std::sqrt(r_px), size_rel * r_px);
        inv += 1.0 / std::pow(rng * sb / r_px, 2.0);
    }
    double s_dep = inv > 0 ? 1.0 / std::sqrt(inv) : rng;
    s_dep = std::max(s_dep, s_lat);
    double a = std::pow(s_lat, 2.0), b = std::pow(s_dep, 2.0) - std::pow(s_lat, 2.0);
    Mat m(3, 3);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m(i, j) = (i == j ? a : 0.0) + b * (e[i] * e[j]);
    return m;
}

void ImageTracker::start(const V3& pos, const V3& v0, double dt, const std::vector<Blob>& blobs,
                         const std::vector<Cam>& cams, double t0) {
    x = {pos[0], pos[1], pos[2], v0[0], v0[1], v0[2]};
    has_x = true;
    P = Mat(6, 6);
    if (cfg.honest_init) {
        Mat Pp = point_cov(pos, blobs, cams);
        double dt2 = std::pow(dt, 2.0);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                P(i, j) = Pp(i, j);
                P(i, j + 3) = P(i + 3, j) = Pp(i, j) / dt;
                P(i + 3, j + 3) = 2 * Pp(i, j) / dt2;
            }
    } else {
        for (int i = 0; i < 3; ++i) P(i, i) = 0.05 * 0.05, P(i + 3, i + 3) = 4.0;
    }
    t = t0, n = 1;
    miss_streak = 0;
}

bool ImageTracker::init_from(const std::vector<Blob>& blobs, const std::vector<Cam>& cams) {
    auto pos = point(blobs, cams);
    double t0 = blobs[0].t;
    if (!pos) return false;
    bool had = has_cand;
    double pt = cand_t;
    V3 pp = cand_pos;
    has_cand = true, cand_t = t0, cand_pos = *pos;
    if (!had || !(0.0 < t0 - pt && t0 - pt < 0.05) || norm(*pos - pp) > 15.0 * (t0 - pt) + 0.05) return false;
    double dt = t0 - pt;
    V3 v0 = (*pos - pp) / dt;
    if (norm(v0) < cfg.min_init_speed) return false;
    start(*pos, v0, dt, blobs, cams, t0);
    return true;
}

double ImageTracker::reproj_px(const V3& q, const Blob& bl, const std::vector<Cam>& cams) {
    const Cam& c = cams[bl.cam];
    auto img = sphere_image(mul_t(c.Rc, q - c.c), c.Rb, c.f);
    return img ? std::hypot(c.cx + img->du - bl.u, c.cy + img->dv - bl.v) : kInf;
}

double ImageTracker::ego_px(const std::vector<Cam>& cams0, const std::vector<Cam>& cams1, double d_ref) {
    double out = 0.0;
    for (size_t k = 0; k < std::min(cams0.size(), cams1.size()); ++k) {
        const M3 &R0 = cams0[k].Rc, &R1 = cams1[k].Rc;
        double tr = 0;                                  // trace(R0.T @ R1), as numpy sums it
        for (int j = 0; j < 3; ++j) tr += R0[j] * R1[j] + R0[3 + j] * R1[3 + j] + R0[6 + j] * R1[6 + j];
        double ang = std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0)));
        out = std::max(out, cams0[k].f * (ang + norm(cams1[k].c - cams0[k].c) / d_ref));
    }
    return out;
}

bool ImageTracker::still(const std::vector<Blob>& bls, const std::vector<V3>& prev_pts,
                         const std::vector<std::vector<Blob>>& prev_blobs, const std::vector<Cam>& cams,
                         double tol) const {
    for (auto& bl : bls)
        if (bl.cam < (int)prev_blobs.size())
            for (auto& pb : prev_blobs[bl.cam])
                if (std::hypot(pb.u - bl.u, pb.v - bl.v) <= tol) return true;
    for (auto& q : prev_pts) {
        bool all = true;
        for (auto& bl : bls)
            if (!(reproj_px(q, bl, cams) <= tol)) {
                all = false;
                break;
            }
        if (all) return true;
    }
    return false;
}

std::vector<Blob> ImageTracker::init_candidates(double t0, const std::vector<std::vector<Blob>>& cands,
                                                const std::vector<Cam>& cams) {
    static const std::vector<Blob> none;
    const std::vector<Blob>& L = cands.size() > 0 ? cands[0] : none;
    const std::vector<Blob>& R = cands.size() > 1 ? cands[1] : none;
    std::vector<std::pair<V3, std::vector<Blob>>> pts;
    if (!L.empty() && !R.empty()) {
        // every left x right pair at once: rays within 3 cm in front of both cameras, then triangulated exactly
        std::vector<V3> D1, D2;
        for (auto& bl : L) D1.push_back(ray(cams[0], bl.u, bl.v));
        for (auto& bl : R) D2.push_back(ray(cams[1], bl.u, bl.v));
        V3 w0 = cams[0].c - cams[1].c;
        for (size_t i = 0; i < L.size(); ++i) {
            double d = dot(D1[i], w0);
            for (size_t j = 0; j < R.size(); ++j) {
                double b = dot(D1[i], D2[j]), e = dot(D2[j], w0);
                double den = std::max(1.0 - b * b, 1e-9);
                double s1 = (b * e - d) / den, s2 = (e - b * d) / den;
                V3 g = w0 + s1 * D1[i] - s2 * D2[j];
                double gap = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
                double bL = L[i].b ? *L[i].b : 0.0, bR = R[j].b ? *R[j].b : 0.0;
                double nL = L[i].n_px, nR = R[j].n_px;
                bool same = std::fabs(bL - bR) <= 0.3 * std::max(bL, bR) + 0.5 &&
                            std::max(nL, nR) <= 2.0 * std::min(nL, nR) + 8;
                if (s1 > 0.1 && s2 > 0.1 && gap < 0.03 && same) {
                    std::vector<Blob> pair{L[i], R[j]};
                    auto p = point(pair, cams);
                    if (p) pts.emplace_back(*p, pair);
                }
            }
        }
    } else {
        for (auto& bl : (!L.empty() ? L : R))
            if (bl.b_truthy()) {
                std::vector<Blob> one{bl};
                auto p = point(one, cams);
                if (p) pts.emplace_back(*p, one);
            }
    }
    if (acq_box) {
        const AcqBox& bx = *acq_box;
        std::vector<std::pair<V3, std::vector<Blob>>> in;
        for (auto& pb : pts) {
            const V3& p = pb.first;
            if (std::fabs(p[0] - bx.cx) <= bx.hx && std::fabs(p[1] - bx.cy) <= bx.hy && 0.0 < p[2] && p[2] < bx.z_max)
                in.push_back(pb);
        }
        pts = in;
    }
    std::optional<CandFrame> prev = std::move(cand_pts);
    cand_pts = CandFrame{t0, pts, {L, R}, cams};
    if (!prev || prev->pts.empty() || pts.empty() || !(0.0 < t0 - prev->t && t0 - prev->t < 0.05)) return {};
    double dt = t0 - prev->t;
    std::vector<V3> prev_pts;
    for (auto& pb : prev->pts) prev_pts.push_back(pb.first);
    double tol = cfg.acq_still_px + cfg.acq_still_ego * ego_px(prev->cams, cams);
    bool found = false;
    int best_size = 0;
    V3 best_p{}, best_v{};
    std::vector<Blob> best_bls;
    for (auto& pb : pts) {
        const V3& p = pb.first;
        const std::vector<Blob>& bls = pb.second;
        if (still(bls, prev_pts, prev->lr, cams, tol)) continue;
        bool has_src = false;
        double src_step = 0;
        V3 src_q{};
        for (auto& q0 : prev_pts) {
            double step = norm(p - q0);
            if (step > 15.0 * dt + 0.05 || step / dt < cfg.min_init_speed) continue;
            bool on_ray = false;
            for (auto& bl : bls)
                if (reproj_px(q0, bl, cams) <= cfg.acq_still_px) {
                    on_ray = true;
                    break;
                }
            if (on_ray) continue;
            if (!has_src || step < src_step) has_src = true, src_step = step, src_q = q0;
        }
        if (!has_src) continue;
        int size = 0;
        for (auto& bl : bls) size += bl.n_px;
        if (!found || size > best_size) found = true, best_size = size, best_p = p, best_v = (p - src_q) / dt,
                                        best_bls = bls;
    }
    if (!found) return {};
    start(best_p, best_v, dt, best_bls, cams, t0);
    return best_bls;
}

bool ImageTracker::gate_one(const Blob& bl, const Cam& cam, bool use_b) const {
    std::vector<double> z{bl.u, bl.v};
    if (use_b) z.push_back(*bl.b);
    std::vector<const Cam*> cs{&cam};
    std::vector<bool> us{use_b};
    V3 p = head3(x);
    auto h0 = h(p, cs, us);
    if (!h0) return false;
    int m = (int)z.size();
    Mat H(m, 3);
    for (int i = 0; i < 3; ++i) {
        V3 dp{0, 0, 0};
        dp[i] = 1e-4;
        auto hi = h(p + dp, cs, us);
        if (!hi) return false;
        for (int r = 0; r < m; ++r) H(r, i) = ((*hi)[r] - (*h0)[r]) / 1e-4;
    }
    std::vector<const Blob*> bs{&bl};
    Mat S = H * P.block(0, 0, 3, 3) * H.t() + noise(bs, us);
    std::vector<double> y(m);
    for (int r = 0; r < m; ++r) y[r] = z[r] - (*h0)[r];
    auto sy = S.solve(y);
    double d2 = 0;
    for (int r = 0; r < m; ++r) d2 += y[r] * sy[r];
    return d2 < chi2_999(z.size());
}

void ImageTracker::update_blobs(double t_at, const std::vector<Blob>& blobs, const std::vector<Cam>& cams) {
    if (!has_x) {
        init_from(blobs, cams);
        return;
    }
    if (t_at < t) return;
    propagate(fl, q, x, P, t_at - t);
    t = t_at;
    std::vector<const Blob*> sel;
    std::vector<const Cam*> cs;
    std::vector<bool> size;
    for (auto& bl : blobs) {
        sel.push_back(&bl);
        cs.push_back(&cams[bl.cam]);
        size.push_back(bl.b.has_value() && (cfg.mode == "fused" || cfg.mode == "mono"));
    }
    if (sel.empty()) return;
    if (cfg.gate) {
        std::vector<const Blob*> ks;
        std::vector<const Cam*> kc;
        std::vector<bool> ku;
        for (size_t k = 0; k < sel.size(); ++k) {
            std::vector<bool> tries = size[k] ? std::vector<bool>{true, false} : std::vector<bool>{false};
            bool kept = false;
            for (bool use_b : tries)
                if (gate_one(*sel[k], *cs[k], use_b)) {
                    ks.push_back(sel[k]), kc.push_back(cs[k]), ku.push_back(use_b);
                    kept = true;
                    break;
                }
            if (!kept) ++rejected;
        }
        if (ks.empty()) {
            if (++miss_streak >= 6) {
                has_x = false;
                x.clear();
                has_cand = false;
            }
            return;
        }
        miss_streak = 0;
        sel = ks, cs = kc, size = ku;
    }
    std::vector<double> z;
    for (size_t k = 0; k < sel.size(); ++k) {
        z.push_back(sel[k]->u);
        z.push_back(sel[k]->v);
        if (size[k]) z.push_back(*sel[k]->b);
    }
    V3 p = head3(x);
    auto h0 = h(p, cs, size);
    if (!h0) return;
    int m = (int)z.size();
    Mat H(m, 6);
    for (int i = 0; i < 3; ++i) {
        V3 dp{0, 0, 0};
        dp[i] = 1e-4;
        auto hi = h(p + dp, cs, size);
        if (!hi) return;
        for (int r = 0; r < m; ++r) H(r, i) = ((*hi)[r] - (*h0)[r]) / 1e-4;
    }
    Mat Rn = noise(sel, size);
    Mat Ht = H.t();
    Mat S = H * P * Ht + Rn;
    Mat K = P * Ht * S.inv();
    std::vector<double> y(m);
    for (int r = 0; r < m; ++r) y[r] = z[r] - (*h0)[r];
    auto dx = K * y;
    for (int i = 0; i < 6; ++i) x[i] = x[i] + dx[i];
    P = (Mat::eye(6) - K * H) * P;
    ++n;
}

void TrackView::estimate(double t_at, V3& p, V3& v, Mat* P_out) const {
    std::vector<double> xx = x;
    Mat PP = P;
    propagate(f, q, xx, PP, t_at - t);
    p = head3(xx), v = tail3(xx);
    if (P_out) *P_out = PP;
}

std::optional<TrackCrossing> TrackView::crossing(double t_at, double z_plane) const {
    V3 p, v;
    Mat PP;
    estimate(t_at, p, v, &PP);
    auto hit = f.crossing_state(p, v, z_plane);
    if (!hit) return std::nullopt;
    Mat J(2, 6);
    for (int k = 0; k < 6; ++k) {
        double hh = k < 3 ? 1e-3 : 1e-2;
        V3 dp{0, 0, 0}, dv{0, 0, 0};
        (k < 3 ? dp[k] : dv[k - 3]) = hh;
        auto hk = f.crossing_state(p + dp, v + dv, z_plane);
        if (hk) J(0, k) = (hk->pos[0] - hit->pos[0]) / hh, J(1, k) = (hk->pos[1] - hit->pos[1]) / hh;
    }
    Mat C = J * PP * J.t();
    return TrackCrossing{hit->pos, hit->vel, hit->t, {C(0, 0), C(0, 1), C(1, 0), C(1, 1)}};
}

}  // namespace omni
