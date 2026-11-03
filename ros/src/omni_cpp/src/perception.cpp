#include "omni_cpp/perception.hpp"

namespace omni {

CameraModel CameraModel::from_config(const CameraConfig& c) {
    double f = (c.height / 2.0) / std::tan(c.fovy_deg * (kPiC / 180.0) / 2);
    return {c.pos, c.R, f, c.width / 2.0, c.height / 2.0, c.width, c.height};
}

M3 rot_zyx(double yaw, double pitch, double roll) {
    double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch), cr = std::cos(roll),
           sr = std::sin(roll);
    return {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp,     cp * sr,                cp * cr};
}

Perception::Perception(const RobotConfig& r, const TrackerConfig& tc, const DetectorConfig& dc)
    : robot(r), flight(r.ball_mass, r.ball_radius, r.ball_cd, r.air_density, r.air_viscosity), tcfg(tc), det(dc) {
    for (auto& c : robot.cameras) cams.push_back(CameraModel::from_config(c));
    tracker = ImageTracker(flight, tcfg, cams.at(0).f);
    R_ball = robot.ball_radius;
}

void Perception::set_play_area(double cx, double cy, double hx, double hy, double z_max, double margin) {
    tracker.acq_box = AcqBox{cx, cy, hx + margin, hy + margin, z_max};
}

void Perception::add_pose(double t, double x, double y, double yaw, double roll, double pitch) {
    if (!poses.empty() && t <= poses.back().t) return;
    poses.push_back({t, x, y, yaw, roll, pitch});
    if (poses.size() > 600) poses.pop_front();
}

PoseSample Perception::pose_at(double t) const {
    if (poses.empty()) return {t, 0, 0, 0, 0, 0};
    if (t <= poses.front().t) return poses.front();
    if (t >= poses.back().t) return poses.back();
    size_t lo = 0, hi = poses.size() - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (poses[mid].t <= t) lo = mid;
        else hi = mid;
    }
    const PoseSample &a = poses[lo], &b = poses[hi];
    double f = (t - a.t) / (b.t - a.t);
    double dyaw = py_mod(b.yaw - a.yaw + kPiC, 2 * kPiC) - kPiC;
    return {t, a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), a.yaw + f * dyaw, a.roll + f * (b.roll - a.roll),
            a.pitch + f * (b.pitch - a.pitch)};
}

std::vector<Cam> Perception::cams_at(double t) const {
    PoseSample p = pose_at(t);
    M3 Rb = rot_zyx(p.yaw, p.pitch, p.roll);
    V3 pb{p.x, p.y, robot.base_link_height};
    std::vector<Cam> out;
    for (auto& c : cams) out.push_back({pb + mul(Rb, c.pos_b), mul(Rb, c.R_b), c.f, c.cx, c.cy, R_ball});
    return out;
}

std::vector<Blob> Perception::process(double t_cap, const std::vector<const Frame*>& frs) {
    ++frames;
    auto cs = cams_at(t_cap);
    bool tracking = tracker.tracking();
    std::vector<Blob> blobs;
    int n_acq = det.cfg.acquire_candidates;
    if (!tracking && n_acq > 1) {
        // acquisition among several candidates: the tracker starts on the one that moves (design.md 31)
        std::vector<std::vector<Blob>> cands;
        for (size_t k = 0; k < frs.size(); ++k) {
            std::vector<Blob> c;
            if (frs[k])
                for (auto& d : det.candidates(*frs[k], n_acq)) c.push_back({(int)k, d.u, d.v, d.b, d.n, t_cap});
            cands.push_back(c);
        }
        blobs = tracker.init_candidates(t_cap, cands, cs);
        if (!blobs.empty()) {
            ++detections;
            stereo += blobs.size() == 2;
            t_last_det = t_cap;
        }
        return blobs;
    }
    for (size_t k = 0; k < frs.size(); ++k) {
        if (!frs[k]) continue;
        std::optional<DetBlob> d;
        if (tracking) {
            auto st = tracker.state_at(t_cap);
            const Cam& c = cs[k];
            auto img = sphere_image(mul_t(c.Rc, V3{st.first[0], st.first[1], st.first[2]} - c.c), c.Rb, c.f);
            if (img) {
                double pred[3] = {c.cx + img->du, c.cy + img->dv, img->b};
                d = det.detect(*frs[k], pred);
            } else {
                d = det.detect(*frs[k]);
            }
        } else {
            d = det.detect(*frs[k]);
        }
        if (d) blobs.push_back({(int)k, d->u, d->v, d->b, d->n, t_cap});
    }
    if (!blobs.empty()) {
        ++detections;
        stereo += blobs.size() == 2;
        t_last_det = t_cap;
        tracker.update_blobs(t_cap, blobs, cs);
    }
    if (tracker.tracking()) {
        // a track the ball no longer feeds (caught, landed, out of view), or one that stopped: drop it
        const auto& x = tracker.x;
        if (t_cap - t_last_det > tcfg.max_gap || std::sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5]) < 0.8)
            tracker.reset();
    }
    return blobs;
}

std::optional<TrackView> Perception::view() const {
    if (!tracker.tracking()) return std::nullopt;
    return TrackView(flight, tcfg.accel_sigma, tracker.t, tracker.x, tracker.P, tracker.n, ready());
}

}  // namespace omni
