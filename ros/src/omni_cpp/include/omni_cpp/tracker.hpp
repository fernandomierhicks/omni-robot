// The ball tracker (design.md 6, 20-22, 30, 31): C++ port of omni_catch/tracker.py, a drag-aware ballistic Kalman
// filter whose measurements are blobs in pixels, with the acquisition among candidates. The Python stays the
// reference (its notes explain each test); test/test_cores.py runs both on the same frames and compares.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "omni_cpp/catch_math.hpp"
#include "omni_cpp/flight.hpp"

namespace omni {

struct TrackerConfig {
    std::string mode = "fused";                 // fused (stereo + size), stereo, mono
    double accel_sigma = 0.5;
    std::optional<double> sigma_c0;             // centroid noise c0 + c1 / sqrt(r_px); none = from the calibration
    double sigma_c1 = 0.4;
    double sigma_b0 = 0.05, sigma_b1 = 0.45;
    std::optional<double> sigma_size_rel;
    double calib_rot_deg = 0.10, calib_f_rel = 0.002, ball_size_rel = 0.015, ego_sigma = 0.001;
    int min_frames = 6;
    bool gate = true, honest_init = true;
    double min_init_speed = 1.5, max_gap = 0.5, acq_still_px = 1.0, acq_still_ego = 1.0;
};

// a camera where the robot was: position, rotation (columns = its axes in the world; it looks along -z), focal
// length and principal point (px), the ball radius the filter assumes
struct Cam {
    V3 c;
    M3 Rc;
    double f, cx, cy, Rb;
};

struct Blob {
    int cam;
    double u, v;
    std::optional<double> b;          // minor semi-axis (px); none = no usable size
    int n_px;
    double t;
    bool b_truthy() const { return b && *b != 0.0; }     // Python's `bl.b or ...`
};

struct SphereImage {
    double du, dv, b;
};
std::optional<SphereImage> sphere_image(const V3& s, double R, double f);

// RK4 on the state, first-order on the covariance, in steps of at most 10 ms (tracker.propagate)
void propagate(const BallFlight& fl, double q, std::vector<double>& x, Mat& P, double dt);

struct AcqBox {
    double cx, cy, hx, hy, z_max;
};

class ImageTracker {
public:
    ImageTracker() = default;
    ImageTracker(const BallFlight& fl, const TrackerConfig& cfg, double f_px = 571.0);

    void reset();
    bool tracking() const { return has_x; }
    std::pair<std::vector<double>, Mat> state_at(double t) const;
    bool init_from(const std::vector<Blob>& blobs, const std::vector<Cam>& cams);
    // cands[k]: camera k's blobs, largest first; the blobs the track started from, or none
    std::vector<Blob> init_candidates(double t, const std::vector<std::vector<Blob>>& cands,
                                      const std::vector<Cam>& cams);
    void update_blobs(double t, const std::vector<Blob>& blobs, const std::vector<Cam>& cams);

    BallFlight fl;
    TrackerConfig cfg;
    double q = 0.25;
    bool has_x = false;
    std::vector<double> x;           // (x, y, z, vx, vy, vz) while tracking
    Mat P;
    double t = 0;
    int n = 0;
    std::optional<AcqBox> acq_box;
    int rejected = 0, miss_streak = 0;
    double c0 = 0, size_rel = 0;

private:
    std::optional<std::vector<double>> h(const V3& pos, const std::vector<const Cam*>& cams,
                                         const std::vector<bool>& use_size) const;
    Mat noise(const std::vector<const Blob*>& blobs, const std::vector<bool>& use_size) const;
    std::optional<V3> point(const std::vector<Blob>& blobs, const std::vector<Cam>& cams) const;
    Mat point_cov(const V3& pos, const std::vector<Blob>& blobs, const std::vector<Cam>& cams) const;
    bool gate_one(const Blob& bl, const Cam& cam, bool use_b) const;
    void start(const V3& pos, const V3& v0, double dt, const std::vector<Blob>& blobs, const std::vector<Cam>& cams,
               double t0);
    static double reproj_px(const V3& q, const Blob& bl, const std::vector<Cam>& cams);
    static double ego_px(const std::vector<Cam>& cams0, const std::vector<Cam>& cams1, double d_ref = 3.0);
    bool still(const std::vector<Blob>& bls, const std::vector<V3>& prev_pts,
               const std::vector<std::vector<Blob>>& prev_blobs, const std::vector<Cam>& cams, double tol) const;

    // init_from's last point; init_candidates' last frame (t, points with their blobs, (L, R), cameras)
    bool has_cand = false;
    double cand_t = 0;
    V3 cand_pos{};
    struct CandFrame {
        double t;
        std::vector<std::pair<V3, std::vector<Blob>>> pts;
        std::vector<std::vector<Blob>> lr;
        std::vector<Cam> cams;
    };
    std::optional<CandFrame> cand_pts;
};

// a snapshot of a track, on either side of a process boundary (tracker.TrackView)
struct TrackCrossing {
    V3 pos, vel;
    double t_go;
    double cov[4];                   // 2 x 2, xy
};

class TrackView {
public:
    TrackView() = default;
    TrackView(const BallFlight& fl, double accel_sigma, double t, std::vector<double> x, Mat P, int n, bool ready)
        : f(fl), q(accel_sigma * accel_sigma), t(t), x(std::move(x)), P(std::move(P)), n(n), ready(ready) {}
    // (position, velocity, covariance) at t
    void estimate(double t_at, V3& p, V3& v, Mat* P_out = nullptr) const;
    std::optional<TrackCrossing> crossing(double t_at, double z_plane) const;

    BallFlight f;
    double q = 0.25, t = 0;
    std::vector<double> x;
    Mat P;
    int n = 0;
    bool ready = false;
};

}  // namespace omni
