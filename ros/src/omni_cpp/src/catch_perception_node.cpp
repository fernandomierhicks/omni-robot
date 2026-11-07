// catch_perception in C++ (design.md 4.9; the core is perception.cpp): omni_catch/perception_node.py's topics,
// parameters, frame pairing, sleep outside CATCH and statistics, so the behaviour and the console see the same node.
//
// Subscribes /mcu/state (where the robot was), /cam/{left,right}/sparse (source "sparse": the sim) or
// /cam/{left,right}/image_raw (rgb8 / bgr8), /catch/field (latched: the play area), /robot_mode (latched). A stereo
// pair is the two frames with the same exposure stamp (within 1 ms); a frame whose partner hasn't come 10 ms later
// is processed alone (at once with a one-camera robot_config: the bench camera head, characterization/pi_camera). Publishes /catch/track (after every pair while tracking), /catch/ball (markers), /catch/perception
// (JSON, 1 Hz). Parameters source, detector (accepted; the C++ has one detector), calib_rot_deg, ball_size_rel,
// arena_cx, arena_cy, arena_hx, arena_hy, sleep, robot_config (default: omni_catch's config/robot_v11.json).
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "omni_cpp/json_value.hpp"
#include "omni_cpp/perception.hpp"
#include "omni_interfaces/msg/ball_track.hpp"
#include "omni_interfaces/msg/mcu_state.hpp"
#include "omni_interfaces/msg/sparse_image.hpp"
#include "omni_cpp/params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace omni {

using omni_interfaces::msg::BallTrack;
using visualization_msgs::msg::Marker;

namespace {

std::string robot_config_path(rclcpp::Node& n) {
    std::string p = params::text(n, "robot_config", "", "the robot's generated json (\"\" = omni_catch's robot_v11.json)");
    return p.empty() ? ament_index_cpp::get_package_share_directory("omni_catch") + "/config/robot_v11.json" : p;
}

builtin_interfaces::msg::Time stamp_of(double t) {
    int64_t ns = std::llround(t * 1e9);
    builtin_interfaces::msg::Time s;
    s.sec = (int32_t)(ns / 1000000000), s.nanosec = (uint32_t)(ns % 1000000000);
    return s;
}

double t_of(const builtin_interfaces::msg::Time& s) { return s.sec + s.nanosec * 1e-9; }

}  // namespace

class CatchPerception : public rclcpp::Node {
public:
    explicit CatchPerception(const rclcpp::NodeOptions& opt) : Node("catch_perception", opt) {
        source_ = params::text(*this, "source", "sparse", "camera input: sparse (SparseImage) | image (sensor_msgs/Image)");
        params::text(*this, "detector", "native", "the ball detector (native: the C++ blob detector, the only one)");
        TrackerConfig tc;
        tc.calib_rot_deg = params::number(*this, "calib_rot_deg", 0.10, 0.0, 5.0, "deg: the stereo calibration's rotation uncertainty (the filter's)", true);
        tc.ball_size_rel = params::number(*this, "ball_size_rel", 0.015, 0.0, 0.5, "the ball's apparent-size measurement noise, relative", true);
        RobotConfig rc(robot_config_path(*this));
        perc_ = std::make_unique<Perception>(rc, tc, DetectorConfig());
        perc_->set_play_area(params::number(*this, "arena_cx", 3.5, -50, 50, "m: the play area's centre x (odom)", true), params::number(*this, "arena_cy", 0.0, -50, 50, "m: the play area's centre y", true),
                             params::number(*this, "arena_hx", 6.0, 0.5, 50, "m: the play area's half length", true), params::number(*this, "arena_hy", 3.0, 0.5, 50, "m: the play area's half width", true));
        z_rim_ = perc_->robot.rim_center(0.0)[1];
        sleep_outside_ = params::flag(*this, "sleep", true, "asleep outside CATCH (no /mcu/state, cameras or fast timers)", true);
        p_track_ = create_publisher<BallTrack>("/catch/track", 20);
        p_ball_ = create_publisher<Marker>("/catch/ball", 10);
        p_stats_ = create_publisher<std_msgs::msg::String>("/catch/perception", 5);
        auto latched = rclcpp::QoS(1).transient_local();
        s_field_ = create_subscription<std_msgs::msg::String>(
            "/catch/field", latched, [this](std_msgs::msg::String::ConstSharedPtr m) { on_field(m->data); });
        wake();
        s_mode_ = create_subscription<std_msgs::msg::String>(
            "/robot_mode", latched, [this](std_msgs::msg::String::ConstSharedPtr m) {
                std::string mode = m->data.substr(0, m->data.find(' '));
                for (auto& c : mode) c = (char)std::toupper((unsigned char)c);
                if (mode == "CATCH" || !sleep_outside_) wake();
                else sleep();
            });
        report_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0), [this] { report(); });
        RCLCPP_INFO(get_logger(), "perception on %s frames, %zu cameras (C++)", source_.c_str(), perc_->cams.size());
    }

private:
    struct Slot {
        int64_t key;
        std::shared_ptr<Frame> fr[2];
        std::shared_ptr<const void> keep[2];      // the message the frame's pixels live in
        double t_seen;
    };

    static double wall() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void on_field(const std::string& text) {
        try {
            Json f = Json::parse(text);
            auto c = f["play_center"].nums(), h = f["play_half"].nums();
            perc_->set_play_area(c.at(0), c.at(1), h.at(0), h.at(1));
        } catch (const std::exception&) {
        }
    }

    void wake() {
        if (awake_) return;
        awake_ = true;
        perc_->poses.clear();                    // a history from before the sleep would bridge the gap
        perc_->reset();
        pending_.clear();
        s_state_ = create_subscription<omni_interfaces::msg::McuState>(
            "/mcu/state", 50, [this](omni_interfaces::msg::McuState::ConstSharedPtr m) {
                perc_->add_pose(t_of(m->header.stamp), m->x, m->y, m->yaw, m->roll, m->pitch);
            });
        const char* sides[2] = {"left", "right"};
        for (int k = 0; k < 2; ++k) {
            if (source_ == "sparse") {
                s_sparse_[k] = create_subscription<omni_interfaces::msg::SparseImage>(
                    std::string("/cam/") + sides[k] + "/sparse", 20,
                    [this, k](omni_interfaces::msg::SparseImage::ConstSharedPtr m) {
                        std::vector<int64_t> b(m->ball.begin(), m->ball.end()), c(m->clutter.begin(), m->clutter.end());
                        add(k, m->header.stamp, std::make_shared<SparseFrame>(m->width, m->height, b, c), nullptr);
                    });
            } else {
                s_image_[k] = create_subscription<sensor_msgs::msg::Image>(
                    std::string("/cam/") + sides[k] + "/image_raw", 5,
                    [this, k](sensor_msgs::msg::Image::ConstSharedPtr m) {
                        if (m->width == 0 || m->height == 0) return;
                        long px = (long)(m->data.size() / ((size_t)m->width * m->height));
                        std::string enc = m->encoding;
                        for (auto& c : enc) c = (char)std::tolower((unsigned char)c);
                        auto fr = std::make_shared<RgbFrame>(m->data.data(), m->width, m->height, (long)m->width * px,
                                                             px, enc.rfind("bgr", 0) == 0);
                        add(k, m->header.stamp, fr, m);
                    });
            }
        }
        flush_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.005), [this] { flush(); });
    }

    void sleep() {
        if (!awake_) return;
        awake_ = false;
        s_state_.reset();
        for (auto& s : s_sparse_) s.reset();
        for (auto& s : s_image_) s.reset();
        flush_->cancel();
        flush_.reset();
        pending_.clear();
        perc_->reset();
        if (was_tracking_) {                      // tell catch_behavior the track ended
            was_tracking_ = false;
            BallTrack m;
            m.header.frame_id = "odom";
            m.header.stamp = now();
            p_track_->publish(m);
        }
    }

    void add(int k, const builtin_interfaces::msg::Time& st, std::shared_ptr<Frame> fr, std::shared_ptr<const void> keep) {
        int64_t ns = (int64_t)st.sec * 1000000000 + st.nanosec;
        auto it = std::find_if(pending_.begin(), pending_.end(), [&](const Slot& s) { return std::llabs(s.key - ns) < 1000000; });
        if (it == pending_.end()) {
            pending_.push_back({ns, {nullptr, nullptr}, {nullptr, nullptr}, wall()});
            it = pending_.end() - 1;
        }
        it->fr[k] = std::move(fr);
        it->keep[k] = std::move(keep);
        if (it->fr[0] && (it->fr[1] || perc_->cams.size() < 2)) process(it - pending_.begin());   // a mono config: at once
    }

    void flush() {
        double t = wall();
        std::vector<int64_t> keys;
        for (auto& s : pending_)
            if (t - s.t_seen > 0.010) keys.push_back(s.key);
        std::sort(keys.begin(), keys.end());
        for (int64_t key : keys) {
            auto it = std::find_if(pending_.begin(), pending_.end(), [&](const Slot& s) { return s.key == key; });
            if (it != pending_.end()) process(it - pending_.begin());
        }
    }

    void process(long i) {
        Slot s = pending_[i];
        pending_.erase(pending_.begin() + i);
        double w0 = wall();
        perc_->process(s.key * 1e-9, {s.fr[0].get(), s.fr[1].get()});
        proc_ms_.push_back(1000 * (wall() - w0));
        if (proc_ms_.size() > 500) proc_ms_.pop_front();
        ++n_pairs_;
        auto v = perc_->view();
        if (!v && !was_tracking_) return;
        was_tracking_ = (bool)v;
        BallTrack m;
        m.header.frame_id = "odom";
        m.header.stamp = stamp_of(s.key * 1e-9);
        if (v) {
            m.header.stamp = stamp_of(v->t);
            m.tracking = true, m.ready = v->ready, m.frames = (uint32_t)v->n;
            for (int k = 0; k < 6; ++k) m.x[k] = v->x[k];
            for (int k = 0; k < 36; ++k) m.covariance[k] = v->P.a[k];
            auto cr = v->crossing(v->t, z_rim_);
            if (cr) {
                for (int k = 0; k < 3; ++k) m.crossing[k] = cr->pos[k];
                m.t_go = cr->t_go;
                m.crossing_sigma = std::sqrt(eig2_max(cr->cov[0], cr->cov[2], cr->cov[3]));
            }
            marker(*v, cr ? &*cr : nullptr);
        }
        p_track_->publish(m);
    }

    void marker(const TrackView& v, const TrackCrossing* cr) {
        Marker mk;
        mk.header.frame_id = "odom";
        mk.header.stamp = now();
        mk.ns = "ball track", mk.id = 0, mk.type = Marker::SPHERE, mk.action = Marker::ADD;
        mk.pose.position.x = v.x[0], mk.pose.position.y = v.x[1], mk.pose.position.z = v.x[2];
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = mk.scale.z = 0.074;
        mk.color.r = 0.85f, mk.color.g = 0.95f, mk.color.b = 0.1f, mk.color.a = 0.9f;
        p_ball_->publish(mk);
        if (cr) {
            Marker mk2;
            mk2.header = mk.header;
            mk2.ns = "rim crossing", mk2.id = 1, mk2.type = Marker::CYLINDER, mk2.action = Marker::ADD;
            mk2.pose.position.x = cr->pos[0], mk2.pose.position.y = cr->pos[1], mk2.pose.position.z = 0.01;
            mk2.pose.orientation.w = 1.0;
            mk2.scale.x = mk2.scale.y = 0.33;
            mk2.scale.z = 0.01;
            mk2.color.r = 1.0f, mk2.color.g = 0.5f, mk2.color.b = 0.0f, mk2.color.a = 0.6f;
            p_ball_->publish(mk2);
        }
    }

    void report() {
        std::vector<double> ms(proc_ms_.begin(), proc_ms_.end());
        if (ms.empty()) ms.push_back(0.0);
        std::sort(ms.begin(), ms.end());
        size_t n = ms.size();
        double med = n % 2 ? ms[n / 2] : 0.5 * (ms[n / 2 - 1] + ms[n / 2]);
        double pos = 0.95 * (n - 1), fl = std::floor(pos);                 // np.percentile's linear interpolation
        double p95 = ms[(size_t)fl] + (pos - fl) * (ms[std::min(n - 1, (size_t)fl + 1)] - ms[(size_t)fl]);
        Json d = Json::object();
        d.set("pairs", Json(n_pairs_));
        d.set("detections", Json((int64_t)perc_->detections));
        d.set("stereo", Json((int64_t)perc_->stereo));
        d.set("coarse", Json((int64_t)perc_->det.n_coarse));
        d.set("tracking", Json(perc_->tracker.tracking()));
        d.set("proc_ms_median", Json(py_round(med, 2)));
        d.set("proc_ms_95", Json(py_round(p95, 2)));
        d.set("pending", Json((int64_t)pending_.size()));
        std_msgs::msg::String m;
        m.data = d.dump();
        p_stats_->publish(m);
    }

    std::string source_;
    std::unique_ptr<Perception> perc_;
    double z_rim_ = 0;
    bool sleep_outside_ = true, awake_ = false, was_tracking_ = false;
    std::vector<Slot> pending_;
    std::deque<double> proc_ms_;
    int64_t n_pairs_ = 0;
    rclcpp::Publisher<BallTrack>::SharedPtr p_track_;
    rclcpp::Publisher<Marker>::SharedPtr p_ball_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_stats_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr s_field_, s_mode_;
    rclcpp::Subscription<omni_interfaces::msg::McuState>::SharedPtr s_state_;
    rclcpp::Subscription<omni_interfaces::msg::SparseImage>::SharedPtr s_sparse_[2];
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr s_image_[2];
    rclcpp::TimerBase::SharedPtr flush_, report_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::CatchPerception)
