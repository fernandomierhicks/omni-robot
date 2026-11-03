// catch_behavior in C++ (design.md 4.9; the core is game.cpp): omni_catch/behavior_node.py's topics, parameters,
// field setup, sleep outside CATCH and status, so mode_manager, catch_perception and the console see the same node.
//
// Subscribes /catch/track, /mcu/state, /catch/setup (latched: the house's thrower, relative to the robot), /robot_mode
// (latched). Publishes /catch/cmd (100 Hz while awake), /catch/status (on every state change and at 2 Hz),
// /catch/target (10 Hz), /catch/field (latched: the field in the odom frame, for catch_perception). Parameters
// thrower_x, thrower_y, arena_cx, arena_cy, arena_hx, arena_hy, return_throws, kid, return_to (the last three change
// at runtime: the console sets them), sleep, robot_config (default: omni_catch's config/robot_v11.json).
#include <cmath>
#include <memory>
#include <optional>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "omni_cpp/game.hpp"
#include "omni_cpp/mcu_msgs.hpp"
#include "omni_interfaces/msg/ball_track.hpp"
#include "omni_interfaces/msg/catch_status.hpp"
#include "omni_interfaces/msg/mcu_command.hpp"
#include "omni_interfaces/msg/mcu_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace omni {

namespace {

double t_of(const builtin_interfaces::msg::Time& s) { return s.sec + s.nanosec * 1e-9; }

}  // namespace

class CatchBehaviorNode : public rclcpp::Node {
public:
    explicit CatchBehaviorNode(const rclcpp::NodeOptions& opt) : Node("catch_behavior", opt) {
        GameParams gp;
        gp.thrower[0] = declare_parameter<double>("thrower_x", 6.8);
        gp.thrower[1] = declare_parameter<double>("thrower_y", 0.0);
        gp.arena_center[0] = declare_parameter<double>("arena_cx", 3.5);
        gp.arena_center[1] = declare_parameter<double>("arena_cy", 0.0);
        gp.arena_half[0] = declare_parameter<double>("arena_hx", 6.0);
        gp.arena_half[1] = declare_parameter<double>("arena_hy", 3.0);
        gp.return_throws = declare_parameter<bool>("return_throws", true);
        gp.kid = declare_parameter<bool>("kid", false);
        gp.return_to = declare_parameter<std::string>("return_to", "thrower");
        sleep_outside_ = declare_parameter<bool>("sleep", true);
        std::string path = declare_parameter<std::string>("robot_config", "");
        if (path.empty()) path = ament_index_cpp::get_package_share_directory("omni_catch") + "/config/robot_v11.json";
        rc_ = std::make_unique<RobotConfig>(path);
        game_ = make_game(gp);
        mux_ = ModeMux("CATCH", gp.kid);
        p_cmd_ = create_publisher<omni_interfaces::msg::McuCommand>("/catch/cmd", 10);
        p_status_ = create_publisher<omni_interfaces::msg::CatchStatus>("/catch/status", 10);
        p_target_ = create_publisher<visualization_msgs::msg::Marker>("/catch/target", 10);
        auto latched = rclcpp::QoS(1).transient_local();
        p_field_ = create_publisher<std_msgs::msg::String>("/catch/field", latched);
        s_setup_ = create_subscription<std_msgs::msg::String>(
            "/catch/setup", latched, [this](std_msgs::msg::String::ConstSharedPtr m) { on_setup(m->data); });
        wake();
        s_mode_ = create_subscription<std_msgs::msg::String>(
            "/robot_mode", latched, [this](std_msgs::msg::String::ConstSharedPtr m) {
                std::string mode = m->data.substr(0, m->data.find(' '));
                for (auto& c : mode) c = (char)std::toupper((unsigned char)c);
                if (mode == "CATCH" || !sleep_outside_) wake();
                else sleep();
            });
        status_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.5), [this] { status(); });
        params_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;
            for (auto& p : ps) {
                if (p.get_name() == "return_throws") {
                    game_->p.return_throws = p.as_bool();
                } else if (p.get_name() == "kid") {
                    game_->p.kid = mux_.kid = p.as_bool();
                } else if (p.get_name() == "return_to") {
                    std::string v = p.as_string();
                    if (v != "thrower" && v != "other") {
                        res.successful = false;
                        res.reason = "return_to: thrower or other";
                        return res;
                    }
                    game_->p.return_to = v;
                }
            }
            return res;
        });
    }

private:
    std::unique_ptr<CatchBehavior> make_game(const GameParams& gp) {
        return std::make_unique<CatchBehavior>(*rc_, gp, [this](const std::string& s) {
            RCLCPP_INFO(get_logger(), "%s", s.c_str());
        });
    }

    void wake() {
        if (awake_) return;
        awake_ = true;
        rv_.reset(), track_.reset();
        s_track_ = create_subscription<omni_interfaces::msg::BallTrack>(
            "/catch/track", 20, [this](omni_interfaces::msg::BallTrack::ConstSharedPtr m) {
                if (!m->tracking) {
                    track_.reset();
                    return;
                }
                Mat P(6, 6);
                for (int k = 0; k < 36; ++k) P.a[k] = m->covariance[k];
                track_ = TrackView(game_->flight, 0.5, t_of(m->header.stamp), std::vector<double>(m->x.begin(), m->x.end()),
                                   P, (int)m->frames, m->ready);
            });
        s_state_ = create_subscription<omni_interfaces::msg::McuState>(
            "/mcu/state", 50, [this](omni_interfaces::msg::McuState::ConstSharedPtr m) {
                RobotView v;
                v.t = t_of(m->header.stamp);
                v.pose[0] = m->x, v.pose[1] = m->y, v.pose[2] = m->yaw;
                v.vel_body[0] = m->vx, v.vel_body[1] = m->vy, v.vel_body[2] = m->wz;
                v.v_cmd[0] = m->v_cmd_x, v.v_cmd[1] = m->v_cmd_y;
                v.tilt = m->tilt, v.tilt_cmd = m->tilt_cmd;
                v.fly_rpm[0] = m->fly_rpm[0], v.fly_rpm[1] = m->fly_rpm[1];
                v.fly_target_rpm = m->fly_target_rpm, v.fly_ready = m->fly_ready, v.ball_held = m->ball_held;
                v.fire_seq_done = m->fire_seq_done, v.mode = m->mode;
                rv_ = v;
            });
        tick_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), [this] { tick(); });
    }

    void sleep() {
        if (!awake_) return;
        awake_ = false;
        s_track_.reset(), s_state_.reset();
        tick_->cancel();
        tick_.reset();
        rv_.reset(), track_.reset();
    }

    void on_setup(const std::string& text) {
        Json s;
        try {
            s = Json::parse(text);
        } catch (const std::exception&) {
            return;
        }
        if (!s.get("ok").truthy() || s.get("seq").dump() == setup_seq_) return;
        if (!rv_) {
            pending_setup_ = s;                    // applied with the first MCU state
            return;
        }
        apply_setup(s);
    }

    // the field relative to the robot -> this game's odom frame: home where the robot is, the thrower dist m ahead,
    // the play area the room around home; the console's settings (kid, returns, return_to) carry over
    void apply_setup(const Json& s) {
        setup_seq_ = s.get("seq").dump();
        pending_setup_.reset();
        double x = rv_->pose[0], y = rv_->pose[1], yaw = rv_->pose[2];
        double d = s["dist"].num(), half = s.has("half") ? s["half"].num() : 1.5;
        double a = yaw + (s.has("bearing") ? s["bearing"].num() : 0.0);
        double tx = x + d * std::cos(a), ty = y + d * std::sin(a);
        GameParams gp = game_->p;
        gp.home[0] = x, gp.home[1] = y, gp.thrower[0] = tx, gp.thrower[1] = ty;
        gp.arena_center[0] = x, gp.arena_center[1] = y, gp.arena_half[0] = gp.arena_half[1] = half;
        gp.return_throws = s.has("returns") ? s["returns"].truthy() : gp.return_throws;
        game_ = make_game(gp);
        track_.reset();
        double mx = (x + tx) / 2, my = (y + ty) / 2;
        Json f = Json::object();
        f.set("seq", s.get("seq"));
        f.set("home", Json::floats({x, y}));
        f.set("thrower", Json::floats({tx, ty}));
        f.set("arena_center", Json::floats({x, y}));
        f.set("arena_half", Json::floats({half, half}));
        f.set("play_center", Json::floats({mx, my}));
        f.set("play_half", Json::floats({d / 2 + 1.5, d / 2 + 1.5}));
        std_msgs::msg::String m;
        m.data = f.dump();
        p_field_->publish(m);
        RCLCPP_INFO(get_logger(), "catch field: home (%.2f, %.2f), thrower %.1f m away at %.0f deg, room %.1f m", x, y,
                    d, (a - yaw) * (180.0 / kPiC), half);
    }

    void tick() {
        if (!rv_) return;
        if (pending_setup_) apply_setup(*pending_setup_);
        double t = now().seconds();
        if (track_ && t - track_->t > 0.6) track_.reset();          // perception went quiet
        CatchCommand cc = game_->step(t, *rv_, track_ ? &*track_ : nullptr);
        auto m = to_msg(mux_.request(cc));
        m.header.stamp = now();
        p_cmd_->publish(m);
        if (game_->state != last_state_) {
            last_state_ = game_->state;
            status();
        }
        if (++n_tick_ % 10) return;          // the target marker at 10 Hz (at 100 Hz it cost the console 7% of a core)
        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "odom";
        mk.header.stamp = m.header.stamp;
        mk.ns = "catch target", mk.id = 0, mk.type = visualization_msgs::msg::Marker::ARROW;
        mk.action = visualization_msgs::msg::Marker::ADD;
        const double* tg = game_->target;
        mk.pose.position.x = tg[0], mk.pose.position.y = tg[1], mk.pose.position.z = 0.02;
        mk.pose.orientation.z = std::sin(tg[2] / 2), mk.pose.orientation.w = std::cos(tg[2] / 2);
        mk.scale.x = 0.3, mk.scale.y = 0.05, mk.scale.z = 0.05;
        mk.color.r = 0.2f, mk.color.g = 0.6f, mk.color.b = 1.0f, mk.color.a = 0.9f;
        p_target_->publish(mk);
    }

    void status() {
        omni_interfaces::msg::CatchStatus s;
        s.header.stamp = now();
        s.state = game_->state, s.tosses = game_->tosses, s.catches = game_->catches;
        s.last = game_->last.dump();
        p_status_->publish(s);
    }

    std::unique_ptr<RobotConfig> rc_;
    std::unique_ptr<CatchBehavior> game_;
    ModeMux mux_;
    std::optional<RobotView> rv_;
    std::optional<TrackView> track_;
    std::optional<Json> pending_setup_;
    std::string setup_seq_ = "null", last_state_;
    bool sleep_outside_ = true, awake_ = false;
    long n_tick_ = 0;
    rclcpp::Publisher<omni_interfaces::msg::McuCommand>::SharedPtr p_cmd_;
    rclcpp::Publisher<omni_interfaces::msg::CatchStatus>::SharedPtr p_status_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr p_target_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_field_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr s_setup_, s_mode_;
    rclcpp::Subscription<omni_interfaces::msg::BallTrack>::SharedPtr s_track_;
    rclcpp::Subscription<omni_interfaces::msg::McuState>::SharedPtr s_state_;
    rclcpp::TimerBase::SharedPtr tick_, status_;
    OnSetParametersCallbackHandle::SharedPtr params_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::CatchBehaviorNode)
