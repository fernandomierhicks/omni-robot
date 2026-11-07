// mode_manager: who drives (design.md 14, 15, 30). The one writer to the MCU link: /mcu/cmd at 100 Hz.
//
// TELEOP takes its twist from several sources, each on its own topic with a priority: the freshest-enough source with
// the highest priority drives, the others are ignored (no two writers ever mix on one topic). Defaults:
//   /cmd_vel/joy     100  the joystick (teleop_twist_joy): a person at the stick always wins
//   /cmd_vel/manual   90  a person through software: Foxglove's Teleop panel, the console's stick when it bypasses Nav2
//   /cmd_vel          50  Nav2 (its collision monitor's output; the console's guarded driving goes through it)
// each fresh for 0.5 s (parameters twist_topics, twist_priorities, twist_timeouts). /cmd_vel/lock (Bool) true stops
// TELEOP whatever the sources say, while it stays fresh (lock_timeout, 1 s). /cmd_vel/active (latched String) names
// the source in charge ("none" when nobody is).
// /catch/cmd (CATCH), /thrower/cmd (THROWER), /pick/cmd_vel (PICK) as before.
// /arm/out: every request carries the arm-out caps while the arm is out. With arm:=true (the arm robot, the default)
// a missing or stale /arm/out (older than arm_out_timeout, 0.5 s; the arm driver sends it at 10 Hz) counts as out: a
// dead arm node caps the base instead of uncapping it. arm:=false (the catch robot) ignores it.
// Service /set_mode. Publishes /mcu/cmd and /robot_mode (latched, e.g. "CATCH adult"). Parameters: mode (initial), kid,
// arm, arm_out_timeout, twist_topics, twist_priorities, twist_timeouts, lock_timeout.
#include <cctype>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "omni_cpp/mcu_msgs.hpp"
#include "omni_cpp/mode_mux.hpp"
#include "omni_interfaces/msg/mcu_command.hpp"
#include "omni_interfaces/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace omni {

using omni_interfaces::msg::McuCommand;

namespace {

std::string upper(std::string s) {
    for (auto& c : s) c = (char)std::toupper((unsigned char)c);
    return s;
}

}  // namespace

class ModeManager : public rclcpp::Node {
public:
    explicit ModeManager(const rclcpp::NodeOptions& opt)
        : Node("mode_manager", opt),
          mux_(upper(declare_parameter<std::string>("mode", "IDLE")), declare_parameter<bool>("kid", false)) {
        p_cmd_ = create_publisher<McuCommand>("/mcu/cmd", 10);
        p_mode_ = create_publisher<std_msgs::msg::String>("/robot_mode", rclcpp::QoS(1).transient_local());
        p_active_ = create_publisher<std_msgs::msg::String>("/cmd_vel/active", rclcpp::QoS(1).transient_local());
        has_arm_ = declare_parameter<bool>("arm", true);
        arm_out_timeout_ = declare_parameter<double>("arm_out_timeout", 0.5);
        lock_timeout_ = declare_parameter<double>("lock_timeout", 1.0);
        auto topics = declare_parameter<std::vector<std::string>>(
            "twist_topics", std::vector<std::string>{"/cmd_vel/joy", "/cmd_vel/manual", "/cmd_vel"});
        auto prios = declare_parameter<std::vector<int64_t>>("twist_priorities", std::vector<int64_t>{100, 90, 50});
        auto touts = declare_parameter<std::vector<double>>("twist_timeouts", std::vector<double>{0.5, 0.5, 0.5});
        if (prios.size() != topics.size() || touts.size() != topics.size())
            throw std::invalid_argument("twist_topics, twist_priorities and twist_timeouts must have the same length");
        sources_.resize(topics.size());
        for (size_t i = 0; i < topics.size(); ++i) {
            Source& src = sources_[i];
            src.topic = topics[i], src.priority = prios[i], src.timeout = touts[i];
            src.sub = create_subscription<geometry_msgs::msg::Twist>(
                src.topic, 10, [this, i](geometry_msgs::msg::Twist::ConstSharedPtr m) {
                    sources_[i].twist = {m->linear.x, m->linear.y, m->angular.z}, sources_[i].t = now_s();
                });
        }
        s_lock_ = create_subscription<std_msgs::msg::Bool>("/cmd_vel/lock", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            lock_ = m->data, t_lock_ = now_s();
        });
        s_catch_ = create_subscription<McuCommand>("/catch/cmd", 10, [this](McuCommand::ConstSharedPtr m) {
            catch_ = from_msg(*m), has_catch_ = true, t_catch_ = now_s();
        });
        s_thrower_ = create_subscription<McuCommand>("/thrower/cmd", 10, [this](McuCommand::ConstSharedPtr m) {
            thrower_ = from_msg(*m), has_thrower_ = true, t_thrower_ = now_s();
        });
        s_pick_ = create_subscription<geometry_msgs::msg::Twist>(
            "/pick/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
                pick_ = {m->linear.x, m->linear.y, m->angular.z}, t_pick_ = now_s();
            });
        s_arm_ = create_subscription<std_msgs::msg::Bool>("/arm/out", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            arm_out_msg_ = m->data, t_arm_ = now_s();
        });
        srv_ = create_service<omni_interfaces::srv::SetMode>(
            "/set_mode", [this](const omni_interfaces::srv::SetMode::Request::SharedPtr q,
                                omni_interfaces::srv::SetMode::Response::SharedPtr a) {
                a->success = mux_.set_mode(q->mode, q->kid, a->message);
                if (a->success) announce();
            });
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), [this] { tick(); });
        announce();
        announce_source("none");
    }

private:
    struct Source {
        std::string topic;
        int64_t priority = 0;
        double timeout = 0.5;
        Twist2 twist{};
        double t = -1e9;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
    };

    void announce_source(const std::string& name) {
        if (name == active_) return;
        active_ = name;
        std_msgs::msg::String m;
        m.data = name;
        p_active_->publish(m);
        RCLCPP_INFO(get_logger(), "drive source: %s", name.c_str());
    }

    // TELEOP's twist: the highest-priority fresh source, none while locked
    const Twist2* teleop_twist(double t) {
        if (lock_ && t - t_lock_ < lock_timeout_) {
            announce_source("locked");
            return nullptr;
        }
        const Source* best = nullptr;
        for (const Source& src : sources_)
            if (t - src.t < src.timeout && (!best || src.priority > best->priority)) best = &src;
        announce_source(best ? best->topic : "none");
        return best ? &best->twist : nullptr;
    }

    void update_arm_out(double t) {
        bool out = has_arm_ && (t - t_arm_ >= arm_out_timeout_ || arm_out_msg_);
        if (out != mux_.arm_out) {
            const char* why = !out ? "arm in: base limits back"
                              : (t - t_arm_ >= arm_out_timeout_ ? "no /arm/out from the arm: base capped as if out"
                                                                : "arm out: base capped");
            RCLCPP_INFO(get_logger(), "%s", why);
        }
        mux_.arm_out = out;
    }

    double now_s() { return now().seconds(); }

    void announce() {
        std_msgs::msg::String m;
        m.data = mux_.mode + (mux_.kid ? " kid" : " adult");
        p_mode_->publish(m);
        RCLCPP_INFO(get_logger(), "mode %s (%s profile)", mux_.mode.c_str(), mux_.kid ? "kid" : "adult");
    }

    void tick() {
        double t = now_s();
        update_arm_out(t);
        Request r;
        if (mux_.mode == "CATCH") {
            if (has_catch_ && t - t_catch_ < 0.3) {
                r = catch_;                        // a copy: the stored request keeps the behavior's own fire_seq
                r.kid = mux_.kid;
                r.fire_seq = mux_.fire(catch_.fire_seq);
            } else {
                r = mux_.request();                // the behavior is silent: a ramped stop
            }
        } else if (mux_.mode == "THROWER") {
            bool fresh = has_thrower_ && t - t_thrower_ < 0.3;
            r = mux_.request(nullptr, fresh ? &thrower_ : nullptr);
        } else if (mux_.mode == "PICK") {
            r = mux_.request(nullptr, nullptr, t - t_pick_ < 0.3 ? &pick_ : nullptr);
        } else {
            r = mux_.request(teleop_twist(t));
        }
        McuCommand m = to_msg(r);
        m.header.stamp = now();
        p_cmd_->publish(m);
    }

    ModeMux mux_;
    std::vector<Source> sources_;
    std::string active_;
    Twist2 pick_{};
    Request catch_, thrower_;
    bool has_catch_ = false, has_thrower_ = false;
    double t_catch_ = -1e9, t_thrower_ = -1e9, t_pick_ = -1e9;
    bool has_arm_ = true, arm_out_msg_ = false, lock_ = false;
    double arm_out_timeout_ = 0.5, lock_timeout_ = 1.0, t_arm_ = -1e9, t_lock_ = -1e9;
    rclcpp::Publisher<McuCommand>::SharedPtr p_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_mode_, p_active_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr s_pick_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_lock_;
    rclcpp::Subscription<McuCommand>::SharedPtr s_catch_, s_thrower_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_arm_;
    rclcpp::Service<omni_interfaces::srv::SetMode>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::ModeManager)
