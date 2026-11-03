// mode_manager: who drives (design.md 14, 15, 30), in C++: omni_catch/mode_node.py's topics, service and behaviour.
// One writer to the MCU link: /mcu/cmd at 100 Hz.
//
// Subscribes /cmd_vel (TELEOP, fresher than 0.5 s), /catch/cmd (CATCH), /thrower/cmd (THROWER), /pick/cmd_vel (PICK),
// /arm/out (every request carries the arm-out caps; the last value holds). Service /set_mode. Publishes /mcu/cmd and
// /robot_mode (latched, e.g. "CATCH adult"). Parameters: mode (initial), kid.
#include <cctype>

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
        s_twist_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
                twist_ = {m->linear.x, m->linear.y, m->angular.z}, t_twist_ = now_s();
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
            if (m->data != mux_.arm_out)
                RCLCPP_INFO(get_logger(), "%s", m->data ? "arm out: base capped" : "arm in: base limits back");
            mux_.arm_out = m->data;
        });
        srv_ = create_service<omni_interfaces::srv::SetMode>(
            "/set_mode", [this](const omni_interfaces::srv::SetMode::Request::SharedPtr q,
                                omni_interfaces::srv::SetMode::Response::SharedPtr a) {
                a->success = mux_.set_mode(q->mode, q->kid, a->message);
                if (a->success) announce();
            });
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), [this] { tick(); });
        announce();
    }

private:
    double now_s() { return now().seconds(); }

    void announce() {
        std_msgs::msg::String m;
        m.data = mux_.mode + (mux_.kid ? " kid" : " adult");
        p_mode_->publish(m);
        RCLCPP_INFO(get_logger(), "mode %s (%s profile)", mux_.mode.c_str(), mux_.kid ? "kid" : "adult");
    }

    void tick() {
        double t = now_s();
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
            r = mux_.request(t - t_twist_ < 0.5 ? &twist_ : nullptr);
        }
        McuCommand m = to_msg(r);
        m.header.stamp = now();
        p_cmd_->publish(m);
    }

    ModeMux mux_;
    Twist2 twist_{}, pick_{};
    Request catch_, thrower_;
    bool has_catch_ = false, has_thrower_ = false;
    double t_twist_ = -1e9, t_catch_ = -1e9, t_thrower_ = -1e9, t_pick_ = -1e9;
    rclcpp::Publisher<McuCommand>::SharedPtr p_cmd_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_mode_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr s_twist_, s_pick_;
    rclcpp::Subscription<McuCommand>::SharedPtr s_catch_, s_thrower_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr s_arm_;
    rclcpp::Service<omni_interfaces::srv::SetMode>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::ModeManager)
