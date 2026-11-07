// mcu_link: the host's end of the MCU link (design.md 5, 30), in C++: omni_mcu/link_node.py's topics, parameters and
// behaviour (its notes are the reference), the frames and bodies compiled from the firmware's own protocol.h. Runs on
// the robot (USB serial to the Teensy) and against the sim (TCP to the bridge).
//
// Publishes /mcu/state (200 Hz), /mcu/state_10hz, /mcu/samples (20 Hz, while someone listens), /odom (50 Hz), /tf
// odom -> base_link (publish_tf), /imu (400 Hz), /mcu/link and /mcu/clock (JSON, 1 Hz), /mcu/info (10 Hz),
// /mcu/time_ref (camera triggers and strobes). Subscribes /mcu/cmd (sent at `rate` Hz as the heartbeat; a request
// older than 0.3 s becomes a ramped stop), /sim/sync_edge (sync_source "topic"). Parameters: port ("tcp:host:port" |
// "serial:/dev/ttyACM0"), publish_tf, rate, sync_source (none | gpio | topic), sync_gpio ("/dev/gpiochip0:<line>"),
// sync_clock (realtime | monotonic | hte), edge_latency_us. Every MCU time is mapped through the clock map, never
// stamped on arrival (design.md 10).
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "omni_cpp/gpio_edges.hpp"
#include "omni_cpp/host_link.hpp"
#include "omni_cpp/params.hpp"
#include "omni_cpp/json.hpp"
#include "omni_interfaces/msg/mcu_command.hpp"
#include "omni_interfaces/msg/mcu_info.hpp"
#include "omni_interfaces/msg/mcu_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace omni {

namespace {

constexpr double DEG = 3.14159265358979323846 / 180.0;
const char* const SAMPLE_COLS = "t,vx,vy,wz,v_cmd_x,v_cmd_y,w_cmd,slip,roll,pitch,yaw";
constexpr size_t N_COLS = 11;

builtin_interfaces::msg::Time stamp_of(double t) {
    int64_t ns = std::llround(t * 1e9);
    builtin_interfaces::msg::Time s;
    s.sec = (int32_t)(ns / 1000000000);
    s.nanosec = (uint32_t)(ns % 1000000000);
    return s;
}

// TCP to the sim's bridge or the Teensy's USB serial port; reads wait at most 50 ms
class Transport {
public:
    explicit Transport(std::string spec) : spec_(std::move(spec)) {}
    ~Transport() { close(); }

    bool open(std::string& err) {
        auto colon = spec_.find(':');
        std::string kind = spec_.substr(0, colon), rest = colon == std::string::npos ? "" : spec_.substr(colon + 1);
        if (kind == "tcp") return open_tcp(rest, err);
        if (kind == "serial") return open_serial(rest, err);
        err = "unknown port " + spec_;
        return false;
    }

    // > 0 bytes, 0 nothing yet, < 0 lost
    ssize_t read(uint8_t* buf, size_t cap) {
        pollfd p{fd_, POLLIN, 0};
        int r = poll(&p, 1, 50);
        if (r == 0) return 0;
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (p.revents & (POLLERR | POLLNVAL)) return -1;
        ssize_t n = tcp_ ? ::recv(fd_, buf, cap, 0) : ::read(fd_, buf, cap);
        if (n == 0) return (p.revents & POLLHUP) || tcp_ ? -1 : 0;     // a closed socket reads 0
        if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
        return n;
    }

    // the whole frame or false (the caller drops the link and reconnects: a fresh stream, so no half frame is
    // continued); a full buffer is waited on, WRITE_DEADLINE_MS for the frame, not spun on
    bool write(const uint8_t* d, size_t n) {
        static constexpr int WRITE_DEADLINE_MS = 50;
        auto t_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(WRITE_DEADLINE_MS);
        while (n) {
            ssize_t w = tcp_ ? ::send(fd_, d, n, MSG_NOSIGNAL) : ::write(fd_, d, n);
            if (w < 0) {
                if (errno == EINTR) continue;
                if (errno != EAGAIN && errno != EWOULDBLOCK) return false;
                auto left = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - std::chrono::steady_clock::now());
                if (left.count() <= 0) {
                    errno = ETIMEDOUT;
                    return false;
                }
                pollfd p{fd_, POLLOUT, 0};
                if (poll(&p, 1, (int)left.count()) < 0 && errno != EINTR) return false;
                continue;
            }
            d += w, n -= (size_t)w;
        }
        return true;
    }

    void close() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

private:
    bool open_tcp(const std::string& hp, std::string& err) {
        auto c = hp.rfind(':');
        std::string host = hp.substr(0, c), port = hp.substr(c + 1);
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        if (int e = getaddrinfo(host.c_str(), port.c_str(), &hints, &res)) {
            err = gai_strerror(e);
            return false;
        }
        int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, res->ai_protocol);
        fcntl(fd, F_SETFL, O_NONBLOCK);
        int r = connect(fd, res->ai_addr, res->ai_addrlen);
        freeaddrinfo(res);
        if (r < 0 && errno == EINPROGRESS) {   // a 1 s connect timeout
            pollfd p{fd, POLLOUT, 0};
            int so = 0;
            socklen_t sl = sizeof so;
            if (poll(&p, 1, 1000) == 1 && getsockopt(fd, SOL_SOCKET, SO_ERROR, &so, &sl) == 0 && so == 0) r = 0;
            else errno = so ? so : ETIMEDOUT;
        }
        if (r < 0) {
            err = std::strerror(errno);
            ::close(fd);
            return false;
        }
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        fd_ = fd, tcp_ = true;
        return true;
    }

    bool open_serial(const std::string& dev, std::string& err) {
        int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            err = dev + ": " + std::strerror(errno);
            return false;
        }
        termios t{};
        if (tcgetattr(fd, &t) == 0) {
            cfmakeraw(&t);
#ifdef B2000000
            cfsetspeed(&t, B2000000);          // the Teensy's native USB ignores it
#endif
            t.c_cc[VMIN] = 0, t.c_cc[VTIME] = 0;
            tcsetattr(fd, TCSANOW, &t);
        }
        fd_ = fd, tcp_ = false;
        return true;
    }

    std::string spec_;
    int fd_ = -1;
    bool tcp_ = false;
};

}  // namespace

class McuLink : public rclcpp::Node {
public:
    explicit McuLink(const rclcpp::NodeOptions& opt) : Node("mcu_link", opt) {
        namespace P = params;
        port_ = P::text(*this, "port", "tcp:127.0.0.1:5761", "the MCU: tcp:host:port (the sim's firmware) or a serial device");
        publish_tf_ = P::flag(*this, "publish_tf", true, "publish odom -> base_link from the MCU (false when the EKF does)", true);
        double rate = P::number(*this, "rate", 100.0, 20.0, 500.0, "Hz: CMDs to the MCU (its heartbeat)", true);
        cmd_timeout_ = P::number(*this, "cmd_timeout", 0.3, 0.05, 0.45,
                                 "s: no /mcu/cmd for this long -> send a ramped stop (below the MCU's own 0.5 s timeout)");
        live_ = std::make_unique<P::Live>(*this);
        live_->bind("cmd_timeout", &cmd_timeout_);
        ClockSyncConfig cc;
        cc.edge_latency_ns = (int64_t)(P::number(*this, "edge_latency_us", 0.0, 0.0, 10000.0,
                                                 "us: the GPIO edge's stamping latency, taken off each SYNC pulse", true) * 1000);
        link_ = std::make_unique<HostLink>(cc);
        sync_source_ = P::text(*this, "sync_source", "none", "the MCU clock map's pulses: none (USB echo only) | gpio | topic");
        auto sync_gpio = P::text(*this, "sync_gpio", "", "the SYNC_OUT pin (gpiochip:line) with sync_source:=gpio");
        auto sync_clock = P::text(*this, "sync_clock", "realtime", "the clock the GPIO edges are stamped on: realtime | monotonic | hte");
        tr_ = std::make_unique<Transport>(port_);

        p_state_ = create_publisher<omni_interfaces::msg::McuState>("/mcu/state", 50);
        p_state10_ = create_publisher<omni_interfaces::msg::McuState>("/mcu/state_10hz", 5);
        p_samples_ = create_publisher<std_msgs::msg::Float32MultiArray>("/mcu/samples", 5);
        p_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 20);
        p_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 50);
        p_link_ = create_publisher<std_msgs::msg::String>("/mcu/link", 5);
        p_info_ = create_publisher<omni_interfaces::msg::McuInfo>("/mcu/info", 5);
        p_clock_ = create_publisher<std_msgs::msg::String>("/mcu/clock", 5);
        p_tref_ = create_publisher<sensor_msgs::msg::TimeReference>("/mcu/time_ref", 50);
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        sub_cmd_ = create_subscription<omni_interfaces::msg::McuCommand>(
            "/mcu/cmd", 10, [this](omni_interfaces::msg::McuCommand::ConstSharedPtr m) { on_cmd(*m); });
        if (sync_source_ == "topic") {
            sub_edge_ = create_subscription<sensor_msgs::msg::TimeReference>(
                "/sim/sync_edge", 10, [this](sensor_msgs::msg::TimeReference::ConstSharedPtr m) {
                    on_sync_edge((int64_t)m->header.stamp.sec * 1000000000 + m->header.stamp.nanosec);
                });
        } else if (sync_source_ == "gpio") {
            auto c = sync_gpio.rfind(':');
            edges_ = std::make_unique<EdgeReader>(
                sync_gpio.substr(0, c), (unsigned)std::stoul(sync_gpio.substr(c + 1)), sync_clock,
                [this](int64_t t) { on_sync_edge(t); }, [this](const std::string& s) { RCLCPP_WARN(get_logger(), "%s", s.c_str()); });
            edges_->start();
        }
        t_send_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] { send(); });
        t_report_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0), [this] { report(); });
        reader_ = std::thread(&McuLink::read_loop, this);
    }

    ~McuLink() override {
        running_ = false;
        if (reader_.joinable()) reader_.join();
        if (edges_) edges_->stop();
        std::lock_guard<std::mutex> g(io_mtx_);
        tr_->close();
    }

private:
    double now_s() { return now().seconds(); }

    // -- commands out ----------------------------------------------------------------------------------------------
    void on_cmd(const omni_interfaces::msg::McuCommand& m) {
        Request r;
        r.req_mode = m.req_mode, r.kid = m.kid, r.drive_mode = m.drive_mode, r.hold_yaw = m.hold_yaw;
        r.a = m.a, r.b = m.b, r.c = m.c, r.v_end = m.v_end, r.a_cap = m.a_cap, r.speed_cap = m.speed_cap;
        r.t_brake = m.t_brake, r.coast_s = m.coast_s, r.a_brake = m.a_brake, r.tilt = m.tilt, r.t_tilt = m.t_tilt;
        r.tilt_after = m.tilt_after, r.fly_rpm = m.fly_rpm, r.fire_seq = m.fire_seq;
        std::lock_guard<std::mutex> g(mtx_);
        req_ = r;
        t_req_ = now_s();
    }

    static bool finite(const Request& r) {
        for (double x : {r.a, r.b, r.c, r.v_end, r.a_cap, r.speed_cap, r.t_brake, r.coast_s, r.a_brake, r.tilt, r.t_tilt,
                         r.tilt_after, r.fly_rpm})
            if (!std::isfinite(x)) return false;
        return true;
    }

    void send() {
        if (!connected_) return;
        double now = now_s();
        std::vector<uint8_t> data;
        {
            std::lock_guard<std::mutex> g(mtx_);
            Request r = req_;
            if (now - t_req_ > cmd_timeout_) {  // nobody is asking: a ramped stop, still armed
                Request s;
                s.req_mode = req_.req_mode == fw::REQ_AUTO ? fw::REQ_AUTO : fw::REQ_DISARM;
                s.kid = req_.kid, s.fire_seq = req_.fire_seq;
                r = s;
            }
            if (!finite(r)) {                   // the MCU would drop it (and time out): send a ramped stop instead
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/mcu/cmd with a non-finite value: stopping");
                Request s;
                s.req_mode = r.req_mode == fw::REQ_AUTO ? fw::REQ_AUTO : fw::REQ_DISARM;
                s.kid = r.kid, s.fire_seq = r.fire_seq;
                r = s;
            }
            if (link_->has_info && link_->info.protocol != fw::PROTOCOL_VERSION) {
                // other firmware: never arm it (it drops our CMDs anyway; this says why in the log)
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                      "MCU protocol %u, this host speaks %u: not arming (flash matching firmware)",
                                      (unsigned)link_->info.protocol, (unsigned)fw::PROTOCOL_VERSION);
                Request s;
                s.kid = r.kid;
                r = s;
            }
            data = link_->encode(to_command(r, link_->sync), this->now().nanoseconds());
        }
        std::lock_guard<std::mutex> g(io_mtx_);
        if (connected_ && !tr_->write(data.data(), data.size())) {
            RCLCPP_WARN(get_logger(), "MCU link write: %s", std::strerror(errno));
            connected_ = false;
        }
    }

    // -- messages in -----------------------------------------------------------------------------------------------
    void read_loop() {
        uint8_t buf[65536];
        while (running_ && rclcpp::ok()) {
            if (!connected_) {
                std::string err;
                {
                    std::lock_guard<std::mutex> g(io_mtx_);
                    tr_->close();
                    if (tr_->open(err)) {
                        connected_ = true;
                        RCLCPP_INFO(get_logger(), "MCU link up on %s", port_.c_str());
                    }
                }
                if (!connected_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MCU link %s: %s", port_.c_str(), err.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
                continue;
            }
            ssize_t n = tr_->read(buf, sizeof buf);
            if (n < 0) {
                RCLCPP_WARN(get_logger(), "MCU link lost: %s", n < 0 && errno ? std::strerror(errno) : "closed");
                connected_ = false;
                continue;
            }
            if (n == 0) continue;
            std::vector<Message> msgs;
            {
                std::lock_guard<std::mutex> g(mtx_);
                msgs = link_->feed(buf, (size_t)n, this->now().nanoseconds());
            }
            for (const Message& m : msgs) {
                if (m.kind == Message::STATE) on_state(m.state, m.t_host);
                else if (m.kind == Message::IMU) on_imu(m.imu, m.t_host);
                else if (m.kind == Message::INFO) on_info(m.info, m.t_host);
                else on_time(m.time, m.t_host);
            }
        }
    }

    // -- time sync (design.md 10) ----------------------------------------------------------------------------------
    void on_sync_edge(int64_t t_ns) {
        std::lock_guard<std::mutex> g(mtx_);
        link_->add_sync_edge(t_ns);
    }

    void on_time(const fw::TimeBody& e, double t_host) {
        if ((e.kind != fw::TE_CAM_TRIGGER && e.kind != fw::TE_CAM_STROBE) || std::isnan(t_host)) return;
        sensor_msgs::msg::TimeReference m;
        m.header.stamp = stamp_of(t_host);
        m.time_ref.sec = (int32_t)(e.t_mcu_us / 1000000);
        m.time_ref.nanosec = (uint32_t)((e.t_mcu_us % 1000000) * 1000);
        m.source = e.kind == fw::TE_CAM_TRIGGER ? "cam_trigger:" + std::to_string(e.seq)
                                                : "cam_strobe:" + std::to_string(e.seq) + ":" + std::to_string(e.value);
        p_tref_->publish(m);
    }

    void on_info(const fw::InfoBody& i, double t_host) {
        omni_interfaces::msg::McuInfo m;
        if (!std::isnan(t_host)) m.header.stamp = stamp_of(t_host);
        m.protocol = i.protocol;
        m.build = std::string(i.build, strnlen(i.build, sizeof i.build));
        m.rc_ok = i.rc_ok, m.rc_manual = i.rc_manual;
        for (int k = 0; k < 8; ++k) m.rc[k] = (float)(i.rc[k] / 1000.0);
        for (int k = 0; k < 4; ++k) m.motor_temp_c[k] = i.dm_t_rotor[k], m.driver_temp_c[k] = i.dm_t_mos[k];
        p_info_->publish(m);
    }

    void on_state(const fw::StateBody& s, double t_host) {
        if (std::isnan(t_host)) return;
        ++n_state_;
        std::string mode = mode_name(s.mode);
        {
            std::lock_guard<std::mutex> g(mtx_);
            last_state_ = s, has_state_ = true;
        }
        if (mode != last_mode_) {
            std::string f;
            for (auto& x : fault_names(s.faults)) f += (f.empty() ? "" : ", ") + x;
            RCLCPP_INFO(get_logger(), "MCU mode %s (faults: %s)", mode.c_str(), f.empty() ? "none" : f.c_str());
            last_mode_ = mode;
        }
        omni_interfaces::msg::McuState m;
        m.header.stamp = stamp_of(t_host);
        m.header.frame_id = "odom";
        m.t_mcu_us = s.t_mcu_us;
        m.mode = mode;
        m.profile = s.profile ? "kid" : "adult";
        m.drive_mode = s.drive_mode;
        m.feed_state = feed_state_name(s.feed_state);
        m.faults = fault_names(s.faults);
        for (int k = 0; k < 4; ++k) m.dm_err[k] = s.dm_err[k];
        m.x = s.x, m.y = s.y, m.yaw = s.yaw, m.vx = s.vx, m.vy = s.vy, m.wz = s.wz, m.slip = s.slip;
        m.roll = s.roll, m.pitch = s.pitch, m.v_cmd_x = s.v_cmd_x, m.v_cmd_y = s.v_cmd_y, m.w_cmd = s.w_cmd;
        m.fly_rpm[0] = s.fly_rpm[0], m.fly_rpm[1] = s.fly_rpm[1];
        m.fly_target_rpm = s.fly_target_rpm;
        m.fly_ready = s.fly_ready;
        m.tilt = s.tilt, m.tilt_cmd = s.tilt_cmd;
        m.ball_held = s.beams & 1;
        m.feed_exit_beam = s.beams & 2;
        m.fire_seq_done = s.fire_seq_done, m.fire_rejected = s.fire_rejected;
        m.v_bus = s.v_bus;
        {
            std::lock_guard<std::mutex> g(mtx_);
            m.sync_rtt_ms = link_->sync.rtt_ns.value_or(0) * 1e-6;
        }
        p_state_->publish(m);
        if (n_state_ % 20 == 0) p_state10_->publish(m);
        if (n_state_ % 40 == 0) {              // 5 Hz: is the Base tab listening (samples start at once)
            want_samples_ = p_samples_->get_subscription_count() > 0;
            if (!want_samples_) samples_.clear();
        }
        if (want_samples_) {
            if (!has_t0_) t0_ = s.t_mcu_us, has_t0_ = true;
            for (double v : {(double)(s.t_mcu_us - t0_) * 1e-6, (double)s.vx, (double)s.vy, (double)s.wz,
                             (double)s.v_cmd_x, (double)s.v_cmd_y, (double)s.w_cmd, (double)s.slip, (double)s.roll,
                             (double)s.pitch, (double)s.yaw})
                samples_.push_back((float)v);
            if (n_state_ % 10 == 0) publish_samples();
        }
        if (n_state_ % 4) return;
        // odometry at 50 Hz, with a covariance that grows with speed and slip (as v10's bridge, estimator.EKF)
        double sp = std::hypot((double)s.vx, (double)s.vy);
        double sv = 0.01 + 0.02 * sp + 3.0 * s.slip;
        double sw = 0.02 + 0.05 * std::fabs((double)s.wz) + 3.0 * s.slip;
        const double T = 0.02, big = 1e3;
        odom_var_[0] += (sv * T) * (sv * T), odom_var_[1] += (sv * T) * (sv * T), odom_var_[2] += (sw * T) * (sw * T);
        nav_msgs::msg::Odometry o;
        o.header = m.header;
        o.child_frame_id = "base_link";
        o.pose.pose.position.x = s.x, o.pose.pose.position.y = s.y;
        double qz = std::sin((double)s.yaw / 2), qw = std::cos((double)s.yaw / 2);
        o.pose.pose.orientation.z = qz, o.pose.pose.orientation.w = qw;
        const double pv[6] = {odom_var_[0], odom_var_[1], big, big, big, odom_var_[2]};
        const double tv[6] = {sv * sv, sv * sv, big, big, big, sw * sw};
        for (int k = 0; k < 6; ++k) o.pose.covariance[k * 7] = pv[k], o.twist.covariance[k * 7] = tv[k];
        o.twist.twist.linear.x = s.vx, o.twist.twist.linear.y = s.vy, o.twist.twist.angular.z = s.wz;
        p_odom_->publish(o);
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tr;
            tr.header = m.header;
            tr.child_frame_id = "base_link";
            tr.transform.translation.x = s.x, tr.transform.translation.y = s.y;
            tr.transform.rotation.z = qz, tr.transform.rotation.w = qw;
            tf_->sendTransform(tr);
        }
    }

    void on_imu(const fw::ImuBody& i, double t_host) {
        if (std::isnan(t_host)) return;
        sensor_msgs::msg::Imu m;
        m.header.stamp = stamp_of(t_host);
        m.header.frame_id = "imu_link";
        m.orientation.w = i.quat[0], m.orientation.x = i.quat[1], m.orientation.y = i.quat[2], m.orientation.z = i.quat[3];
        const double rp = (0.5 * DEG) * (0.5 * DEG);     // roll / pitch (est), yaw drifts (game rotation vector)
        m.orientation_covariance = {rp, 0, 0, 0, rp, 0, 0, 0, (1.0 * DEG) * (1.0 * DEG)};
        m.angular_velocity.x = i.gyro[0], m.angular_velocity.y = i.gyro[1], m.angular_velocity.z = i.gyro[2];
        const double g = 0.01 * DEG * std::sqrt(200.0), b = 0.5 / 60 * DEG;      // BNO085 (est, as sensors.py)
        const double sg = g * g + b * b;
        m.angular_velocity_covariance = {sg, 0, 0, 0, sg, 0, 0, 0, sg};
        m.linear_acceleration.x = i.accel[0], m.linear_acceleration.y = i.accel[1], m.linear_acceleration.z = i.accel[2];
        const double na = 150e-6 * 9.81 * std::sqrt(200.0);
        const double sa = na * na + 0.1 * 0.1;
        m.linear_acceleration_covariance = {sa, 0, 0, 0, sa, 0, 0, 0, sa};
        p_imu_->publish(m);
    }

    void publish_samples() {
        size_t n = samples_.size() / N_COLS;
        if (!n) return;
        std_msgs::msg::Float32MultiArray m;
        m.layout.dim.resize(2);
        m.layout.dim[0].label = "rows", m.layout.dim[0].size = (uint32_t)n, m.layout.dim[0].stride = (uint32_t)samples_.size();
        m.layout.dim[1].label = SAMPLE_COLS, m.layout.dim[1].size = N_COLS, m.layout.dim[1].stride = N_COLS;
        m.data.swap(samples_);
        samples_.clear();
        p_samples_->publish(m);
    }

    void report() {
        JsonObject d, c;
        {
            std::lock_guard<std::mutex> g(mtx_);
            const LinkStats& st = link_->stats;
            d.boolean("connected", connected_).string("port", port_).integer("states", st.states)
                .integer("imus", st.imus).integer("cmds", st.cmds).integer("errors", st.errors)
                .number("rtt_ms", link_->sync.rtt_ns.value_or(0) * 1e-6, 3);
            if (has_state_) {
                d.string("mode", mode_name(last_state_.mode)).strings("faults", fault_names(last_state_.faults))
                    .integer("mcu_rx_errors", last_state_.rx_errors)
                    .integer("mcu_cmd_rejected", last_state_.cmd_rejected)
                    .integer("mcu_tick_late", last_state_.tick_late)
                    .integer("mcu_tick_max_us", last_state_.tick_max_us);
            } else {
                d.null("mode").null("faults").null("mcu_rx_errors").null("mcu_cmd_rejected").null("mcu_tick_late")
                    .null("mcu_tick_max_us");
            }
            d.integer("infos", st.infos);
            if (link_->has_info) d.integer("protocol", link_->info.protocol);
            else d.null("protocol");
            d.integer("protocol_host", fw::PROTOCOL_VERSION);
            if (link_->has_info)
                d.string("fw_build", std::string(link_->info.build, strnlen(link_->info.build, sizeof link_->info.build)));
            else d.null("fw_build");
            std_msgs::msg::String cs;
            cs.data = link_->sync.status_json(sync_source_);
            p_clock_->publish(cs);
        }
        std_msgs::msg::String ls;
        ls.data = d.str();
        p_link_->publish(ls);
    }

    std::string port_, sync_source_, last_mode_;
    bool publish_tf_ = true;
    double cmd_timeout_ = 0.3;
    std::unique_ptr<params::Live> live_;
    std::unique_ptr<HostLink> link_;
    std::unique_ptr<Transport> tr_;
    std::unique_ptr<EdgeReader> edges_;
    std::mutex mtx_;              // the link (its clock map), the request, the last state
    std::mutex io_mtx_;           // the transport's open / close / write
    std::atomic<bool> connected_{false}, running_{true};
    std::thread reader_;
    Request req_;
    double t_req_ = -1e18;
    fw::StateBody last_state_{};
    bool has_state_ = false;
    int64_t n_state_ = 0;
    bool want_samples_ = false, has_t0_ = false;
    uint64_t t0_ = 0;
    std::vector<float> samples_;
    double odom_var_[3] = {0, 0, 0};

    rclcpp::Publisher<omni_interfaces::msg::McuState>::SharedPtr p_state_, p_state10_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr p_samples_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr p_odom_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr p_imu_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr p_link_, p_clock_;
    rclcpp::Publisher<omni_interfaces::msg::McuInfo>::SharedPtr p_info_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr p_tref_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<omni_interfaces::msg::McuCommand>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sub_edge_;
    rclcpp::TimerBase::SharedPtr t_send_, t_report_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::McuLink)
