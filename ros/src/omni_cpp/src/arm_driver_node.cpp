// arm_driver: the SO-101's servo bus and joints in C++ (design.md 11, 4.10): the every-cycle half of the arm, so the tidy
// task and the planning (omni_arm's arm_brain) can sleep in Python between requests. The driving itself is ArmDriverCore
// (arm_driver_core.hpp: the bus, SYNC_READ then SYNC_WRITE each cycle, the calibration, the motion player), the code the
// v11 sim runs too; this node adds the port, the joint states, TF and /arm/out. It plays the motions the arm brain
// sends, one at a time. Nothing else writes /arm/motion.
//
// Subscribes /arm/motion (omni_interfaces/ArmMotion: MOVE waypoints with quintic or linear segments, HOLD, RELAX; guarded
// or not), imu_topic (sensor_msgs/Imu: gravity's direction in base_link for the load model; none: straight down).
// Publishes /arm/joint_states (every cycle; effort = the duty), /arm/servo (omni_interfaces/ArmServoState every cycle: the
// telemetry and the load estimate, design.md 4.5), /tf base_link -> arm_shoulder .. arm_gripper, arm_jaw, arm_tool,
// arm_wrist_cam_optical (tf_hz, default every cycle), /arm/out (Bool, 10 Hz), /arm/hw (omni_interfaces/ArmHardware: 5 Hz
// and on a change of motion or a reflex). Parameters: port ("tcp:host:port" | "serial:/dev/ttyACM0" | a device),
// calibration (a LeRobot calibration json; "" = the sim's nominal one), load_calibration (the arm load calibration's json,
// omni_calib arm_load; "" = the nominal model; reread when the file changes), rate (50), tf_hz (0 = rate), config
// ("" | d435 | a json path), the servo settings written at connect, the strain detector and the reflex.
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <filesystem>
#include <optional>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omni_cpp/arm_driver_core.hpp"
#include "omni_cpp/arm_kin.hpp"
#include "omni_cpp/arm_load.hpp"
#include "omni_cpp/params.hpp"
#include "omni_cpp/arm_motion.hpp"
#include "omni_cpp/feetech.hpp"
#include "omni_interfaces/msg/arm_hardware.hpp"
#include "omni_interfaces/msg/arm_motion.hpp"
#include "omni_interfaces/msg/arm_servo_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace omni {

using omni_interfaces::msg::ArmHardware;
using omni_interfaces::msg::ArmMotion;
using omni_interfaces::msg::ArmServoState;

namespace {


const char* const LINKS[] = {"arm_shoulder", "arm_upper_arm", "arm_lower_arm", "arm_wrist", "arm_gripper", "arm_jaw",
                             "tool"};

double steady_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// TCP to the sim's servo bus or the bus board's serial port (1 Mbaud)
class Transport : public sts::Port {
public:
    ~Transport() override { close(); }

    bool open(const std::string& spec, std::string& err) {
        std::string kind = "serial", rest = spec;
        if (spec.rfind("tcp:", 0) == 0) kind = "tcp", rest = spec.substr(4);
        else if (spec.rfind("serial:", 0) == 0) rest = spec.substr(7);
        return kind == "tcp" ? open_tcp(rest, err) : open_serial(rest, err);
    }

    void write(const uint8_t* d, size_t n) override {
        while (n) {
            ssize_t w = tcp_ ? ::send(fd_, d, n, MSG_NOSIGNAL) : ::write(fd_, d, n);
            if (w < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN) {                // a full buffer: wait for room
                    pollfd p{fd_, POLLOUT, 0};
                    poll(&p, 1, 50);
                    continue;
                }
                throw std::runtime_error(std::strerror(errno));
            }
            d += w, n -= (size_t)w;
        }
    }

    size_t read(uint8_t* buf, size_t cap, int wait_ms) override {
        pollfd p{fd_, POLLIN, 0};
        int r = poll(&p, 1, wait_ms);
        if (r == 0) return 0;
        if (r < 0) {
            if (errno == EINTR) return 0;
            throw std::runtime_error(std::strerror(errno));
        }
        if (p.revents & (POLLERR | POLLNVAL)) throw std::runtime_error("port error");
        ssize_t n = tcp_ ? ::recv(fd_, buf, cap, 0) : ::read(fd_, buf, cap);
        if (n == 0) {
            if (tcp_ || (p.revents & POLLHUP)) throw std::runtime_error("port closed");
            return 0;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) return 0;
            throw std::runtime_error(std::strerror(errno));
        }
        return (size_t)n;
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
        if (r < 0 && errno == EINPROGRESS) {          // a 1 s connect timeout
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
            cfsetspeed(&t, B1000000);
            t.c_cc[VMIN] = 0, t.c_cc[VTIME] = 0;
            tcsetattr(fd, TCSANOW, &t);
        }
        fd_ = fd, tcp_ = false;
        return true;
    }

    int fd_ = -1;
    bool tcp_ = false;
};

std::string config_file(const std::string& name) {
    std::string dir = ament_index_cpp::get_package_share_directory("omni_arm") + "/config/";
    if (name.empty()) return dir + "robot_arm.json";
    if (name.size() > 5 && name.compare(name.size() - 5, 5, ".json") == 0) return name;
    if (name.find('/') != std::string::npos || name.find('\\') != std::string::npos) return name;
    return dir + "robot_arm_" + name + ".json";
}

}  // namespace

class ArmDriver : public rclcpp::Node {
public:
    explicit ArmDriver(const rclcpp::NodeOptions& opt) : Node("arm_driver", opt) {
        namespace P = params;
        port_spec_ = P::text(*this, "port", "tcp:127.0.0.1:5763", "the servo bus: tcp:host:port (the sim) | serial:/dev/... | a device");
        std::string cal = P::text(*this, "calibration", "", "a LeRobot calibration json (\"\" = the sim's nominal one)");
        rate_ = P::number(*this, "rate", 50.0, 5.0, 200.0, "Hz: the bus cycle (read, play, write)", true);
        double tf_hz = P::number(*this, "tf_hz", 0.0, 0.0, 200.0, "Hz: the arm's TF (0 = every cycle)", true);
        std::string config = P::text(*this, "config", "", "the robot_arm*.json: \"\" | d435 | a path");
        load_path_ = P::text(*this, "load_calibration", "",
                             "the arm load calibration json (omni_calib arm_load; \"\" = the nominal model), reread when it changes", true);
        core_ = std::make_unique<ArmDriverCore>(config_file(config), cal, "");
        load_model();
        // the servo settings written at connect where they differ (design.md 4.5; est: LeRobot's SO-101 follower)
        ServoSettings& ss = core_->settings;
        ss.write = P::flag(*this, "servo_write_settings", true, "write the servo settings below at connect where they differ (EEPROM)");
        ss.p_gain = (int)P::integer(*this, "servo_p_gain", 16, 1, 64, "the servos' position P (Feetech 21; LeRobot 16, factory 32)");
        ss.d_gain = (int)P::integer(*this, "servo_d_gain", 32, 0, 64, "the servos' D (22)");
        ss.max_temp = (int)P::integer(*this, "servo_max_temp", 65, 40, 80, "C: a servo unloads past this (13)");
        ss.protect_current = (int)P::integer(*this, "servo_protect_current", 500, 50, 600, "6.5 mA: the arm joints' current limit (28)");
        ss.grip_protect_current = (int)P::integer(*this, "servo_grip_protect_current", 250, 50, 600, "6.5 mA: the gripper's (LeRobot 250)");
        ss.overload_torque = (int)P::integer(*this, "servo_overload_torque", 80, 10, 100, "%: the arm joints' overload level (36)");
        ss.grip_overload_torque = (int)P::integer(*this, "servo_grip_overload_torque", 25, 10, 100, "%: the gripper's (LeRobot 25)");
        // the strain detector and the reflex (design.md 4.5)
        core_->detector.tau = P::number(*this, "strain_tau", 0.04, 0.0, 1.0, "s: the outside torque's low-pass", true);
        core_->detector.n = (int)P::integer(*this, "strain_cycles", 4, 1, 50, "cycles over the threshold before a joint is strained", true);
        core_->reflex.on = P::flag(*this, "reflex", true, "a strained joint (guarded motion) stops the arm and backs it off");
        core_->reflex.nominal_ok = P::flag(*this, "reflex_nominal", false,
                                           "the reflex fires on the nominal model too (before the arm load calibration)");
        core_->reflex.backoff = P::number(*this, "reflex_backoff", 0.05, 0.0, 0.3, "rad: how far the strained joints give");
        core_->reflex.soft_torque = (int)P::integer(*this, "reflex_soft_torque", 350, 0, 1000,
                                                    "0.1 %: the strained joints' torque limit until the next command");
        core_->trim_cfg.on = P::flag(*this, "hold_trim", true,
                                     "holding still, nudge the goals sent until the encoders read the goal (the servos' droop)");
        core_->trim_cfg.rate = P::number(*this, "hold_trim_rate", 3.0, 0.0, 20.0, "1/s: the hold trim's integral rate");
        core_->trim_cfg.max = P::number(*this, "hold_trim_max", 0.07, 0.0, 0.2, "rad: the most the hold trim adds");
        core_->deflect.on = P::flag(*this, "deflect", true, "settled, a joint pushed off its goal is strained");
        core_->deflect.angle = P::number(*this, "deflect_angle", 0.012, 0.002, 0.1, "rad: how far it may give");
        core_->deflect.settle = P::number(*this, "deflect_settle", 1.0, 0.2, 5.0,
                                          "s after a motion: the hold trim settles the droop, then the cue watches");
        imu_topic_ = P::text(*this, "imu_topic", "/imu", "sensor_msgs/Imu: gravity's direction for the load model");
        core_->arm_torque = (int)P::integer(*this, "arm_torque", ARM_TORQUE, 0, 1000,
                                            "the arm joints' torque limit, 0.1 % of stall (written at each torque-on)");
        core_->grip_torque = (int)P::integer(*this, "grip_torque", GRIP_TORQUE, 0, 1000,
                                             "the gripper's torque limit, 0.1 % of stall (a stalled STS3215 heats)");
        bus_timeout_ = P::number(*this, "bus_timeout", 0.02, 0.002, 0.5, "s: the wait for each servo's reply", true);
        retry_s_ = P::number(*this, "retry_s", 2.0, 0.2, 60.0, "s between attempts to open the bus");
        live_ = std::make_unique<P::Live>(*this);
        live_->bind("arm_torque", &core_->arm_torque);
        live_->bind("grip_torque", &core_->grip_torque);
        live_->bind("retry_s", &retry_s_);
        live_->bind("reflex", &core_->reflex.on);
        live_->bind("reflex_nominal", &core_->reflex.nominal_ok);
        live_->bind("hold_trim", &core_->trim_cfg.on);
        live_->bind("hold_trim_rate", &core_->trim_cfg.rate);
        live_->bind("hold_trim_max", &core_->trim_cfg.max);
        live_->bind("deflect", &core_->deflect.on);
        live_->bind("deflect_angle", &core_->deflect.angle);
        live_->bind("deflect_settle", &core_->deflect.settle);
        live_->bind("reflex_backoff", &core_->reflex.backoff);
        live_->bind("reflex_soft_torque", &core_->reflex.soft_torque);
        tf_every_ = tf_hz > 0 ? std::max(1, (int)std::lround(rate_ / tf_hz)) : 1;
        out_every_ = std::max(1, (int)(rate_ / 10));
        hw_every_ = std::max(1, (int)(rate_ / 5));

        p_js_ = create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);
        p_out_ = create_publisher<std_msgs::msg::Bool>("/arm/out", 10);
        p_hw_ = create_publisher<ArmHardware>("/arm/hw", 10);
        p_servo_ = create_publisher<ArmServoState>("/arm/servo", 10);
        s_imu_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS(),
                                                            [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { on_imu(*m); });
        p_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
        s_motion_ = create_subscription<ArmMotion>("/arm/motion", 10, [this](ArmMotion::ConstSharedPtr m) { on_motion(*m); });
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this] { tick(); });
        RCLCPP_INFO(get_logger(), "arm driver on %s at %.0f Hz (%s)", port_spec_.c_str(), rate_, config_file(config).c_str());
    }

private:
    double now_s() { return now().seconds(); }

    // -- the load model (design.md 4.5) -----------------------------------------------------------------------------
    void load_model() {
        load_mtime_ = {};
        if (load_path_.empty()) {
            core_->set_load_model(LoadModel::nominal(core_->fk()));
            return;
        }
        try {
            core_->set_load_model(LoadModel::load(load_path_, core_->fk()));
            load_mtime_ = std::filesystem::last_write_time(load_path_);
            RCLCPP_INFO(get_logger(), "arm load model: %s", core_->model.source.c_str());
        } catch (const std::exception& e) {
            core_->set_load_model(LoadModel::nominal(core_->fk()));
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "arm load model %s: %s (the nominal model)",
                                 load_path_.c_str(), e.what());
        }
    }

    void check_load_file() {
        if (load_path_.empty()) return;
        std::error_code ec;
        auto t = std::filesystem::last_write_time(load_path_, ec);
        if (!ec && t != load_mtime_) load_model();
    }

    void on_imu(const sensor_msgs::msg::Imu& m) {
        // world -> base: gravity (0, 0, -9.81) in the IMU's frame (taken as base_link's; its mount offset is ~1 deg)
        double x = m.orientation.x, y = m.orientation.y, z = m.orientation.z, w = m.orientation.w;
        double n = std::sqrt(x * x + y * y + z * z + w * w);
        if (n < 0.5) return;
        x /= n, y /= n, z /= n, w /= n;
        // the third row of R (world from body) is the world z in body coordinates
        Vec3 up = {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
        core_->set_gravity({-9.81 * up[0], -9.81 * up[1], -9.81 * up[2]});
        t_imu_ = now_s();
    }

    // -- the bus (the driving itself is ArmDriverCore's, the code the sim runs too) ---------------------------------
    void connect() {
        if (core_->connected() || steady_s() - t_connect_ < retry_s_) return;
        t_connect_ = steady_s();
        auto port = std::make_unique<Transport>();
        std::string err;
        if (!port->open(port_spec_, err)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "servo bus %s: %s (retrying)", port_spec_.c_str(), err.c_str());
            return;
        }
        try {
            std::vector<int> missing = core_->connect(*port, bus_timeout_);
            if (!missing.empty()) {
                std::string s;
                for (int id : missing) s += " " + std::to_string(id);
                RCLCPP_WARN(get_logger(), "servo bus: no answer from ids%s (retrying)", s.c_str());
                return;
            }
            port_ = std::move(port);
            RCLCPP_INFO(get_logger(), "arm connected on %s: holding where it is", port_spec_.c_str());
            for (auto& n : core_->config_notes) RCLCPP_INFO(get_logger(), "servo settings: %s", n.c_str());
            publish_hw();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "servo bus %s: %s (retrying)", port_spec_.c_str(), e.what());
            core_->disconnect();
        }
    }

    void lost(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "servo bus lost: %s", e.what());
        core_->disconnect();
        port_.reset();
        publish_hw();
    }

    // -- commands -------------------------------------------------------------------------------------------------
    void on_motion(const ArmMotion& m) {
        motion_id_ = m.id;
        std::string why;
        try {
            if (!core_->command(m.command, m.interpolation == ArmMotion::LINEAR, m.times, m.waypoints, why,
                                m.guard != ArmMotion::GUARD_OFF)) {
                RCLCPP_WARN(get_logger(), "arm motion %u: %s", m.id, why.c_str());
                return;
            }
            publish_hw();
        } catch (const std::exception& e) {
            lost(e);
        }
    }

    // -- the cycle ------------------------------------------------------------------------------------------------
    void tick() {
        connect();
        if (!core_->connected()) {
            if (++k_ % hw_every_ == 0) publish_hw();
            return;
        }
        if (now_s() - t_imu_ > 1.0) core_->set_gravity({0, 0, -9.81});   // no IMU: straight down
        if (k_ % std::max(1, (int)rate_) == 0) check_load_file();
        int reflexes = core_->reflexes;
        try {
            core_->read(now_s());
        } catch (const std::exception& e) {
            lost(e);
            return;
        }
        publish_joints();
        bool was_moving = core_->moving;
        try {
            core_->drive(now_s());
        } catch (const std::exception& e) {
            lost(e);
            return;
        }
        ++k_;
        if (k_ % out_every_ == 0) {
            std_msgs::msg::Bool b;
            b.data = core_->tool_out();
            p_out_->publish(b);
        }
        if (k_ % tf_every_ == 0) publish_tf();
        publish_servo();
        if (core_->reflexes != reflexes)
            RCLCPP_WARN(get_logger(), "arm strained: %s; stopped and backed off", core_->reflex_why.c_str());
        if (k_ % hw_every_ == 0 || was_moving != core_->moving || core_->reflexes != reflexes) publish_hw();
    }

    void publish_servo() {
        const ArmDriverCore& c = *core_;
        ArmServoState m;
        m.header.stamp = now();
        auto put = [](std::vector<double>& v, const Joints& a) { v.assign(a.begin(), a.end()); };
        put(m.position, c.q), put(m.position_raw, c.q_enc), put(m.velocity, c.dq), put(m.goal, c.goal);
        put(m.goal_velocity, c.goal_vel), put(m.goal_acceleration, c.goal_acc), put(m.duty, c.load);
        if (c.has_current) put(m.current, c.current);
        put(m.volts, c.volts_j), put(m.gravity, c.est.grav), put(m.motor, c.est.motor), put(m.external, c.est.ext);
        put(m.band, c.est.band), put(m.filtered, c.detector.filt);
        for (int i = 0; i < 6; ++i) {
            m.temp_c.push_back((uint8_t)c.temp_c[i]);
            m.status.push_back((uint8_t)c.status[i]);
            m.strained.push_back(c.detector.strained[i]);
        }
        for (int i = 0; i < 3; ++i) m.tool_force[i] = c.tool_force_n[i], m.gravity_vec[i] = c.gravity[i];
        m.guard = c.guard;
        p_servo_->publish(m);
    }

    // -- outputs --------------------------------------------------------------------------------------------------
    void publish_joints() {
        const ArmDriverCore& c = *core_;
        sensor_msgs::msg::JointState m;
        m.header.stamp = now();
        m.name = c.fk().joints;
        m.position.assign(c.q.begin(), c.q.end());
        m.velocity.assign(c.dq.begin(), c.dq.end());
        m.effort.assign(c.load.begin(), c.load.end());
        p_js_->publish(m);
    }

    void publish_tf() {
        auto fk = core_->fk().fk(core_->q);
        tf2_msgs::msg::TFMessage msg;
        auto stamp = now();
        auto add = [&](const std::string& child, const Mat4& T) {
            geometry_msgs::msg::TransformStamped tr;
            tr.header.stamp = stamp, tr.header.frame_id = "base_link", tr.child_frame_id = child;
            tr.transform.translation.x = T[3], tr.transform.translation.y = T[7], tr.transform.translation.z = T[11];
            auto q = quat_of(T);
            tr.transform.rotation.x = q[0], tr.transform.rotation.y = q[1], tr.transform.rotation.z = q[2], tr.transform.rotation.w = q[3];
            msg.transforms.push_back(tr);
        };
        for (const char* name : LINKS) add(std::string(name) == "tool" ? "arm_tool" : name, fk.at(name));
        Mat4 optical = mat_mul(fk.at("wrist_cam"), Mat4{1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1});   // MuJoCo camera -> optical
        add("arm_wrist_cam_optical", optical);
        p_tf_->publish(msg);
    }

    void publish_hw() {
        const ArmDriverCore& c = *core_;
        bool on = c.connected();
        ArmHardware m;
        m.header.stamp = now();
        m.connected = on;
        m.torque = on && c.torque;
        m.ok = c.ok();
        m.port = port_spec_;
        m.joint_names = c.fk().joints;
        for (int id : c.missing) m.missing.push_back((uint8_t)id);
        m.position.assign(c.q.begin(), c.q.end());
        if (c.have_goal && c.torque) m.goal.assign(c.goal.begin(), c.goal.end());
        m.load.assign(c.load.begin(), c.load.end());
        for (int i = 0; i < 6; ++i) m.temp_c.push_back((uint8_t)c.temp_c[i]);
        m.volts = c.volts;
        m.tool_out = on && c.fk().tool_out(c.q);
        m.motion_id = motion_id_;
        m.moving = c.moving;
        if (c.has_current) m.current.assign(c.current.begin(), c.current.end());
        m.external.assign(c.est.ext.begin(), c.est.ext.end());
        for (int i = 0; i < 6; ++i) m.strained.push_back(c.detector.strained[i]);
        for (int i = 0; i < 3; ++i) m.tool_force[i] = c.tool_force_n[i];
        m.load_model = c.model.source;
        m.reflexes = (uint32_t)c.reflexes;
        m.reflex = c.reflex_why;
        m.reflex_active = c.reflex_active;
        m.servo_notes = c.config_notes;
        p_hw_->publish(m);
    }

    std::string port_spec_;
    double rate_ = 50;
    int tf_every_ = 1, out_every_ = 5, hw_every_ = 10;
    long k_ = 0;
    std::unique_ptr<ArmDriverCore> core_;
    std::unique_ptr<Transport> port_;
    double t_connect_ = -1e9, bus_timeout_ = 0.02, retry_s_ = 2.0;
    uint32_t motion_id_ = 0;
    std::unique_ptr<params::Live> live_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr p_js_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_out_;
    rclcpp::Publisher<ArmHardware>::SharedPtr p_hw_;
    rclcpp::Publisher<ArmServoState>::SharedPtr p_servo_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr s_imu_;
    std::string load_path_, imu_topic_;
    std::filesystem::file_time_type load_mtime_{};
    double t_imu_ = -1e9;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr p_tf_;
    rclcpp::Subscription<ArmMotion>::SharedPtr s_motion_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::ArmDriver)
