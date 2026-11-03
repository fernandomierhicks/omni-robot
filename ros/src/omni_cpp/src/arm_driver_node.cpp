// arm_driver: the SO-101's servo bus and joints in C++ (design.md 11, 4.10): the every-cycle half of omni_arm's arm_node,
// so the tidy task and the planning can sleep in Python between requests. It owns the bus (SYNC_READ then SYNC_WRITE
// each cycle), the calibration, the joint states and TF from the joints, and /arm/out; it plays the motions the arm
// brain (omni_arm's arm_node, launch arm_impl:=cpp) sends, one at a time. Nothing else writes /arm/motion.
//
// Subscribes /arm/motion (omni_interfaces/ArmMotion: MOVE waypoints with quintic or linear segments, HOLD, RELAX).
// Publishes /arm/joint_states (every cycle), /tf base_link -> arm_shoulder .. arm_gripper, arm_jaw, arm_tool,
// arm_wrist_cam_optical (tf_hz, default every cycle), /arm/out (Bool, 10 Hz), /arm/hw (omni_interfaces/ArmHardware: 5 Hz
// and on a change of motion). Parameters: port ("tcp:host:port" | "serial:/dev/ttyACM0" | a device), calibration (a
// LeRobot calibration json; "" = the sim's nominal one), rate (50), tf_hz (0 = rate), config ("" | d435 | a json path).
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
#include <optional>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omni_cpp/arm_kin.hpp"
#include "omni_cpp/arm_motion.hpp"
#include "omni_cpp/feetech.hpp"
#include "omni_interfaces/msg/arm_hardware.hpp"
#include "omni_interfaces/msg/arm_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace omni {

using omni_interfaces::msg::ArmHardware;
using omni_interfaces::msg::ArmMotion;

namespace {

constexpr double kPi = 3.14159265358979323846;

constexpr int ARM_TORQUE = 1000;      // torque limit, 0.1% of stall: the arm's joints at full torque
constexpr int GRIP_TORQUE = 500;      // the gripper at half (driver.py)
constexpr double RETRY_S = 2.0;       // between attempts to open the bus
constexpr double BUS_TIMEOUT = 0.02;  // s per reply

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
        port_spec_ = declare_parameter<std::string>("port", "tcp:127.0.0.1:5763");
        std::string cal = declare_parameter<std::string>("calibration", "");
        rate_ = declare_parameter<double>("rate", 50.0);
        double tf_hz = declare_parameter<double>("tf_hz", 0.0);
        std::string config = declare_parameter<std::string>("config", "");
        fk_ = std::make_unique<ArmFk>(config_file(config));
        for (int i = 0; i < 6; ++i) cal_[i] = sts::nominal_calibration(fk_->servo_ids.at(i), fk_->lo[i], fk_->hi[i]);
        if (!cal.empty()) load_calibration(cal);
        tf_every_ = tf_hz > 0 ? std::max(1, (int)std::lround(rate_ / tf_hz)) : 1;
        out_every_ = std::max(1, (int)(rate_ / 10));
        hw_every_ = std::max(1, (int)(rate_ / 5));

        p_js_ = create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);
        p_out_ = create_publisher<std_msgs::msg::Bool>("/arm/out", 10);
        p_hw_ = create_publisher<ArmHardware>("/arm/hw", 10);
        p_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
        s_motion_ = create_subscription<ArmMotion>("/arm/motion", 10, [this](ArmMotion::ConstSharedPtr m) { on_motion(*m); });
        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate_), [this] { tick(); });
        RCLCPP_INFO(get_logger(), "arm driver on %s at %.0f Hz (%s)", port_spec_.c_str(), rate_, config_file(config).c_str());
    }

private:
    double now_s() { return now().seconds(); }

    void load_calibration(const std::string& path) {
        std::ifstream f(path);
        if (!f) throw std::runtime_error("calibration: can't read " + path);
        std::stringstream ss;
        ss << f.rdbuf();
        Json raw = Json::parse(ss.str());
        for (int i = 0; i < 6; ++i) {
            const Json& j = raw[fk_->joints.at(i)];
            sts::JointCal c;
            c.id = (int)j["id"].integer();
            c.drive_mode = j.has("drive_mode") ? (int)j["drive_mode"].integer() : 0;
            c.homing_offset = j.has("homing_offset") ? (int)j["homing_offset"].integer() : 0;
            c.range_min = (int)j["range_min"].integer();
            c.range_max = (int)j["range_max"].integer();
            cal_[i] = c;
        }
    }

    // -- the bus --------------------------------------------------------------------------------------------------
    void connect() {
        if (bus_ || steady_s() - t_connect_ < RETRY_S) return;
        t_connect_ = steady_s();
        auto port = std::make_unique<Transport>();
        std::string err;
        if (!port->open(port_spec_, err)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "servo bus %s: %s (retrying)", port_spec_.c_str(), err.c_str());
            return;
        }
        auto bus = std::make_unique<sts::Bus>(*port, BUS_TIMEOUT);
        try {
            std::vector<int> missing;
            for (int id : fk_->servo_ids)
                if (!bus->ping(id)) missing.push_back(id);
            if (!missing.empty()) {
                std::string s;
                for (int id : missing) s += " " + std::to_string(id);
                RCLCPP_WARN(get_logger(), "servo bus: no answer from ids%s (retrying)", s.c_str());
                return;
            }
            port_ = std::move(port);
            bus_ = std::move(bus);
            enable(true);
            read();
            goal_ = q_, have_goal_ = true;
            play_.reset();
            moving_ = false;
            RCLCPP_INFO(get_logger(), "arm connected on %s: holding where it is", port_spec_.c_str());
            publish_hw();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "servo bus %s: %s (retrying)", port_spec_.c_str(), e.what());
            bus_.reset();
            port_.reset();
        }
    }

    void lost(const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "servo bus lost: %s", e.what());
        bus_.reset();
        port_.reset();
        moving_ = false;
        play_.reset();
        publish_hw();
    }

    // torque on (each servo's limit first) or off (the arm goes limp: hold it, it has no brakes)
    void enable(bool on) {
        for (int i = 0; i < 6; ++i) {
            int id = cal_[i].id;
            if (on) {
                bus_->write(id, "torque_limit", i == 5 ? GRIP_TORQUE : ARM_TORQUE);
                bus_->write(id, "acceleration", 0);
            }
            bus_->write(id, "torque_enable", on ? 1 : 0);
        }
        torque_ = on;
    }

    void read() {
        auto st = bus_->sync_read_status(fk_->servo_ids);
        std::vector<int> miss;
        bool errors = false;
        for (int i = 0; i < 6; ++i) {
            auto it = st.find(cal_[i].id);
            if (it == st.end()) {
                miss.push_back(cal_[i].id);
                continue;
            }
            const sts::ServoStatus& r = it->second;
            errors = errors || !r.ok;              // an error reply (overload, heat, voltage): not ok (driver.py)
            q_[i] = cal_[i].to_rad(r.position);
            dq_[i] = cal_[i].sign() * r.speed * 2 * kPi / sts::TICKS;
            load_[i] = cal_[i].sign() * r.load;
            temp_[i] = r.temp_c;
            volts_ = r.volts;
        }
        missing_ = miss;
        servo_errors_ = errors;
    }

    void send(const Joints& g) {
        Joints q;
        std::vector<std::pair<int, int>> goals;
        for (int i = 0; i < 6; ++i) {
            q[i] = std::min(std::max(g[i], fk_->lo[i]), fk_->hi[i]);
            goals.push_back({cal_[i].id, cal_[i].to_ticks(q[i])});
        }
        goal_ = q;
        bus_->sync_write_goals(goals);
    }

    // -- commands -------------------------------------------------------------------------------------------------
    void on_motion(const ArmMotion& m) {
        motion_id_ = m.id;
        if (!bus_) {
            RCLCPP_WARN(get_logger(), "arm motion %u: the arm isn't connected", m.id);
            return;
        }
        try {
            if (m.command == ArmMotion::RELAX) {
                play_.reset(), moving_ = false;
                enable(false);
                have_goal_ = false;
            } else if (m.command == ArmMotion::HOLD) {
                play_.reset(), moving_ = false;
                if (!torque_) enable(true);
                goal_ = q_, have_goal_ = true;
            } else if (m.command == ArmMotion::MOVE) {
                size_t n = m.times.size();
                if (n == 0 || m.waypoints.size() != 6 * n) {
                    RCLCPP_WARN(get_logger(), "arm motion %u: %zu times, %zu waypoint values", m.id, n, m.waypoints.size());
                    return;
                }
                if (!torque_) enable(true), goal_ = q_, have_goal_ = true;
                std::vector<Joints> wp(n);
                for (size_t k = 0; k < n; ++k)
                    for (int i = 0; i < 6; ++i) wp[k][i] = m.waypoints[6 * k + i];
                wp[0] = goal_;                                          // from where the arm is commanded now
                play_ = ArmPlay(m.interpolation == ArmMotion::LINEAR ? ArmPlay::LINEAR : ArmPlay::QUINTIC, m.times, std::move(wp));
                moving_ = true;
            } else {
                RCLCPP_WARN(get_logger(), "arm motion %u: unknown command %u", m.id, m.command);
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
        if (!bus_) {
            if (++k_ % hw_every_ == 0) publish_hw();
            return;
        }
        try {
            read();
        } catch (const std::exception& e) {
            lost(e);
            return;
        }
        double t = now_s();
        publish_joints();
        bool was_moving = moving_;
        if (play_ && !play_->empty()) {
            goal_ = play_->sample(t);
            if (play_->done(t)) moving_ = false;
        }
        try {
            if (have_goal_ && torque_) send(goal_);
        } catch (const std::exception& e) {
            lost(e);
            return;
        }
        ++k_;
        if (k_ % out_every_ == 0) {
            std_msgs::msg::Bool b;
            b.data = fk_->tool_out(q_) || (have_goal_ && fk_->tool_out(goal_));
            p_out_->publish(b);
        }
        if (k_ % tf_every_ == 0) publish_tf();
        if (k_ % hw_every_ == 0 || was_moving != moving_) publish_hw();
    }

    // -- outputs --------------------------------------------------------------------------------------------------
    void publish_joints() {
        sensor_msgs::msg::JointState m;
        m.header.stamp = now();
        m.name = fk_->joints;
        m.position.assign(q_.begin(), q_.end());
        m.velocity.assign(dq_.begin(), dq_.end());
        m.effort.assign(load_.begin(), load_.end());
        p_js_->publish(m);
    }

    void publish_tf() {
        auto fk = fk_->fk(q_);
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
        ArmHardware m;
        m.header.stamp = now();
        m.connected = bus_ != nullptr;
        m.torque = bus_ && torque_;
        m.ok = bus_ && missing_.empty() && !servo_errors_;
        m.port = port_spec_;
        m.joint_names = fk_->joints;
        for (int id : missing_) m.missing.push_back((uint8_t)id);
        m.position.assign(q_.begin(), q_.end());
        if (have_goal_ && torque_) m.goal.assign(goal_.begin(), goal_.end());
        m.load.assign(load_.begin(), load_.end());
        for (int i = 0; i < 6; ++i) m.temp_c.push_back((uint8_t)temp_[i]);
        m.volts = volts_;
        m.tool_out = bus_ && fk_->tool_out(q_);
        m.motion_id = motion_id_;
        m.moving = moving_;
        p_hw_->publish(m);
    }

    std::string port_spec_;
    double rate_ = 50;
    int tf_every_ = 1, out_every_ = 5, hw_every_ = 10;
    long k_ = 0;
    std::unique_ptr<ArmFk> fk_;
    std::array<sts::JointCal, 6> cal_;
    std::unique_ptr<Transport> port_;
    std::unique_ptr<sts::Bus> bus_;
    double t_connect_ = -1e9;
    bool torque_ = false, have_goal_ = false, moving_ = false;
    Joints q_{}, dq_{}, load_{}, goal_{};
    std::array<int, 6> temp_{};
    double volts_ = 0;
    std::vector<int> missing_;
    bool servo_errors_ = false;
    std::optional<ArmPlay> play_;
    uint32_t motion_id_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr p_js_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr p_out_;
    rclcpp::Publisher<ArmHardware>::SharedPtr p_hw_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr p_tf_;
    rclcpp::Subscription<ArmMotion>::SharedPtr s_motion_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omni

RCLCPP_COMPONENTS_REGISTER_NODE(omni::ArmDriver)
