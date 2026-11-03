// Feetech STS3215 servo bus in C++ (design.md 11): omni_arm/feetech.py's packets, parser, calibration and Bus, for
// arm_driver. ROS-free; the port is anything with write / read (a TCP socket to the sim's bus, the bus board's serial
// port). omni_cpp/test checks it against the Python module.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace omni::sts {

constexpr uint8_t INST_PING = 0x01, INST_READ = 0x02, INST_WRITE = 0x03, INST_SYNC_READ = 0x82, INST_SYNC_WRITE = 0x83;
constexpr uint8_t BROADCAST = 0xFE;
constexpr int TICKS = 4096;
constexpr int STATUS_ADDR = 56, STATUS_LEN = 8;       // present position .. temperature, one SYNC_READ
constexpr double LOAD_UNIT = 0.001;

struct Reg {
    int addr, size;
};
Reg reg(const std::string& name);                     // throws std::out_of_range

uint8_t checksum(const uint8_t* body, size_t n);
std::vector<uint8_t> packet(uint8_t sid, uint8_t instr, const std::vector<uint8_t>& params = {});
std::vector<uint8_t> status_packet(uint8_t sid, uint8_t error, const std::vector<uint8_t>& params = {});
int to_sm(int v, int bit);
int from_sm(int v, int bit);

struct Packet {
    uint8_t id, len, code;                            // code: the instruction, or the status packet's error byte
    std::vector<uint8_t> params;
};

// Splits a byte stream into packets once the checksum is good
class Parser {
public:
    std::vector<Packet> feed(const uint8_t* data, size_t n);
    int errors = 0;

private:
    std::vector<uint8_t> buf_;
};

struct JointCal {
    int id = 0, drive_mode = 0, homing_offset = 0, range_min = 0, range_max = TICKS - 1;
    double mid() const { return 0.5 * (range_min + range_max); }
    double sign() const { return drive_mode ? -1.0 : 1.0; }
    double to_rad(int ticks) const;
    int to_ticks(double rad) const;                   // rounded half to even, clipped to the range
};

// The calibration of a servo set up so each joint's range sits symmetric about tick 2048 (the sim's)
JointCal nominal_calibration(int id, double lo, double hi);

struct ServoStatus {
    int position = 0, speed = 0;                      // ticks; ticks/s
    double load = 0, volts = 0;
    int temp_c = 0;
    bool ok = false;
};

class Port {
public:
    virtual ~Port() = default;
    virtual void write(const uint8_t* d, size_t n) = 0;                     // throws std::runtime_error when lost
    // up to cap bytes, waiting at most wait_ms for the first; 0 = nothing yet; throws when lost
    virtual size_t read(uint8_t* buf, size_t cap, int wait_ms) = 0;
};

// One half-duplex bus: write instructions, read status packets, timeout_s per reply
class Bus {
public:
    using Clock = std::function<double()>;
    explicit Bus(Port& port, double timeout_s = 0.01, Clock clock = nullptr);
    bool ping(int sid);
    bool write(int sid, const std::string& reg_name, int value, bool reply = true);
    void sync_write_goals(const std::vector<std::pair<int, int>>& goals);   // {id, ticks}: goal position, time 0, speed 0
    std::map<int, ServoStatus> sync_read_status(const std::vector<int>& ids);
    int n_timeouts = 0, n_tx = 0, n_rx = 0;
    Parser parser;

private:
    std::map<int, Packet> replies(const std::vector<int>& ids);
    void send(const std::vector<uint8_t>& d);
    Port& port_;
    double timeout_;
    Clock clock_;
};

}  // namespace omni::sts
