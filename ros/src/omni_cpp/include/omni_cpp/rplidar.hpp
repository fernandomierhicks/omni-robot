// The Slamtec RPLIDAR A2M8 (design.md 2, 4.6; characterization/lidar) without Slamtec's SDK or sllidar_ros2: its
// serial protocol, its three answer formats, and each sample's time from the device's own sample clock. ROS-free:
// rplidar_node runs it on the serial port; test/test_cores.py checks the decode against omni_console/rplidar.py (the
// console's reader) on the bench's recordings, and the stamps against known sample times.
//
// Why its own driver (Nov 16): the device sends no times, and sllidar_ros2 stamps a turn when the previous turn was
// handed over: late by 36-61 ms in Stability, jittering by up to a capsule. In the sim that took slam_toolbox's
// heading error from 0.6 to 9.7 deg p95 (simulation/v11 sil/studies/results/lidar_a2m8_slam_2026-11-16.md). But the
// device samples at an exact rate (+/-10 ppm on the bench), so a sample's time is its count:
//
//   t(n) = line(n + 1) - delay       line: the lower envelope of the packets' arrivals against their sample counts
//                                    (a packet whose last sample is n_end - 1 can't arrive before line(n_end))
//                                    delay: the packet's transmission (bytes x 10 / baud) + latency (a parameter: the
//                                    device's processing and USB's fastest delivery, 0.5 ms est; omni_calib measures it)
//
// The envelope comes from every packet (42 a second in Stability), so neither USB nor the host's read jitter nor the
// motor's speed (open loop, it creeps ~5 % while warming) moves the stamps. A lost packet is counted from its gap in
// the encoder angle (the capsules carry it); a gap that can't be counted starts a new line (SampleClock::lines).
//
// Turns follow the encoder angle and are re-binned by the reported bearing (nearest return per bin), REP-103 in the
// laser frame (x at the lidar's 0 deg mark, counter-clockwise; the device's angles run clockwise): bin i at angle
// i x 2 pi / bins. The head turns clockwise, so the array runs against time: bin bins - 1 is measured first, at the
// stamp, and bin i at stamp + (bins - 1 - i) x time_increment (scan_prep's time_order -1). A turn starts half a bin
// past the 0 mark, so the seam falls between bins; it follows the encoder angle plus the device's typical bearing
// offset (Ultra: -8 deg, the SDK's). Its stamp and time_increment are a line fitted to its samples' times against
// those angles; a sample whose own (parallax-corrected) bearing lands across the seam (near things, within ~10 deg
// of it) is dropped: its time would be a turn off.
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace omni::rplidar {

// requests (A5 CMD [len payload checksum])
constexpr uint8_t STOP = 0x25, RESET = 0x40, SCAN = 0x20, EXPRESS_SCAN = 0x82, GET_INFO = 0x50, GET_HEALTH = 0x52,
                  GET_SAMPLERATE = 0x59, GET_CONF = 0x84, SET_PWM = 0xF0;
// GET_LIDAR_CONF keys
constexpr uint32_t CONF_COUNT = 0x70, CONF_US_PER_SAMPLE = 0x71, CONF_MAX_DIST = 0x74, CONF_ANS_TYPE = 0x75,
                   CONF_TYPICAL = 0x7C, CONF_NAME = 0x7F;
// the scan answers: standard samples (5 bytes), express capsules (84 bytes, 32 samples), ultra capsules (132, 96)
constexpr uint8_t ANS_STANDARD = 0x81, ANS_EXPRESS = 0x82, ANS_ULTRA = 0x84;

std::vector<uint8_t> request(uint8_t cmd, const std::vector<uint8_t>& payload = {});

// an answer's 7-byte descriptor: A5 5A, size (30 bits) and mode (2 bits), type
struct Descriptor {
    uint32_t size = 0;
    uint8_t mode = 0, type = 0;
};
bool descriptor(const uint8_t* d, size_t n, Descriptor& out);

std::string model_name(uint8_t model);   // 0x28 -> A2M8

struct Format {
    uint8_t type;
    int K;          // samples a packet
    int bytes;      // bytes a packet
    bool decode_next;   // a capsule's samples need the next capsule's start angle
};
Format format(uint8_t ans_type);   // throws std::invalid_argument for another type

struct Sample {
    int64_t n = 0;      // its place in the scan's sample count (lost packets counted)
    double enc = 0;     // deg, clockwise: the head's encoder angle (Standard: the reported angle)
    double angle = 0;   // deg, clockwise: the bearing the device reports (the SDK's parallax correction applied)
    double mm = 0;      // range, 0 = no return
    int quality = 0;
    bool start = false; // the device's new-turn flag
};

// a packet whose last sample is n_end - 1 had arrived (its last byte) by t
struct Anchor {
    int64_t n_end;
    double t;
    bool fresh;         // the count before it is unknown: the clock starts a new line
    size_t at;          // the samples decoded before it in the same feed (they're timed before it counts)
};

// the scan's bytes (after its descriptor) -> samples, and each packet's anchor
class Decoder {
public:
    Decoder(uint8_t ans_type, double period_s);
    void feed(const uint8_t* d, size_t n, double t, std::vector<Sample>& out, std::vector<Anchor>& anchors);

    const Format fmt;
    int64_t bad = 0;        // bytes skipped (framing, checksums)
    int64_t packets = 0;    // packets taken
    int64_t lost = 0;       // packets counted missing from the encoder angle
    int64_t breaks = 0;     // gaps that couldn't be counted (each a new clock line)

private:
    void capsule(const uint8_t* c, double t, std::vector<Sample>& out, std::vector<Anchor>& anchors);
    void standard(const uint8_t* s, double t, std::vector<Sample>& out, std::vector<Anchor>& anchors);
    void decode(const uint8_t* prev, const uint8_t* cur, int64_t n0, std::vector<Sample>& out) const;

    double period_;
    std::vector<uint8_t> buf_;
    bool gap_ = false;               // bytes skipped since the last good packet
    bool has_prev_ = false;          // the previous capsule, waiting for this one's start angle
    std::vector<uint8_t> prev_;
    int64_t prev_n_ = 0;
    int64_t next_n_ = 0;             // the next packet's first sample, if none is lost
    bool any_ = false;               // a packet taken yet
    double a_last_ = 0, t_last_ = 0; // the last good capsule's start angle (deg) and arrival
    double adv_ = 0;                 // deg a capsule (smoothed; 0: unknown)
};

// sample counts -> times: the lower envelope of the anchors (module comment)
class SampleClock {
public:
    SampleClock(double period_s, double delay_s, double window_s = 10.0, double bucket_s = 0.05, double fit_min_s = 3.0);
    void add(const Anchor& a);
    double time(int64_t n) const;    // sample n's time (the anchors' clock); NaN before the first anchor
    void set_delay(double delay_s) { delay_ = delay_s; }
    double delay() const { return delay_; }
    double residual(const Anchor& a) const;   // an anchor's arrival above the line (s): >= 0 within the envelope

    double period;                   // s a sample: nominal until the line spans fit_min_s, then fitted
    int lines = 0;                   // lines started (the first, then one per uncounted gap)
    double spread = 0;               // s: median of the buckets' lowest arrivals above the line (USB's floor jitter)
    double ppm() const { return (period / nominal_ - 1.0) * 1e6; }

private:
    struct Bucket {
        int64_t k;                   // floor(t / bucket_s) since the line began
        int64_t x;                   // n_end - n0
        double y;                    // t - t0
    };
    void fit();

    double nominal_, delay_, window_, bucket_, fit_min_;
    bool has_ = false;
    int64_t n0_ = 0;
    double t0_ = 0, c_ = 0, t_last_ = 0;     // line: t = t0 + c + period (n_end - n0)
    std::deque<Bucket> b_;
};

// one turn on the LaserScan grid (module comment)
struct Turn {
    double stamp = 0;            // bin bins - 1's time: the turn's start
    double time_increment = 0;   // s between bins, in time order
    double scan_time = 0;        // bins x time_increment
    std::vector<float> ranges;   // m by bin; +inf no return, NaN no sample in the bin
    int samples = 0;             // samples in the turn
    int dropped = 0;             // across the seam
    double fit_rms = 0;          // s: the samples' times about the stamp line
};

class TurnBuilder {
public:
    explicit TurnBuilder(int bins);
    // true when s began a new turn and the previous one is in out; line: the clock line t came from
    bool add(const Sample& s, double t, Turn& out, int line = 0);
    void reset();                                      // a new scan: no partial turn

    const int bins;
    int64_t turns = 0, dropped = 0;
    static constexpr double MAX_GAP_S = 0.1;   // samples further apart end a partial turn unpublished
    // at a turn's end its samples are timed again by the clock, which has more packets by then (a line just begun
    // has one); NaN keeps the time it had (its line has ended)
    std::function<double(int64_t n, int line)> retime;

private:
    bool finish(Turn& out);
    double inc_deg_;
    bool started_ = false;
    double off_ = 0, off_turn_ = 0;  // deg: the reported bearing less the encoder angle, smoothed; this turn's
    bool has_off_ = false;
    double p_max_ = -1;              // the turn's furthest progress past the cut (deg; -1: none yet)
    std::vector<double> t_, p_, ang_, mm_;
    std::vector<int64_t> n_;
    std::vector<int> line_;
};

struct StreamConfig {
    uint8_t ans_type = ANS_ULTRA;
    double period_s = 250e-6;        // Stability: 4 k samples/s
    int baud = 115200;
    double latency_s = 0.0005;
    int bins = 360;
    double window_s = 10.0;
};

// bytes from the port -> turns: Decoder -> SampleClock -> TurnBuilder
class LidarStream {
public:
    explicit LidarStream(const StreamConfig& c);
    std::vector<Turn> feed(const uint8_t* d, size_t n, double t);
    void set_latency(double latency_s);

    StreamConfig cfg;
    Decoder dec;
    SampleClock clock;
    TurnBuilder turns;
    std::vector<Sample> samples;     // the last feed's samples, with their times (for the checks)
    std::vector<double> times;
    std::vector<Anchor> anchors;     // the last feed's anchors
};

}  // namespace omni::rplidar
