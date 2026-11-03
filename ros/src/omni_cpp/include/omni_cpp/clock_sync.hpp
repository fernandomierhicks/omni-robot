// The MCU's clock on the host's clock (design.md 10): t_host = a * t_mcu + b, offset and drift. C++ port of
// omni_mcu/clock_sync.py (no ROS; the link node uses it); test/test_cores.py checks the two against each other.
//
//   pulses  the Teensy's 1 Hz SYNC_OUT edge, reported by the MCU (a TIME event) and stamped by the host on a GPIO:
//           one point of the line per pulse; a least-squares line over pulse_window, late outliers rejected
//   echoes  the CMD / STATE exchange (NTP-style): the fastest exchange of each second over echo_window; the
//           fallback when no pulses come, and the coarse map that pairs a pulse with its edge
#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>

namespace omni {

struct ClockFit {
    double b, e;                 // host = h0 + x + b + e x, x = mcu - m0 (ns)
};

struct ClockSyncConfig {
    double echo_window_s = 30.0;
    double pulse_window_s = 120.0;
    double pulse_period_s = 1.0;
    int64_t edge_latency_ns = 0;  // the GPIO stamp's own fixed delay (bench: scope vs stamp)
    double refit_s = 0.25;
};

class ClockSync {
public:
    explicit ClockSync(const ClockSyncConfig& c = {});

    // evidence in
    void add(int64_t t1_host_ns, int64_t t2_mcu_us, int64_t t3_mcu_us, int64_t t4_host_ns);   // one echo
    void add_mcu_pulse(int64_t seq, int64_t t_mcu_us);
    void add_host_edge(int64_t t_host_ns);

    // mapping
    std::string mode() const;                  // "pulse" | "echo" | "none"
    double mcu_to_host_ns(double t_mcu_ns) const;   // NaN without a line
    double host_to_mcu_ns(double t_host_ns) const;
    double to_host_s(int64_t t_mcu_us) const;
    int64_t to_mcu_us(double t_host_s) const;       // 0 without a line
    std::optional<int64_t> offset_ns() const;
    std::string status_json(const std::string& source = "") const;   // /mcu/clock

    std::optional<int64_t> rtt_ns;             // the fastest round trip in the echo window
    int n_resets = 0, n_pulses = 0, n_edges = 0, n_paired = 0, n_rejected = 0;
    int n_echo_used = 0, n_pulse_used = 0;
    std::optional<ClockFit> echo_fit, pulse_fit;
    double echo_rms, pulse_rms;

private:
    struct Bin { int64_t rtt, mcu_mid, host_mid; };
    struct Pair { int64_t m, h, seq; };
    void reset(int64_t mcu_ns, int64_t host_ns);
    void fit_echo();
    void pair();
    void fit_pulses();
    const std::optional<ClockFit>& active() const;

    int64_t echo_window_ns_, pulse_window_ns_, period_ns_, edge_latency_ns_, refit_ns_;
    std::optional<int64_t> m0_, h0_;
    std::map<int64_t, Bin> bins_;              // host second -> the fastest exchange in it
    std::deque<Pair> pairs_;
    std::deque<std::pair<int64_t, int64_t>> mcu_pulses_;   // unpaired (seq, mcu ns)
    std::deque<int64_t> edges_;                // unpaired host edge stamps (ns)
    std::optional<int64_t> t_last_host_, t_refit_;
};

}  // namespace omni
