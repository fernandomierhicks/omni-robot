// Teensy 4.1 pin map for the omni-robot MCU (design.md 4, 30), checked at compile time: a function on a pin that
// can't do it, or two functions on one pin, fails the build (static_assert). board_report() prints the table, and
// firmware/PINS.md is written from it (sil/build_sil.py).
//
// Pin capabilities are from PJRC's Teensy 4.1 pinout card (pjrc.com/teensy/card11a_rev4_web.pdf, and the "Teensy
// 4.1" page). The DShot pins depend on the bidirectional-DShot library chosen (FlexPWM + input capture): VERIFY.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace board {

// fixed-function pins (each peripheral instance has its own pins on the i.MX RT1062 as Teensy routes them)
enum Fn : uint8_t {
    GPIO, PWM, ANALOG,
    SER1_RX, SER1_TX, SER2_RX, SER2_TX, SER3_RX, SER3_TX, SER4_RX, SER4_TX, SER5_RX, SER5_TX, SER6_RX, SER6_TX,
    SER7_RX, SER7_TX, SER8_RX, SER8_TX,
    CAN1_TX, CAN1_RX, CAN2_TX, CAN2_RX, CAN3_TX, CAN3_RX,
    SPI0_MOSI, SPI0_MISO, SPI0_SCK, SPI1_MOSI, SPI1_MISO, SPI1_SCK,
    I2C0_SDA, I2C0_SCL, I2C1_SDA, I2C1_SCL, I2C2_SDA, I2C2_SCL,
};

struct FixedPin {
    Fn fn;
    uint8_t pin;
};

constexpr FixedPin FIXED[] = {
    {SER1_RX, 0}, {SER1_TX, 1}, {SER2_RX, 7}, {SER2_TX, 8}, {SER3_RX, 15}, {SER3_TX, 14}, {SER4_RX, 16},
    {SER4_TX, 17}, {SER5_RX, 21}, {SER5_TX, 20}, {SER6_RX, 25}, {SER6_TX, 24}, {SER7_RX, 28}, {SER7_TX, 29},
    {SER8_RX, 34}, {SER8_TX, 35},
    {CAN1_TX, 22}, {CAN1_RX, 23}, {CAN2_TX, 1}, {CAN2_RX, 0}, {CAN3_TX, 31}, {CAN3_RX, 30},
    {SPI0_MOSI, 11}, {SPI0_MISO, 12}, {SPI0_SCK, 13}, {SPI1_MOSI, 26}, {SPI1_MISO, 39}, {SPI1_SCK, 27},
    {I2C0_SDA, 18}, {I2C0_SCL, 19}, {I2C1_SDA, 17}, {I2C1_SCL, 16}, {I2C2_SDA, 25}, {I2C2_SCL, 24},
};

// PWM-capable pins (FlexPWM / QuadTimer) and analog inputs (A0-A17)
constexpr uint8_t PWM_PINS[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18, 19, 22, 23, 24, 25, 28, 29,
                                33, 36, 37, 42, 43, 44, 45, 46, 47, 51, 54};
constexpr uint8_t ANALOG_PINS[] = {14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 38, 39, 40, 41};
constexpr uint8_t N_PINS = 55;        // 0-41 on the edges, 42-47 SD pads, 48-54 bottom pads
constexpr uint8_t HEADER_PINS = 42;   // the ones a carrier board reaches without the pads

struct Use {
    uint8_t pin;
    Fn fn;
    const char* what;
};

// the pins by name (src/main.cpp uses these)
namespace pin {
constexpr uint8_t CAN1_TX = 22, CAN1_RX = 23, CAN3_TX = 31, CAN3_RX = 30;
constexpr uint8_t BNO_MOSI = 11, BNO_MISO = 12, BNO_SCK = 13, BNO_CS = 10, BNO_INT = 9, BNO_RST = 6, BNO_WAKE = 5;
constexpr uint8_t RC_RX = 7, RC_TX = 8, ROBOCLAW_RX = 15, ROBOCLAW_TX = 14, SERVO_BUS = 17;
constexpr uint8_t DSHOT_L = 2, DSHOT_R = 3;
constexpr uint8_t IR_EMIT = 36, BEAM_CATCH = 33, BEAM_FEED_EXIT = 32;
constexpr uint8_t ESTOP = 35, BUMPER_FRONT = 37, BUMPER_REAR = 38;
constexpr uint8_t V_BUS = 41, V_12 = 40, I_BUS = 39;
constexpr uint8_t BUZZER = 4, LED = 28;
}  // namespace pin

// THE PIN MAP. One line per signal.
constexpr Use PINS[] = {
    // drive motors: 4 x Damiao DM-2325 on CAN1 through a 3.3 V transceiver (SN65HVD230), 1 Mbit/s
    {pin::CAN1_TX, CAN1_TX, "CAN1 TX: DM-2325 x4"},
    {pin::CAN1_RX, CAN1_RX, "CAN1 RX: DM-2325 x4"},
    // payload connector bus (design.md 15: the module announces itself over CAN); CAN-FD capable
    {pin::CAN3_TX, CAN3_TX, "CAN3 TX: payload connector (module id), reserved"},
    {pin::CAN3_RX, CAN3_RX, "CAN3 RX: payload connector, reserved"},
    // IMU: BNO085 on SPI0 (400 Hz reports need SPI, not I2C)
    {pin::BNO_MOSI, SPI0_MOSI, "SPI0 MOSI: BNO085"},
    {pin::BNO_MISO, SPI0_MISO, "SPI0 MISO: BNO085"},
    {pin::BNO_SCK, SPI0_SCK, "SPI0 SCK: BNO085 (also the on-board LED: it flickers)"},
    {pin::BNO_CS, GPIO, "BNO085 CS"},
    {pin::BNO_INT, GPIO, "BNO085 INT (data ready, interrupt)"},
    {pin::BNO_RST, GPIO, "BNO085 RST"},
    {pin::BNO_WAKE, GPIO, "BNO085 PS0/WAKE"},
    // RC: ELRS receiver, CRSF at 420 kbaud (sticks + failsafe; design.md 15)
    {pin::RC_RX, SER2_RX, "Serial2 RX: ELRS receiver (CRSF)"},
    {pin::RC_TX, SER2_TX, "Serial2 TX: ELRS receiver (CRSF telemetry)"},
    // feed: RoboClaw 2x7A, packet serial (two Pololu 25D feed motors, its own speed / position PID)
    {pin::ROBOCLAW_RX, SER3_RX, "Serial3 RX: RoboClaw (feed motors)"},
    {pin::ROBOCLAW_TX, SER3_TX, "Serial3 TX: RoboClaw (feed motors)"},
    // tilt: Feetech STS3250 on its half-duplex TTL bus (1 Mbaud): Serial4's TX pin in half-duplex mode
    {pin::SERVO_BUS, SER4_TX, "Serial4 TX (half duplex): STS3250 tilt servo bus"},
    // flywheels: AM32 4-in-1 ESC, bidirectional DShot600 (eRPM telemetry on the same wire). VERIFY with the library
    {pin::DSHOT_L, PWM, "DShot ch1: flywheel L (FlexPWM4)"},
    {pin::DSHOT_R, PWM, "DShot ch2: flywheel R (FlexPWM4)"},
    // break-beams: one 38 kHz emitter drive for both, two demodulating receivers
    {pin::IR_EMIT, PWM, "IR emitters, 38 kHz (both beams)"},
    {pin::BEAM_CATCH, GPIO, "catch beam receiver"},
    {pin::BEAM_FEED_EXIT, GPIO, "feed-exit beam receiver"},
    // safety inputs (design.md 14)
    {pin::ESTOP, GPIO, "e-stop chain sense (relay open = 1)"},
    {pin::BUMPER_FRONT, GPIO, "bumper switches, front (wired-OR)"},
    {pin::BUMPER_REAR, GPIO, "bumper switches, rear (wired-OR)"},
    // power monitoring
    {pin::V_BUS, ANALOG, "A17: battery voltage (divider)"},
    {pin::V_12, ANALOG, "A16: 12 V rail"},
    {pin::I_BUS, ANALOG, "A15: battery current (hall sensor)"},
    // people-facing
    {pin::BUZZER, PWM, "buzzer (start-of-motion chirp, fire warning)"},
    {pin::LED, GPIO, "status LED"},
};
constexpr size_t N_USED = sizeof(PINS) / sizeof(PINS[0]);

// what the robot needs, one entry per signal: the build fails if the map is missing one
constexpr const char* REQUIRED[] = {
    "CAN1 TX", "CAN1 RX", "SPI0 MOSI", "SPI0 MISO", "SPI0 SCK", "BNO085 CS", "BNO085 INT", "BNO085 RST",
    "Serial2 RX", "Serial3 RX", "Serial3 TX", "Serial4 TX", "DShot ch1", "DShot ch2", "IR emitters",
    "catch beam", "feed-exit beam", "e-stop", "bumper switches, front", "bumper switches, rear", "A17: battery",
    "buzzer",
};

// -- compile-time checks ------------------------------------------------------------------------------------------

constexpr bool in_list(uint8_t pin, const uint8_t* list, size_t n) {
    for (size_t i = 0; i < n; ++i)
        if (list[i] == pin) return true;
    return false;
}

constexpr bool capable(const Use& u) {
    if (u.pin >= N_PINS) return false;
    switch (u.fn) {
        case GPIO: return true;
        case PWM: return in_list(u.pin, PWM_PINS, sizeof(PWM_PINS));
        case ANALOG: return in_list(u.pin, ANALOG_PINS, sizeof(ANALOG_PINS));
        default:
            for (const auto& f : FIXED)
                if (f.fn == u.fn) return f.pin == u.pin;
            return false;
    }
}

constexpr bool all_capable() {
    for (const auto& u : PINS)
        if (!capable(u)) return false;
    return true;
}

constexpr bool all_unique() {
    for (size_t i = 0; i < N_USED; ++i)
        for (size_t j = i + 1; j < N_USED; ++j)
            if (PINS[i].pin == PINS[j].pin) return false;
    return true;
}

constexpr bool starts_with(const char* s, const char* p) {
    while (*p) {
        if (*s != *p) return false;
        ++s;
        ++p;
    }
    return true;
}

constexpr bool all_required() {
    for (const char* r : REQUIRED) {
        bool found = false;
        for (const auto& u : PINS) found = found || starts_with(u.what, r);
        if (!found) return false;
    }
    return true;
}

constexpr size_t n_free_header() {
    size_t n = 0;
    for (uint8_t p = 0; p < HEADER_PINS; ++p) {
        bool used = false;
        for (const auto& u : PINS) used = used || u.pin == p;
        n += used ? 0 : 1;
    }
    return n;
}

static_assert(all_unique(), "board_teensy41.h: two signals on one pin");
static_assert(all_capable(), "board_teensy41.h: a signal on a pin that can't do it");
static_assert(all_required(), "board_teensy41.h: a required signal has no pin");
static_assert(n_free_header() >= 4, "board_teensy41.h: fewer than 4 spare header pins left");

// a text table of the map (for PINS.md and the boot log); returns the length written
size_t board_report(char* out, size_t cap);

}  // namespace board
