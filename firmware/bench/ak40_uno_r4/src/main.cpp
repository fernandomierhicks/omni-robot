// AK40-10 CAN bench: an Arduino UNO R4 WiFi stands in for the Teensy 4.1 (README.md here).
//
// Hardware: the R4's own CAN controller, CANTX D10 / CANRX D13, through a 3.3/5 V CAN transceiver board to one
// CubeMars AK40-10 V3.0 at 1 Mbit/s. Frames are built and decoded by the robot's codec (lib/omni_fw/ak_motor.h,
// servo mode). Frames the codec does not decode are printed raw, so a motor left in another mode is still visible.
//
// Serial (USB, 115200), one command per line; every motion command streams at 100 Hz for its duration (default
// 3000 ms, so the motor's own 100 ms timeout never trips mid-command), then sends zero current (coast) and stops:
//   v <erpm> [ms]            velocity, electrical rpm (output rpm x 10 gear x 14 pole pairs)
//   w <rad/s> [ms]           velocity, output shaft rad/s (converted with the robot's gear and pole pairs)
//   c <A> [ms]               q-axis current (torque)
//   b <A> [ms]               current brake (hold against motion)
//   pv <deg> <erpm> <erpm/s> [ms]   position with speed and acceleration limits (servo mode 6)
//   o                        set the origin here, TEMPORARY (mode 5, 0); the permanent origin is never sent
//   x                        stop now: zero current (coast)
//   t <hex id> <hex bytes>   send one raw extended frame (bring-up of other modes)
//   ts <hex id> <hex bytes>  send one raw standard frame
//   i <id>                   motor CAN id (default 1)
//   r                        toggle raw printing of every received frame
//   q <ms>                   status print period (default 100, 0 = off)
//   ?                        help
// Output lines: "S ..." decoded status (latest of the period, with the frame count), "F ..." a raw frame,
// "# ..." info / errors.
#include <Arduino.h>
#include <Arduino_CAN.h>
#include <stdarg.h>

#include "../../../lib/omni_fw/ak_motor.h"

using namespace fw;

// bench caps (well under the robot's: 60000 ERPM, ~4 N.m); raise only on purpose
static constexpr float BENCH_MAX_ERPM = 15000.0f;      // 107 rpm at the output
static constexpr float BENCH_MAX_ACC = 30000.0f;       // ERPM/s
static constexpr float BENCH_MAX_AMPS = 4.0f;
static constexpr uint32_t BENCH_MAX_MS = 60000;
static constexpr uint32_t DEFAULT_MS = 3000;
static constexpr uint32_t STREAM_MS = 10;              // 100 Hz command stream
static constexpr float GEAR = 10.0f, POLE_PAIRS = 14.0f;   // fw_params_arm.h AK_GEAR / AK_POLE_PAIRS (VERIFY)

enum Cmd { C_NONE, C_VEL, C_CUR, C_BRAKE, C_POSVEL };

static uint8_t g_id = 1;
static Cmd g_cmd = C_NONE;
static float g_val = 0, g_spd = 0, g_acc = 0;
static uint32_t g_until = 0, g_last_tx = 0;
static uint32_t g_print_ms = 100, g_last_print = 0;
static bool g_raw = false;

static AkStatus g_st;
static bool g_have_st = false;
static uint32_t g_n_status = 0, g_n_other = 0, g_n_other_printed = 0, g_tx_fail = 0, g_tx_ok = 0;
static uint32_t g_last_status_ms = 0;

static char g_line[96];
static size_t g_len = 0;

static void pf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void pf(const char* fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) Serial.write(buf, n < (int)sizeof buf ? n : (int)sizeof buf - 1);
}

static void print_frame(const char* tag, bool ext, uint32_t id, uint8_t len, const uint8_t* d) {
    char buf[64];
    int n = snprintf(buf, sizeof buf, "%s %lu %s %0*lX %u :", tag, (unsigned long)millis(), ext ? "E" : "S",
                     ext ? 8 : 3, (unsigned long)id, len);
    Serial.write(buf, n);
    for (uint8_t i = 0; i < len; i++) {
        n = snprintf(buf, sizeof buf, " %02X", d[i]);
        Serial.write(buf, n);
    }
    Serial.println();
}

static bool send(const FwCanFrame& f) {
    uint32_t id = f.ext ? (f.id | CanMsg::CAN_EFF_FLAG) : f.id;
    CanMsg m(id, f.len, f.data);
    int r = CAN.write(m);
    if (r < 0) {
        g_tx_fail++;
        return false;
    }
    g_tx_ok++;
    if (g_raw) print_frame("T", f.ext, f.id, f.len, f.data);
    return true;
}

static FwCanFrame posvel_cmd(uint8_t id, float deg, float erpm, float acc) {
    FwCanFrame f{};
    f.id = ((uint32_t)AK_POS_VEL << 8) | id;
    f.ext = 1;
    f.len = 8;
    int32_t p = (int32_t)lrintf(deg * 10000.0f);
    int16_t v = (int16_t)lrintf(erpm / 10.0f), a = (int16_t)lrintf(acc / 10.0f);
    f.data[0] = (uint8_t)(p >> 24); f.data[1] = (uint8_t)(p >> 16); f.data[2] = (uint8_t)(p >> 8); f.data[3] = (uint8_t)p;
    f.data[4] = (uint8_t)((uint16_t)v >> 8); f.data[5] = (uint8_t)v;
    f.data[6] = (uint8_t)((uint16_t)a >> 8); f.data[7] = (uint8_t)a;
    return f;
}

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void stop_now(const char* why) {
    // one frame: the library has a single extended TX mailbox, so a burst only fails while the first is on the bus;
    // the motor's own 100 ms timeout (set in the CubeMars tool) backs it up
    send(ak_current_cmd(g_id, 0.0f));
    if (g_cmd != C_NONE) pf("# stop (%s): zero current, coasting\n", why);
    g_cmd = C_NONE;
}

static uint32_t dur_arg(const char* s) {
    if (!s) return DEFAULT_MS;
    long ms = atol(s);
    if (ms <= 0) return DEFAULT_MS;
    return (uint32_t)ms > BENCH_MAX_MS ? BENCH_MAX_MS : (uint32_t)ms;
}

static void help() {
    Serial.println("# AK40-10 bench (UNO R4 WiFi, CAN D10/D13, 1 Mbit/s). Commands:");
    Serial.println("#  v <erpm> [ms] | w <rad/s> [ms] | c <A> [ms] | b <A> [ms] | pv <deg> <erpm> <erpm/s> [ms]");
    Serial.println("#  o (temporary origin) | x (stop) | t/ts <hex id> <hex bytes> | i <id> | r (raw) | q <ms> | ?");
    pf("#  caps: %.0f ERPM, %.0f ERPM/s, %.1f A, %lu ms; id %u\n", BENCH_MAX_ERPM, BENCH_MAX_ACC,
                  BENCH_MAX_AMPS, (unsigned long)BENCH_MAX_MS, g_id);
}

static void start(Cmd c, float val, uint32_t ms) {
    g_cmd = c;
    g_val = val;
    g_until = millis() + ms;
    g_last_tx = 0;          // send at once
}

static void handle(char* line) {
    char* tok[8] = {nullptr};
    int n = 0;
    for (char* p = strtok(line, " \t\r"); p && n < 8; p = strtok(nullptr, " \t\r")) tok[n++] = p;
    if (n == 0) return;
    const char* c = tok[0];
    if (!strcmp(c, "?")) { help(); return; }
    if (!strcmp(c, "x")) { stop_now("x"); g_cmd = C_NONE; Serial.println("# ok x"); return; }
    if (!strcmp(c, "d")) {
        // RA4M1 CAN0 error state. ECSR latches since the last clear: bit0 stuff, 1 form, 2 ACK, 3 CRC, 4 bit-recessive
        // (sent recessive, read dominant), 5 bit-dominant (sent dominant, read recessive: TX not reaching RX),
        // 6 ACK delimiter. ACK only = our frames are on the bus and no node acknowledges them
        uint8_t ecsr = R_CAN0->ECSR;
        pf("# diag ECSR=0x%02X TEC=%u REC=%u STR=0x%04X%s%s%s%s%s%s%s\n", ecsr, R_CAN0->TECR, R_CAN0->RECR,
           R_CAN0->STR, ecsr & 1 ? " stuff" : "", ecsr & 2 ? " form" : "", ecsr & 4 ? " ACK" : "",
           ecsr & 8 ? " crc" : "", ecsr & 16 ? " bit-rec" : "", ecsr & 32 ? " bit-dom" : "",
           ecsr & 64 ? " ackdelim" : "");
        R_CAN0->ECSR = 0x80;   // clear the codes, keep EDPM (accumulate)
        // CANRX level (D13): an idle bus reads high (recessive). Stuck low = the controller never sees bus idle,
        // never transmits: RX not wired / transceiver unpowered or in standby (D13 also carries the board LED)
        uint32_t low = 0, txlow = 0;
        for (int k = 0; k < 2000; k++) txlow += digitalRead(PIN_CAN0_TX) == LOW;
        pf("# CANTX D10 low in %lu of 2000 samples (idle controller: 0)\n", (unsigned long)txlow);
        pf("# PFS P103 0x%08lX P102 0x%08lX CTLR 0x%04X\n", (unsigned long)R_PFS->PORT[1].PIN[3].PmnPFS,
           (unsigned long)R_PFS->PORT[1].PIN[2].PmnPFS, R_CAN0->CTLR);
        for (int k = 0; k < 2000; k++) low += digitalRead(PIN_CAN0_RX) == LOW;
        pf("# CANRX D13 low in %lu of 2000 samples%s\n", (unsigned long)low,
           low > 1900 ? " (STUCK DOMINANT)" : (low == 0 ? " (idle recessive)" : " (traffic)"));
        return;
    }
    if (!strcmp(c, "dt")) {
        // isolate: D10 as GPIO high, then back to the CAN peripheral; D13 (left on CAN) read each time
        auto rx_low = []() { uint32_t lo = 0; for (int k = 0; k < 2000; k++) lo += digitalRead(PIN_CAN0_RX) == LOW; return lo; };
        pf("# dt CAN-mux D10=%d D13 low %lu\n", digitalRead(PIN_CAN0_TX), (unsigned long)rx_low());
        pinMode(PIN_CAN0_TX, OUTPUT);
        digitalWrite(PIN_CAN0_TX, HIGH);
        delay(5);
        pf("# dt GPIO-high D10=%d D13 low %lu PFS 0x%08lX\n", digitalRead(PIN_CAN0_TX), (unsigned long)rx_low(),
           (unsigned long)R_PFS->PORT[1].PIN[3].PmnPFS);
        R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_CAN0_TX].pin,
                        (uint32_t)IOPORT_CFG_PERIPHERAL_PIN | (uint32_t)IOPORT_PERIPHERAL_CAN);
        for (int t = 0; t < 4; t++) {
            delay(t == 0 ? 0 : 50);
            pf("# dt CAN-mux again +%dms D10=%d D13 low %lu ECSR 0x%02X STR 0x%04X\n", t * 50,
               digitalRead(PIN_CAN0_TX), (unsigned long)rx_low(), R_CAN0->ECSR, R_CAN0->STR);
        }
        return;
    }
    if (!strcmp(c, "dp")) {
        // wiring probe: which header pin carries the transceiver's RXD (idle high, toggling with the motor's 500 Hz
        // broadcast)? Then: is D13 driven low or floating (pull-up test)? Reconfigures D13, so it resets the board
        for (int pin = 0; pin <= 21; pin++) {
            if (pin == PIN_CAN0_TX) continue;
            uint32_t low = 0, edges = 0;
            int prev = digitalRead(pin);
            for (int k = 0; k < 20000; k++) {
                int v = digitalRead(pin);
                low += v == LOW;
                edges += v != prev;
                prev = v;
            }
            pf("# pin %s%d: low %lu/20000, edges %lu\n", pin >= 14 ? "A" : "D", pin >= 14 ? pin - 14 : pin,
               (unsigned long)low, (unsigned long)edges);
        }
        // loop through the transceiver: drive TXD by hand; RXD must follow (high = recessive, low = dominant)
        pinMode(PIN_CAN0_TX, OUTPUT);
        for (int lvl = 1; lvl >= 0; lvl--) {
            digitalWrite(PIN_CAN0_TX, lvl);
            delay(2);
            uint32_t lo = 0;
            for (int k = 0; k < 2000; k++) lo += digitalRead(PIN_CAN0_RX) == LOW;
            pf("# D10(TXD) %s -> D13(RXD) low %lu/2000\n", lvl ? "HIGH" : "LOW ", (unsigned long)lo);
        }
        digitalWrite(PIN_CAN0_TX, HIGH);
        pinMode(PIN_CAN0_RX, INPUT_PULLUP);
        delay(5);
        uint32_t low = 0;
        for (int k = 0; k < 2000; k++) low += digitalRead(PIN_CAN0_RX) == LOW;
        pf("# D13 with pull-up: low %lu/2000 (%s)\n", (unsigned long)low,
           low > 1900 ? "actively driven low" : (low == 0 ? "floating or high: nothing drives it low" : "mixed"));
        pf("# resetting to restore CAN\n");
        Serial.flush();
        delay(50);
        NVIC_SystemReset();
    }
    if (!strcmp(c, "r")) { g_raw = !g_raw; pf("# raw %s\n", g_raw ? "on" : "off"); return; }
    if (!strcmp(c, "q") && n >= 2) { g_print_ms = (uint32_t)atol(tok[1]); pf("# print %lu ms\n", (unsigned long)g_print_ms); return; }
    if (!strcmp(c, "i") && n >= 2) { g_id = (uint8_t)atoi(tok[1]); pf("# id %u\n", g_id); return; }
    if (!strcmp(c, "o")) {
        FwCanFrame f{};
        f.id = ((uint32_t)AK_ORIGIN << 8) | g_id;
        f.ext = 1;
        f.len = 1;
        f.data[0] = 0;      // temporary only
        pf("# origin (temporary) %s\n", send(f) ? "sent" : "TX FAIL");
        return;
    }
    if ((!strcmp(c, "t") || !strcmp(c, "ts")) && n >= 2) {
        FwCanFrame f{};
        f.ext = !strcmp(c, "t");
        f.id = strtoul(tok[1], nullptr, 16);
        for (int k = 2; k < n && f.len < 8; k++) f.data[f.len++] = (uint8_t)strtoul(tok[k], nullptr, 16);
        pf("# raw tx %s\n", send(f) ? "sent" : "TX FAIL");
        return;
    }
    if ((!strcmp(c, "v") || !strcmp(c, "w")) && n >= 2) {
        float e = atof(tok[1]);
        if (!strcmp(c, "w")) e = ak_erpm(e, GEAR, POLE_PAIRS);
        e = clampf(e, -BENCH_MAX_ERPM, BENCH_MAX_ERPM);
        uint32_t ms = dur_arg(n >= 3 ? tok[2] : nullptr);
        start(C_VEL, e, ms);
        pf("# ok vel %.0f ERPM (%.3f rad/s out) for %lu ms\n", e, ak_rad_s(e, GEAR, POLE_PAIRS), (unsigned long)ms);
        return;
    }
    if ((!strcmp(c, "c") || !strcmp(c, "b")) && n >= 2) {
        float a = clampf(atof(tok[1]), -BENCH_MAX_AMPS, BENCH_MAX_AMPS);
        uint32_t ms = dur_arg(n >= 3 ? tok[2] : nullptr);
        start(!strcmp(c, "c") ? C_CUR : C_BRAKE, a, ms);
        pf("# ok %s %.2f A for %lu ms\n", !strcmp(c, "c") ? "current" : "brake", a, (unsigned long)ms);
        return;
    }
    if (!strcmp(c, "pv") && n >= 4) {
        g_spd = clampf(fabsf(atof(tok[2])), 0, BENCH_MAX_ERPM);
        g_acc = clampf(fabsf(atof(tok[3])), 0, BENCH_MAX_ACC);
        uint32_t ms = dur_arg(n >= 5 ? tok[4] : nullptr);
        start(C_POSVEL, atof(tok[1]), ms);
        pf("# ok posvel %.2f deg at %.0f ERPM, %.0f ERPM/s for %lu ms\n", g_val, g_spd, g_acc, (unsigned long)ms);
        return;
    }
    pf("# ? unknown or incomplete: %s\n", c);
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}
    if (!CAN.begin(CanBitRate::BR_1000k)) {
        Serial.println("# CAN.begin FAILED");
    } else {
        Serial.println("# CAN up, 1 Mbit/s");
    }
    // Start-up quirk (found on the bench, Nov 12): straight after CAN.begin the transceiver's RXD (D13) sat low for
    // good, so the controller never saw an idle bus and never sent or received. Driving D10 high as a GPIO for a
    // moment, then handing it back to the CAN peripheral, releases it
    pinMode(PIN_CAN0_TX, OUTPUT);
    digitalWrite(PIN_CAN0_TX, HIGH);
    delay(5);
    R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_CAN0_TX].pin,
                    (uint32_t)IOPORT_CFG_PERIPHERAL_PIN | (uint32_t)IOPORT_PERIPHERAL_CAN);
    delay(5);
    {
        uint32_t lo = 0;
        for (int k = 0; k < 2000; k++) lo += digitalRead(PIN_CAN0_RX) == LOW;
        pf("# CANRX after the TX kick: low %lu/2000%s\n", (unsigned long)lo, lo > 1900 ? " STILL STUCK" : "");
    }
    help();
}

void loop() {
    // serial commands
    while (Serial.available()) {
        char ch = (char)Serial.read();
        if (ch == '\n') {
            g_line[g_len] = 0;
            handle(g_line);
            g_len = 0;
        } else if (g_len + 1 < sizeof g_line) {
            g_line[g_len++] = ch;
        }
    }

    // CAN receive
    while (CAN.available()) {
        CanMsg m = CAN.read();
        FwCanFrame f{};
        f.ext = m.isExtendedId();
        f.id = f.ext ? m.getExtendedId() : m.getStandardId();
        f.len = m.data_length;
        memcpy(f.data, m.data, f.len);
        AkStatus s;
        if (ak_decode_status(f, s)) {
            g_n_status++;
            g_last_status_ms = millis();
            if (s.id == g_id || !g_have_st) { g_st = s; g_have_st = true; }
            if (g_raw) print_frame("F", f.ext, f.id, f.len, f.data);
        } else {
            g_n_other++;
            // undecoded frames: always print the first 20, then everything only in raw mode
            if (g_raw || g_n_other_printed < 20) {
                g_n_other_printed++;
                print_frame("F", f.ext, f.id, f.len, f.data);
            }
        }
    }

    // command stream
    uint32_t now = millis();
    if (g_cmd != C_NONE) {
        if ((int32_t)(now - g_until) >= 0) {
            stop_now("duration over");
        } else if (g_last_tx == 0 || now - g_last_tx >= STREAM_MS) {
            g_last_tx = now ? now : 1;
            switch (g_cmd) {
                case C_VEL: send(ak_velocity_cmd(g_id, g_val)); break;
                case C_CUR: send(ak_current_cmd(g_id, g_val)); break;
                case C_BRAKE: send(ak_brake_cmd(g_id, g_val)); break;
                case C_POSVEL: send(posvel_cmd(g_id, g_val, g_spd, g_acc)); break;
                default: break;
            }
        }
    }

    // status print
    if (g_print_ms && now - g_last_print >= g_print_ms) {
        uint32_t dt = now - g_last_print;
        g_last_print = now;
        int err = 0;
        bool cerr = CAN.isError(err);
        if (g_have_st) {
            pf("S %lu id=%u pos=%.1f erpm=%.0f w=%.3f cur=%.2f T=%d err=%u n=%lu other=%lu tx=%lu txfail=%lu age=%lu%s\n",
                          (unsigned long)now, g_st.id, g_st.pos_deg, g_st.erpm,
                          ak_rad_s(g_st.erpm, GEAR, POLE_PAIRS), g_st.current, g_st.temp_c, g_st.err,
                          (unsigned long)g_n_status, (unsigned long)g_n_other, (unsigned long)g_tx_ok,
                          (unsigned long)g_tx_fail, (unsigned long)(now - g_last_status_ms), cerr ? " CANERR" : "");
        } else {
            pf("S %lu nostatus n=0 other=%lu tx=%lu txfail=%lu dt=%lu%s%s\n", (unsigned long)now,
                          (unsigned long)g_n_other, (unsigned long)g_tx_ok, (unsigned long)g_tx_fail,
                          (unsigned long)dt, cerr ? " CANERR=" : "", cerr ? String(err).c_str() : "");
        }
        if (cerr) CAN.clearError();
        g_n_status = 0;
        g_n_other = 0;
    }
}
