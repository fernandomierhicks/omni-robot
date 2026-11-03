// The C++ cores driven by text lines on stdin, for test_cores.py: it runs the same inputs through the Python cores
// and compares. One core per run (the first argument); one command per line; numbers printed with %.17g.
//
//   clock   cfg <echo_window_s> <pulse_window_s> <period_s> <edge_latency_ns> <refit_s> | echo <t1> <t2> <t3> <t4>
//           | pulse <seq> <t_us> | edge <t_ns> | host <t_mcu_us> | mcu <t_host_s> | status
//   link    feed <hex> <t_ns> | cmd <t_ns> <18 Request fields> | stats
//   mux     init <mode> <kid> | set <mode> <kid 0/1, -1: keep> | arm <0/1> | fire <seq>
//           | req <n | t vx vy wz | h tilt rpm seq | p vx vy wz>
//   heading cfg <mode> <spin_rate> <turn_max> <lead> <period> | odom <t> <x> <y> <yaw> <wz> | plan <t> <length>
//           | cmd <t> <vx> <vy> <wz> | step <t>
//   scan    floor <n> <tan_tilt> <azimuth> <height> <roll> <pitch> <margin> + a line of angles + a line of ranges
//           | deskew <n> <lx> <ly> <t_ref> <angle_min> <inc> <n_key> <n_out> <vx> <vy> <w> <x0> <y0> <yaw0>
//             + lines of angles, ranges, times (the robot moves at a constant twist from (x0, y0, yaw0))
//           | sclock <latency> | upd <stamp> <scan_time>
//   catch   (the catch cores, design.md 4.9) config <robot_v11.json> first, then
//           rim|exit <tilt> | speed|elev <v> <tilt> | tiltfor <v> <el> | accel <kid> | top <ux> <uy> <yaw> <kid>
//           | cross <p3> <v3> <z> | apex <p3> <v3> | release <p3> <v3> <z> | launch <exit3> <ux> <uy> <el> <target3>
//           | zone <hx> <hy> <robot_r> <margin> <cx> <cy> <n> <x y r>... | choose <p0 2> <v0 2> <pc 2> <T> <a> <vmax>
//             <coast> <a_ball> <moving> | pyfloat <v> | round <v> <nd>
//   detect  sparse <w> <h> + a line of ball indices + a line of clutter indices | rgb <w> <h> <bgr> + a hex line
//           | detect [<pu> <pv> <pr>] | cands <n>
//   percep  config <path> | area <cx> <cy> <hx> <hy> | pose <t> <x> <y> <yaw> <roll> <pitch>
//           | frame <t> <has_left> <has_right> + per frame present: "<w> <h>", ball indices, clutter indices
//   game    config <path> | param <name> <value> | new | rv <t> <x> <y> <yaw> <vbx> <vby> <wz> <vcx> <vcy> <tilt>
//           <tilt_cmd> <fly0> <fly1> <fly_target> <fly_ready> <ball_held> | track <t> <n> <ready> <x 6> <P 36>
//           | notrack | step <t>
//   arm     cfg <robot_arm.json> | fk|tool|toolout <q 6> | quat <16> | pkt|spkt <id> <code> <hex> | feed <hex>
//           | sm|fsm <v> <bit> | cal <id> <drive> <min> <max> | rad <ticks> | ticks <rad> | nominal <id> <lo> <hi>
//           | bus ping|write|read|goals ... | play <kind> <n> + a line of times and a line of n x 6 | sample <t>
//   mux     ... | catch <18 numbers: drive tx ty tyaw v_end t_brake coast a_brake tilt t_tilt tilt_after fly fire>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <memory>

#include "omni_cpp/arm_kin.hpp"
#include "omni_cpp/arm_motion.hpp"
#include "omni_cpp/clock_sync.hpp"
#include "omni_cpp/feetech.hpp"
#include "omni_cpp/detector.hpp"
#include "omni_cpp/game.hpp"
#include "omni_cpp/heading.hpp"
#include "omni_cpp/host_link.hpp"
#include "omni_cpp/intercept.hpp"
#include "omni_cpp/mode_mux.hpp"
#include "omni_cpp/perception.hpp"
#include "omni_cpp/scan.hpp"

using namespace omni;

static void num(double v) { std::printf("%.17g", v); }
static void nums(std::initializer_list<double> v) {
    bool first = true;
    for (double x : v) std::printf(first ? "%.17g" : " %.17g", x), first = false;
    std::printf("\n");
}

static std::string rest_of(std::istringstream& is) {
    std::string s;
    std::getline(is >> std::ws, s);
    return s;
}

static std::vector<double> read_line_nums() {
    std::string line;
    std::getline(std::cin, line);
    std::istringstream is(line);
    std::vector<double> v;
    std::string tok;
    while (is >> tok) v.push_back(std::strtod(tok.c_str(), nullptr));
    return v;
}

static std::vector<int64_t> read_line_ints() {
    std::vector<int64_t> v;
    for (double d : read_line_nums()) v.push_back((int64_t)d);
    return v;
}

static std::vector<uint8_t> read_line_hex() {
    std::string line;
    std::getline(std::cin, line);
    std::vector<uint8_t> b;
    for (size_t i = 0; i + 1 < line.size(); i += 2) b.push_back((uint8_t)std::stoi(line.substr(i, 2), nullptr, 16));
    return b;
}

static void print_blob(const char* tag, const DetBlob& d) {
    std::printf("%s ", tag);
    nums({d.u, d.v, d.b, (double)d.n});
}

// the catch cores' state across lines
struct CatchCheck {
    std::unique_ptr<RobotConfig> robot;
    std::unique_ptr<Perception> perc;
    GameParams gp;
    std::unique_ptr<CatchBehavior> game;
    RobotView rv;
    std::unique_ptr<TrackView> track;
    SafetyZone zone;
    Detector det;
    std::unique_ptr<Frame> frame;
    std::vector<uint8_t> rgb;
    std::vector<std::string> logs;
};

static std::unique_ptr<SparseFrame> read_sparse(int w, int h) {
    auto ball = read_line_ints(), clutter = read_line_ints();
    return std::make_unique<SparseFrame>(w, h, ball, clutter);
}

static void game_param(GameParams& p, const std::string& k, std::istringstream& is) {
    double v;
    std::string s;
    if (k == "return_to" || k == "catch_mode") {
        is >> s;
        (k == "return_to" ? p.return_to : p.catch_mode) = s;
        return;
    }
    is >> v;
    if (k == "thrower") p.thrower[0] = v, is >> p.thrower[1];
    else if (k == "home") p.home[0] = v, is >> p.home[1];
    else if (k == "arena_center") p.arena_center[0] = v, is >> p.arena_center[1];
    else if (k == "arena_half") p.arena_half[0] = v, is >> p.arena_half[1];
    else if (k == "return_throws") p.return_throws = v != 0;
    else if (k == "kid") p.kid = v != 0;
    else if (k == "gate") p.gate = v != 0;
    else if (k == "ceiling") p.ceiling = v;
    else if (k == "learn_thrower") p.learn_thrower = v != 0;
    else throw std::runtime_error("game param " + k);
}

static void run_catch(CatchCheck& c, const std::string& core, const std::string& op, std::istringstream& is) {
    if (op == "config") {
        c.robot = std::make_unique<RobotConfig>(rest_of(is));
        if (core == "percep") c.perc = std::make_unique<Perception>(*c.robot);
        return;
    }
    static const RobotConfig none;                     // the detector needs no robot
    const RobotConfig& R = c.robot ? *c.robot : none;
    BallFlight fl(R.ball_mass, R.ball_radius, R.ball_cd, R.air_density, R.air_viscosity);
    auto v3 = [&]() {
        V3 v;
        is >> v[0] >> v[1] >> v[2];
        return v;
    };
    if (core == "catch") {
        double a, b, d;
        if (op == "rim" || op == "exit") {
            is >> a;
            auto r = op == "rim" ? R.rim_center(a) : R.exit_point(a);
            nums({r[0], r[1]});
        } else if (op == "speed" || op == "elev" || op == "tiltfor") {
            is >> a >> b;
            nums({op == "speed" ? R.speed_for(a, b) : op == "elev" ? R.elevation_at(a, b) : R.tilt_for(a, b)});
        } else if (op == "accel") {
            is >> a;
            nums({R.plan_accel(a != 0)});
        } else if (op == "top") {
            double y;
            is >> a >> b >> y >> d;
            nums({R.top_speed(a, b, y, d != 0)});
        } else if (op == "cross") {
            V3 p = v3(), v = v3();
            is >> d;
            auto h = fl.crossing_state(p, v, d);
            if (!h) std::printf("none\n");
            else nums({h->pos[0], h->pos[1], h->pos[2], h->vel[0], h->vel[1], h->vel[2], h->t});
        } else if (op == "apex") {
            V3 p = v3(), v = v3();
            nums({fl.apex(p, v)});
        } else if (op == "release") {
            V3 p = v3(), v = v3();
            is >> d;
            auto r = fl.release_point(p, v, d);
            if (!r) std::printf("none\n");
            else nums({r->first[0], r->first[1], r->first[2], r->second});
        } else if (op == "launch") {
            V3 e = v3();
            double ux, uy, el;
            is >> ux >> uy >> el;
            V3 t = v3();
            nums({fl.solve_launch_speed(e, ux, uy, el, t)});
        } else if (op == "zone") {
            int n;
            c.zone = SafetyZone();
            is >> c.zone.half_x >> c.zone.half_y >> c.zone.robot_radius >> c.zone.margin >> c.zone.cx >> c.zone.cy >> n;
            for (int i = 0; i < n; ++i) {
                Person p;
                is >> p.x >> p.y >> p.r;
                c.zone.people.push_back(p);
            }
        } else if (op == "choose") {
            double p0[2], v0[2], pc[2], T, acc, vmax, coast, ab, mv;
            is >> p0[0] >> p0[1] >> v0[0] >> v0[1] >> pc[0] >> pc[1] >> T >> acc >> vmax >> coast >> ab >> mv;
            auto ch = choose_intercept(p0, v0, pc, T, acc, vmax, c.zone, coast, ab, mv != 0);
            nums({ch.v_f, ch.dx, ch.dy, (double)ch.feasible, (double)ch.gated});
        } else if (op == "pyfloat" || op == "round") {
            std::string tok;                       // strtod, not >>: libstdc++'s stream can't read "nan"
            int nd = 0;
            is >> tok >> nd;
            a = std::strtod(tok.c_str(), nullptr);
            if (op == "pyfloat") {
                std::printf("%s\n", py_float(a).c_str());
                return;
            }
            std::printf("%s\n", py_float(py_round(a, nd)).c_str());
        }
    } else if (core == "percep") {
        Perception& P = *c.perc;
        if (op == "area") {
            double cx, cy, hx, hy;
            is >> cx >> cy >> hx >> hy;
            P.set_play_area(cx, cy, hx, hy);
        } else if (op == "pose") {
            double t, x, y, yaw, roll, pitch;
            is >> t >> x >> y >> yaw >> roll >> pitch;
            P.add_pose(t, x, y, yaw, roll, pitch);
        } else if (op == "frame") {
            double t;
            int has[2];
            is >> t >> has[0] >> has[1];
            std::unique_ptr<SparseFrame> fr[2];
            std::vector<const Frame*> frs;
            for (int k = 0; k < 2; ++k) {
                if (has[k]) {
                    auto wh = read_line_nums();
                    fr[k] = read_sparse((int)wh[0], (int)wh[1]);
                }
                frs.push_back(fr[k].get());
            }
            auto blobs = P.process(t, frs);
            std::printf("B %zu %ld %ld %ld %ld\n", blobs.size(), P.frames, P.detections, P.stereo, P.det.n_coarse);
            auto v = P.view();
            if (!v) {
                std::printf("V none\n");
                return;
            }
            std::printf("V %.17g %d %d", v->t, v->n, (int)v->ready);
            for (double x : v->x) std::printf(" %.17g", x);
            for (double x : v->P.a) std::printf(" %.17g", x);
            std::printf("\n");
            auto cr = v->crossing(v->t, P.robot.rim_center(0.0)[1]);
            if (!cr) std::printf("C none\n");
            else
                std::printf("C %.17g %.17g %.17g %.17g %.17g\n", cr->pos[0], cr->pos[1], cr->pos[2], cr->t_go,
                            std::sqrt(eig2_max(cr->cov[0], cr->cov[2], cr->cov[3])));
        }
    } else if (core == "game") {
        if (op == "param") {
            std::string k;
            is >> k;
            game_param(c.gp, k, is);
        } else if (op == "new") {
            c.game = std::make_unique<CatchBehavior>(R, c.gp, [&c](const std::string& s) { c.logs.push_back(s); });
        } else if (op == "rv") {
            RobotView& v = c.rv;
            int ready, held;
            is >> v.t >> v.pose[0] >> v.pose[1] >> v.pose[2] >> v.vel_body[0] >> v.vel_body[1] >> v.vel_body[2] >>
                v.v_cmd[0] >> v.v_cmd[1] >> v.tilt >> v.tilt_cmd >> v.fly_rpm[0] >> v.fly_rpm[1] >> v.fly_target_rpm >>
                ready >> held;
            v.fly_ready = ready, v.ball_held = held;
        } else if (op == "notrack") {
            c.track.reset();
        } else if (op == "track") {
            double t;
            int n, ready;
            is >> t >> n >> ready;
            std::vector<double> x(6);
            Mat P(6, 6);
            for (auto& e : x) is >> e;
            for (auto& e : P.a) is >> e;
            c.track = std::make_unique<TrackView>(c.game->flight, 0.5, t, x, P, n, ready != 0);
        } else if (op == "step") {
            double t;
            is >> t;
            c.logs.clear();
            CatchCommand m = c.game->step(t, c.rv, c.track.get());
            for (auto& s : c.logs) std::printf("L %s\n", s.c_str());
            std::printf("S %s %d %d %u ", c.game->state.c_str(), c.game->tosses, c.game->catches, m.fire_seq);
            nums({(double)m.drive_mode, m.target[0], m.target[1], m.target[2], m.v_end, m.t_brake, m.coast_s, m.a_brake,
                  m.tilt, m.t_tilt, m.tilt_after, m.fly_rpm});
            std::printf("J %s\n", c.game->last.dump().c_str());
        }
    } else if (core == "detect") {
        if (op == "sparse") {
            int w, h;
            is >> w >> h;
            c.frame = read_sparse(w, h);
        } else if (op == "rgb") {
            int w, h, bgr;
            is >> w >> h >> bgr;
            c.rgb = read_line_hex();
            c.frame = std::make_unique<RgbFrame>(c.rgb.data(), w, h, (long)w * 3, 3, bgr != 0);
        } else if (op == "detect") {
            double pred[3];
            bool has = (bool)(is >> pred[0] >> pred[1] >> pred[2]);
            auto d = c.det.detect(*c.frame, has ? pred : nullptr);
            if (!d) std::printf("none %ld\n", c.det.n_coarse);
            else print_blob("D", *d);
        } else if (op == "cands") {
            int n;
            is >> n;
            auto cs = c.det.candidates(*c.frame, n);
            std::printf("N %zu\n", cs.size());
            for (auto& d : cs) print_blob("D", d);
        }
    }
}

static void print_request(const Request& r) {
    std::printf("%d %d %d %d ", r.req_mode, (int)r.kid, r.drive_mode, (int)r.hold_yaw);
    for (double v : {r.a, r.b, r.c, r.v_end, r.a_cap, r.speed_cap, r.t_brake, r.coast_s, r.a_brake, r.tilt, r.t_tilt,
                     r.tilt_after, r.fly_rpm}) {
        num(v);
        std::printf(" ");
    }
    std::printf("%d\n", r.fire_seq);
}

// -- the arm cores (design.md 11): the STS3215 protocol, calibration, forward kinematics and the motion player --------
namespace {

std::vector<uint8_t> from_hex(const std::string& s) {
    std::vector<uint8_t> v;
    for (size_t i = 0; i + 1 < s.size(); i += 2) v.push_back((uint8_t)std::stoi(s.substr(i, 2), nullptr, 16));
    return v;
}

void print_hex(const std::vector<uint8_t>& v) {
    for (uint8_t b : v) std::printf("%02x", b);
}

// six servos answering SYNC_READ with fixed positions, recording SYNC_WRITE goals (a stand-in for the bus)
struct SimServos : sts::Port {
    std::vector<uint8_t> tx;
    sts::Parser parser;
    std::map<int, int> goals;
    void write(const uint8_t* d, size_t n) override {
        for (auto& p : parser.feed(d, n)) {
            if (p.code == sts::INST_SYNC_READ) {
                for (size_t i = 2; i < p.params.size(); ++i) {
                    int s = p.params[i];
                    std::vector<uint8_t> body;
                    for (int v : {2048 + 100 * s, sts::to_sm(-50, 15), sts::to_sm(-200, 10)})
                        body.push_back((uint8_t)(v & 0xFF)), body.push_back((uint8_t)(v >> 8));
                    body.push_back(120), body.push_back(31);
                    auto st = sts::status_packet((uint8_t)s, 0, body);
                    tx.insert(tx.end(), st.begin(), st.end());
                }
            } else if (p.code == sts::INST_SYNC_WRITE) {
                for (size_t i = 2; i + 6 < p.params.size() + 1; i += 7) goals[p.params[i]] = p.params[i + 1] | p.params[i + 2] << 8;
            } else if (p.code == sts::INST_PING || p.code == sts::INST_WRITE) {
                auto st = sts::status_packet(p.id, 0);
                tx.insert(tx.end(), st.begin(), st.end());
            }
        }
    }
    size_t read(uint8_t* buf, size_t cap, int) override {
        size_t n = std::min(cap, tx.size());
        std::copy(tx.begin(), tx.begin() + (long)n, buf);
        tx.erase(tx.begin(), tx.begin() + (long)n);
        return n;
    }
};

struct ArmCheck {
    std::unique_ptr<ArmFk> fk;
    sts::Parser parser;
    sts::JointCal cal;
    SimServos servos;
    ArmPlay play;
};

void run_arm(ArmCheck& c, const std::string& op, std::istringstream& is) {
    if (op == "cfg") {
        std::string path = rest_of(is);
        c.fk = std::make_unique<ArmFk>(path);
    } else if (op == "fk" || op == "tool" || op == "toolout") {
        Joints q{};
        for (auto& v : q) is >> v;
        if (op == "toolout") {
            std::printf("%d\n", c.fk->tool_out(q) ? 1 : 0);
        } else if (op == "tool") {
            for (double v : c.fk->tool(q)) std::printf("%.17g ", v);
            std::printf("\n");
        } else {
            for (auto& [name, T] : c.fk->fk(q)) {
                std::printf("%s", name.c_str());
                for (double v : T) std::printf(" %.17g", v);
                std::printf("\n");
            }
        }
    } else if (op == "quat") {
        Mat4 T{};
        for (auto& v : T) is >> v;
        auto q = quat_of(T);
        nums({q[0], q[1], q[2], q[3]});
    } else if (op == "pkt" || op == "spkt") {
        int sid, code;
        std::string h;
        is >> sid >> code >> h;
        auto p = from_hex(h);
        print_hex(op == "pkt" ? sts::packet((uint8_t)sid, (uint8_t)code, p) : sts::status_packet((uint8_t)sid, (uint8_t)code, p));
        std::printf("\n");
    } else if (op == "feed") {
        std::string h;
        is >> h;
        auto d = from_hex(h);
        for (auto& p : c.parser.feed(d.data(), d.size())) {
            std::printf("%d %d %d ", p.id, p.len, p.code);
            print_hex(p.params);
            std::printf(";");
        }
        std::printf(" %d\n", c.parser.errors);
    } else if (op == "sm" || op == "fsm") {
        int v, bit;
        is >> v >> bit;
        std::printf("%d\n", op == "sm" ? sts::to_sm(v, bit) : sts::from_sm(v, bit));
    } else if (op == "cal") {
        is >> c.cal.id >> c.cal.drive_mode >> c.cal.range_min >> c.cal.range_max;
    } else if (op == "rad") {
        int t;
        is >> t;
        nums({c.cal.to_rad(t)});
    } else if (op == "ticks") {
        double r;
        is >> r;
        std::printf("%d\n", c.cal.to_ticks(r));
    } else if (op == "nominal") {
        int id;
        double lo, hi;
        is >> id >> lo >> hi;
        auto n = sts::nominal_calibration(id, lo, hi);
        std::printf("%d %d\n", n.range_min, n.range_max);
    } else if (op == "bus") {                           // bus ping|write|read|goals ...
        std::string what;
        is >> what;
        sts::Bus bus(c.servos, 0.0, [] { return 0.0; });
        if (what == "ping") {
            int id;
            is >> id;
            std::printf("%d\n", bus.ping(id) ? 1 : 0);
        } else if (what == "write") {
            int id, v;
            std::string r;
            is >> id >> r >> v;
            std::printf("%d\n", bus.write(id, r, v) ? 1 : 0);
        } else if (what == "read") {
            std::vector<int> ids;
            for (int i; is >> i;) ids.push_back(i);
            for (auto& [id, s] : bus.sync_read_status(ids))
                std::printf("%d %d %d %.17g %.17g %d %d;", id, s.position, s.speed, s.load, s.volts, s.temp_c, s.ok ? 1 : 0);
            std::printf("\n");
        } else {                                        // goals id:ticks ...
            std::vector<std::pair<int, int>> g;
            std::string tok;
            while (is >> tok) g.push_back({std::stoi(tok.substr(0, tok.find(':'))), std::stoi(tok.substr(tok.find(':') + 1))});
            bus.sync_write_goals(g);
            for (auto& [id, v] : c.servos.goals) std::printf("%d=%d ", id, v);
            std::printf("\n");
        }
    } else if (op == "play") {                          // play <0 quintic | 1 linear> <n>, then the n times, then n x 6 values
        int kind, n;
        is >> kind >> n;
        std::string l1, l2;
        std::getline(std::cin, l1);
        std::getline(std::cin, l2);
        std::istringstream a(l1), b(l2);
        std::vector<double> t(n);
        std::vector<Joints> q(n);
        for (auto& v : t) a >> v;
        for (auto& w : q)
            for (auto& v : w) b >> v;
        c.play = ArmPlay(kind ? ArmPlay::LINEAR : ArmPlay::QUINTIC, t, q);
    } else if (op == "sample") {
        double t;
        is >> t;
        auto q = c.play.sample(t);
        nums({q[0], q[1], q[2], q[3], q[4], q[5]});
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string core = argc > 1 ? argv[1] : "";
    ClockSync clock;
    HostLink link;
    ModeMux mux;
    HeadingShaper heading;
    ScanClock sclock;
    CatchCheck cc;
    ArmCheck arm;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream is(line);
        std::string op;
        if (!(is >> op)) continue;
        if (core == "clock") {
            if (op == "cfg") {
                ClockSyncConfig c;
                is >> c.echo_window_s >> c.pulse_window_s >> c.pulse_period_s >> c.edge_latency_ns >> c.refit_s;
                clock = ClockSync(c);
            } else if (op == "echo") {
                long long t1, t2, t3, t4;
                is >> t1 >> t2 >> t3 >> t4;
                clock.add(t1, t2, t3, t4);
            } else if (op == "pulse") {
                long long s, t;
                is >> s >> t;
                clock.add_mcu_pulse(s, t);
            } else if (op == "edge") {
                long long t;
                is >> t;
                clock.add_host_edge(t);
            } else if (op == "host") {
                long long t;
                is >> t;
                num(clock.to_host_s(t));
                std::printf("\n");
            } else if (op == "mcu") {
                double t;
                is >> t;
                std::printf("%lld\n", (long long)clock.to_mcu_us(t));
            } else if (op == "status") {
                std::printf("%s %lld %d %d %d %d %d\n", clock.mode().c_str(),
                            (long long)clock.rtt_ns.value_or(-1), clock.n_echo_used, clock.n_pulse_used,
                            clock.n_rejected, clock.n_resets, clock.n_paired);
            }
        } else if (core == "link") {
            if (op == "feed") {
                std::string hex;
                long long t;
                is >> hex >> t;
                std::vector<uint8_t> b;
                for (size_t i = 0; i + 1 < hex.size(); i += 2) b.push_back((uint8_t)std::stoi(hex.substr(i, 2), nullptr, 16));
                for (auto& m : link.feed(b.data(), b.size(), t)) {
                    std::printf("%c ", "SINT"[m.kind]);
                    num(m.t_host);
                    std::printf("\n");
                }
            } else if (op == "cmd") {
                long long t;
                int req_mode, kid, drive, hold, fire;
                Request r;
                is >> t >> req_mode >> kid >> drive >> hold >> r.a >> r.b >> r.c >> r.v_end >> r.a_cap >> r.speed_cap >>
                    r.t_brake >> r.coast_s >> r.a_brake >> r.tilt >> r.t_tilt >> r.tilt_after >> r.fly_rpm >> fire;
                r.req_mode = (uint8_t)req_mode, r.kid = kid, r.drive_mode = (uint8_t)drive, r.hold_yaw = hold;
                r.fire_seq = (uint16_t)fire;
                for (uint8_t x : link.encode(to_command(r, link.sync), t)) std::printf("%02x", x);
                std::printf("\n");
            } else if (op == "stats") {
                auto& s = link.stats;
                std::printf("%lld %lld %lld %lld %lld %lld %lld\n", (long long)s.frames, (long long)s.errors,
                            (long long)s.states, (long long)s.imus, (long long)s.infos, (long long)s.cmds,
                            (long long)s.times);
            }
        } else if (core == "mux") {
            if (op == "init") {
                std::string m;
                int k;
                is >> m >> k;
                mux = ModeMux(m, k);
            } else if (op == "set") {
                std::string m, msg;
                int k;
                is >> m >> k;
                bool ok = mux.set_mode(m, k < 0 ? std::nullopt : std::optional<bool>(k == 1), msg);
                std::printf("%d %s %d\n", (int)ok, mux.mode.c_str(), (int)mux.kid);
            } else if (op == "arm") {
                int a;
                is >> a;
                mux.arm_out = a;
            } else if (op == "fire") {
                int s;
                is >> s;
                std::printf("%d\n", mux.fire((uint16_t)s));
            } else if (op == "req") {
                std::string k;
                is >> k;
                Twist2 tw{};
                Request th;
                if (k == "t" || k == "p") is >> tw.vx >> tw.vy >> tw.wz;
                if (k == "h") {
                    int seq;
                    is >> th.tilt >> th.fly_rpm >> seq;
                    th.fire_seq = (uint16_t)seq;
                }
                print_request(mux.request(k == "t" ? &tw : nullptr, k == "h" ? &th : nullptr, k == "p" ? &tw : nullptr));
            } else if (op == "catch") {
                CatchCommand cm;
                int drive, fire;
                is >> drive >> cm.target[0] >> cm.target[1] >> cm.target[2] >> cm.v_end >> cm.t_brake >> cm.coast_s >>
                    cm.a_brake >> cm.tilt >> cm.t_tilt >> cm.tilt_after >> cm.fly_rpm >> fire;
                cm.drive_mode = (uint8_t)drive, cm.fire_seq = (uint16_t)fire;
                print_request(mux.request(cm));
            }
        } else if (core == "heading") {
            if (op == "cfg") {
                HeadingConfig c;
                is >> c.mode >> c.spin_rate >> c.turn_max >> c.lead >> c.period;
                heading = HeadingShaper(c);
            } else if (op == "odom") {
                double t, x, y, yaw, wz;
                is >> t >> x >> y >> yaw >> wz;
                heading.on_odom(t, x, y, yaw, wz);
            } else if (op == "plan") {
                double t, l;
                is >> t >> l;
                heading.on_plan(t, l);
            } else if (op == "cmd" || op == "step") {
                double t, vx = 0, vy = 0, wz = 0;
                is >> t;
                std::optional<Cmd3> c;
                if (op == "cmd") {
                    is >> vx >> vy >> wz;
                    c = heading.on_cmd(t, vx, vy, wz);
                } else {
                    c = heading.step(t);
                }
                if (!c) {
                    std::printf("none\n");
                } else {
                    num(c->vx), std::printf(" "), num(c->vy), std::printf(" "), num(c->wz), std::printf("\n");
                }
            }
        } else if (core == "scan") {
            if (op == "floor") {
                size_t n;
                LidarCal cal;
                double roll, pitch, margin;
                is >> n >> cal.tan_tilt >> cal.azimuth >> cal.height >> roll >> pitch >> margin;
                auto a = read_line_nums(), r = read_line_nums();
                for (bool b : floor_mask(a, r, cal, roll, pitch, margin)) std::printf("%d", (int)b);
                std::printf("\n");
            } else if (op == "deskew") {
                size_t n, n_out;
                int n_key;
                double lx, ly, t_ref, amin, inc, vx, vy, w, x0, y0, yaw0;
                is >> n >> lx >> ly >> t_ref >> amin >> inc >> n_key >> n_out >> vx >> vy >> w >> x0 >> y0 >> yaw0;
                auto a = read_line_nums(), r = read_line_nums(), ts = read_line_nums();
                auto pose = [&](double t) {
                    double yaw = std::fmod(yaw0 + w * t + kPi, 2 * kPi);
                    if (yaw < 0) yaw += 2 * kPi;
                    return Pose2{x0 + vx * t, y0 + vy * t, yaw - kPi};
                };
                for (double v : deskew(a, r, ts, pose, lx, ly, t_ref, amin, inc, n_out, n_key)) {
                    num(v);
                    std::printf(" ");
                }
                std::printf("\n");
            } else if (op == "sclock") {
                double lat;
                is >> lat;
                sclock = ScanClock(lat);
            } else if (op == "upd") {
                double s, st;
                is >> s >> st;
                num(sclock.update(s, st));
                std::printf(" ");
                num(sclock.T);
                std::printf("\n");
            }
        } else if (core == "arm") {
            run_arm(arm, op, is);
        } else if (core == "catch" || core == "percep" || core == "game" || core == "detect") {
            run_catch(cc, core, op, is);
        }
        std::fflush(stdout);
    }
    return 0;
}
