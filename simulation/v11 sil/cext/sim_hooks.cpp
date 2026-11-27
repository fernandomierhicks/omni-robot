// sim_hooks.cpp: the v11 sim's per-physics-step Python hooks in C++ (Nov 26), one call per step on MuJoCo's own
// mjModel / mjData (struct access through MuJoCo's headers; nothing linked). Twins of, and checked against
// (test_v11 test_cpp_hooks_match_python):
//   contact_model.ViscoContacts.pre_step  the squeeze slides' Maxwell arms; the ball contacts are only detected here
//                                          (the return value), their laws stay in Python (rare: a parked ball)
//   floor.FloorFriction.pre_step          the friction field (random Fourier modes) and the floor patches
//   floor.RollingResistance.pre_step      load-dependent drag (robot_model.wheel_loads); the surface's crr from Python
//   arm_so101.ArmServos.step              the STS3215s' slew and speed-dependent torque limit
// State lives in numpy arrays the Python objects own (sim_hooks.py builds these structs from them), so a study that
// changes a servo's goal or speed, or a floor's crr, is seen at the next step.
#include <cmath>
#include <cstdint>

#include "mujoco/mujoco.h"

#if defined(_WIN32)
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C" __attribute__((visibility("default")))
#endif

struct Contacts {           // ViscoContacts
    const uint8_t* is_ball; // [ngeom]
    int n_slides;
    const int* slide_q;     // qpos adr
    const int* slide_v;     // dof adr
    const double* slide_k;  // k_m
    double* x_d;            // [n_slides] the dashpots' ends (Python's array)
    double decay;
};

struct Floor {              // FloorFriction
    int n_pairs;
    const int* pairs;
    const int* geoms;
    double mu0;
    int n_modes;
    const double* k;        // [n_modes][2]
    const double* phi;
    double amp;
    int n_patches;
    const double* patches;  // [n_patches][5]: x0 y0 x1 y1 mu
};

struct Rolling {            // RollingResistance
    int n_wheels;
    const int* load_adr;    // wload sensor adr
    const int* load_site;   // carrier site
    const int* load_body;   // carrier body (subtree mass)
    const int* drive;       // drive dof per wheel
    const double* drive0;
    const double* crr;      // per wheel, set by Python every 5 ms
    double R, r, pin_mu, pin_r;
    const int* roller_start;   // [n_wheels + 1] offsets into roller_dofs / roller0
    const int* roller_dofs;
    const double* roller0;
    double* load_out;       // [n_wheels] the loads (kept for Python's wheel_loads users)
};

struct Arm {                // ArmServos
    int n;
    const int* act;
    const int* vadr;
    double* cmd;
    const double* target;
    const double* speed;
    const double* torque_scale;
    double stall, no_load;
};

struct Hooks {
    int abi;                // the struct layout: sim_hooks.py checks it against ABI
    Contacts* contacts;     // or null
    Floor* floor;
    Rolling* rolling;
    Arm* arm;
    int floor_div;
    int64_t tick;           // the floor hooks run when ++tick % floor_div == 0, as Robot.pre_step
};

static const int ABI = 1;

API int hooks_abi() { return ABI * 10000000 + mjVERSION_HEADER; }

static double clip(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

static int contacts_pre(const mjModel* m, mjData* d, const Contacts* c) {
    int ball = 0;
    for (int i = 0; i < d->ncon; ++i) {
        const int g0 = d->contact[i].geom[0], g1 = d->contact[i].geom[1];
        if ((g0 >= 0 && c->is_ball[g0]) || (g1 >= 0 && c->is_ball[g1])) {
            ball = 1;
            break;
        }
    }
    for (int j = 0; j < c->n_slides; ++j) {
        const double x = d->qpos[c->slide_q[j]];
        c->x_d[j] = x + (c->x_d[j] - x) * c->decay;
        d->qfrc_applied[c->slide_v[j]] = -c->slide_k[j] * (x - c->x_d[j]);
    }
    return ball;
}

static void floor_friction(mjModel* m, const mjData* d, const Floor* f) {
    for (int i = 0; i < f->n_pairs; ++i) {
        const double x = d->geom_xpos[3 * f->geoms[i]], y = d->geom_xpos[3 * f->geoms[i] + 1];
        double mu = f->mu0;
        if (f->n_modes) {
            double s = 0.0;
            for (int k = 0; k < f->n_modes; ++k)
                s += std::cos(x * f->k[2 * k] + y * f->k[2 * k + 1] + f->phi[k]);
            mu += f->amp * s;
        }
        for (int p = 0; p < f->n_patches; ++p) {
            const double* q = f->patches + 5 * p;
            if (x >= std::fmin(q[0], q[2]) && x <= std::fmax(q[0], q[2]) && y >= std::fmin(q[1], q[3]) &&
                y <= std::fmax(q[1], q[3]))
                mu = q[4];
        }
        mu = clip(mu, 0.2, 1.4);
        m->pair_friction[5 * f->pairs[i]] = mu;
        m->pair_friction[5 * f->pairs[i] + 1] = mu;
    }
}

static void rolling(mjModel* m, const mjData* d, const Rolling* r) {
    for (int k = 0; k < r->n_wheels; ++k) {
        const double* R = d->site_xmat + 9 * r->load_site[k];
        const double* s = d->sensordata + r->load_adr[k];
        const double fz = R[6] * s[0] + R[7] * s[1] + R[8] * s[2];     // (R @ f)[2]
        double load = m->body_subtreemass[r->load_body[k]] * 9.81 - fz;
        load = load > 0.0 ? load : 0.0;
        r->load_out[k] = load;
        const double f = r->crr[k] * load;
        m->dof_frictionloss[r->drive[k]] = r->drive0[k] + f * r->R;
        const double pin = r->pin_mu * r->pin_r * load;
        for (int j = r->roller_start[k]; j < r->roller_start[k + 1]; ++j)
            m->dof_frictionloss[r->roller_dofs[j]] = r->roller0[j] + f * r->r + pin;
    }
}

static void arm_step(mjModel* m, mjData* d, const Arm* a, double dt) {
    for (int j = 0; j < a->n; ++j) {
        const double step = a->speed[j] * dt;
        a->cmd[j] += clip(a->target[j] - a->cmd[j], -step, step);
        d->ctrl[a->act[j]] = a->cmd[j];
        const double w = d->qvel[a->vadr[j]];
        const double frac = clip(1.0 - std::fabs(w) / a->no_load, 0.0, 1.0);
        const double stall = a->stall * a->torque_scale[j];
        const double up = w > 0 ? stall * frac : stall;
        const double down = w < 0 ? stall * frac : stall;
        m->actuator_forcerange[2 * a->act[j]] = -down;
        m->actuator_forcerange[2 * a->act[j] + 1] = up;
    }
}

// Robot.pre_step's hooks in its order: contacts, then (every floor_div) the floor hooks; the arm at the end. Returns 1
// when a ball touches something (Python then applies its contact laws).
API int hooks_pre(mjModel* m, mjData* d, Hooks* h, double dt, int with_arm) {
    int ball = h->contacts ? contacts_pre(m, d, h->contacts) : 0;
    h->tick += 1;
    if (h->tick % h->floor_div == 0) {
        if (h->floor)
            floor_friction(m, d, h->floor);
        if (h->rolling)
            rolling(m, d, h->rolling);
    }
    if (with_arm && h->arm)
        arm_step(m, d, h->arm, dt);
    return ball;
}

API void hooks_arm(mjModel* m, mjData* d, Hooks* h, double dt) {
    if (h->arm)
        arm_step(m, d, h->arm, dt);
}
