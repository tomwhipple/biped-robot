// The gait track and the stance trim behind it, for the software-in-the-loop sim (#297 B2): the firmware's
// own components (gaitplay/track, balance/stance_trim) behind a small C boundary, so the Python twin plays a
// sway gait through exactly the code the robot runs (experiments/roll-balance/sil_gait.py).
#include <math.h>
#include <string.h>

#include "balance/stance_trim.h"
#include "gaitplay/lift_gate.h"
#include "gaitplay/track.h"

namespace {

struct Gait {
    gaitplay::Track track;
    balance::StanceTrim trim;
    balance::TrimDriver driver{trim};
    gaitplay::LiftGate gate;
    uint32_t prev_t = 0;
};

}  // namespace

extern "C" {

void* sil_gait_new(float kp, float ki, float lim_deg, float tlim_deg, float fade_s, float abort_deg) {
    Gait* g = new Gait();
    balance::TrimConfig c;
    c.kp = kp;
    c.ki = ki;
    c.lim_rad = lim_deg / 57.29578f;
    c.t_lim_rad = tlim_deg / 57.29578f;
    c.fade_s = fade_s;
    c.abort_rad = abort_deg / 57.29578f;
    if (balance::validateTrim(c) != nullptr) {
        delete g;
        return nullptr;
    }
    g->trim.configure(c);
    return g;
}

void sil_gait_free(void* h) { delete static_cast<Gait*>(h); }

// T in deg, hike convention (as `trim T`)
int sil_gait_set_learned(void* h, char side, float deg) {
    return static_cast<Gait*>(h)->trim.setLearned(side, deg / 57.29578f) ? 0 : -1;
}

float sil_gait_learned(void* h, char side) { return static_cast<Gait*>(h)->trim.learned(side) * 57.29578f; }

// One frame in the CLI's hex form (`track add`); 0 = queued
int sil_gait_add(void* h, const char* hex) {
    Gait* g = static_cast<Gait*>(h);
    gaitplay::Frame f;
    if (!gaitplay::parseFrame(hex, g->prev_t, &f)) return -1;
    if (!g->track.add(f)) return -2;
    if (!(f.flags & gaitplay::kRest)) g->prev_t = f.t_ms;
    return 0;
}

int sil_gait_start(void* h) {
    Gait* g = static_cast<Gait*>(h);
    g->gate.begin();
    return g->track.start() ? 0 : -1;
}

// The lift gate's thresholds (deg, deg/s, ms) and on/off; 0 = set, -1 = refused (bad values)
int sil_gait_gate_cfg(void* h, float lvl_deg, float rate_deg_s, int dwell_ms, int timeout_ms, int enabled) {
    Gait* g = static_cast<Gait*>(h);
    gaitplay::LiftGateConfig c = g->gate.config();
    c.lvl_rad = lvl_deg / 57.29578f;
    c.rate_rad_s = rate_deg_s / 57.29578f;
    c.dwell_ms = static_cast<uint32_t>(dwell_ms);
    c.timeout_ms = static_cast<uint32_t>(timeout_ms);
    c.enabled = enabled != 0;
    if (gaitplay::validateGate(c) != nullptr) return -1;
    g->gate.configure(c);
    return 0;
}

// out4: gates evaluated, holds, total held ms, timeouts
void sil_gait_gate_stats(void* h, uint32_t* out4) {
    const gaitplay::GateStats& s = static_cast<Gait*>(h)->gate.stats();
    out4[0] = s.gates;
    out4[1] = s.holds;
    out4[2] = s.held_ms;
    out4[3] = s.timeouts;
}
void sil_gait_stop(void* h) { static_cast<Gait*>(h)->track.stop(); }
uint32_t sil_gait_start_time(void* h) { return static_cast<Gait*>(h)->track.startTime(); }
int sil_gait_queued(void* h) { return static_cast<Gait*>(h)->track.queued(); }

// One 50 Hz tick at raw track time t_ms with the torso roll (rad; NaN = none) and its rate (rad/s). Out: the 12 goals (ticks,
// kLegJoints order) with the hip roll corrections NOT applied (hip_joint_rad[2] = L, R: the trim's and the
// driver residual's, joint rad), the phase, the trim state; report[8] = stance, lean_in, lean_out, e_mid,
// t_mid, hike_mid (rad), aborted, reverted, filled when a single support ended (return bit 1). Bit 0 of
// the return: the track is done. gate4 = the track time played at, holding, the gate event (GateEvt), the
// hold's length (ms) on kHoldEnd / kTimeout.
int sil_gait_tick(void* h, uint32_t t_ms, float roll_rad, float rate_rad_s, float dt_s, int32_t* q12, float* phi,
                  float* hip_joint_rad, int* stance, int* trim_state, float* report7, float* gate4) {
    Gait* g = static_cast<Gait*>(h);
    // the lift gate owns the clock: t_ms is the raw track time (no hold), the gate returns the one to play at
    gaitplay::GateIn gi;
    gi.wall_ms = t_ms;
    gi.t_raw_ms = t_ms;
    gi.roll_rad = roll_rad;
    gi.rate_rad_s = rate_rad_s;
    gi.have_gate = g->track.gateAhead(g->gate.naturalMs(t_ms), &gi.t_gate_ms, &gi.side);
    const gaitplay::GateOut go = g->gate.tick(gi);
    gaitplay::Sample s;
    if (go.ev == gaitplay::GateEvt::kTimeout) g->track.abortToRest(go.t_ms);     // both feet down: rest
    const gaitplay::State st = g->track.sample(go.t_ms, &s);
    if (gate4 != nullptr) {
        gate4[0] = static_cast<float>(go.t_ms);
        gate4[1] = go.holding ? 1.0f : 0.0f;
        gate4[2] = static_cast<float>(static_cast<int>(go.ev));
        gate4[3] = static_cast<float>(go.hold_ms);
    }
    memcpy(q12, s.q, sizeof s.q);
    *phi = s.phi;
    const gaitplay::Support sup = gaitplay::supportFromPhase(s.phi, s.lift && !go.holding);
    balance::SupportIn in;
    in.stance = sup.stance;
    in.progress = sup.progress;
    in.lowering = sup.lowering;
    balance::StepReport rep;
    const bool ended = g->driver.step(in, roll_rad, dt_s, &rep);
    // hip_joint_rad[2]: the L and R hip roll corrections, the trim's and the residual's
    hip_joint_rad[0] = hip_joint_rad[1] = 0.0f;
    if (g->trim.engaged()) hip_joint_rad[g->trim.side() == 'L' ? 0 : 1] += g->trim.hipJoint();
    if (g->driver.residualSide() != 0) hip_joint_rad[g->driver.residualSide() == 'L' ? 0 : 1] += g->driver.residualHip();
    *stance = g->trim.engaged() ? g->trim.side() : 0;
    *trim_state = static_cast<int>(g->trim.state());
    if (ended && report7 != nullptr) {
        report7[0] = static_cast<float>(rep.stance);
        report7[1] = rep.lean_in;
        report7[2] = rep.lean_out;
        report7[3] = rep.e_mid;
        report7[4] = rep.t_mid;
        report7[5] = rep.hike_mid;
        report7[6] = rep.aborted ? 1.0f : 0.0f;
        report7[7] = rep.reverted ? 1.0f : 0.0f;
    }
    return (st == gaitplay::State::kDone ? 1 : 0) | (ended ? 2 : 0);
}

}  // extern "C"

// The trim on a gait clock alone (the policy path, #297 B2): gaitplay::supportFromPhase + TrimDriver +
// StanceTrim, one tick per call -- what sim/trim_kernel.py is pinned to (tests/test_trim_kernel.py).
namespace {
struct ClockTrim {
    balance::StanceTrim trim;
    balance::TrimDriver driver{trim};
};
}  // namespace

extern "C" {

void* sil_trim_new(float kp, float ki, float lim_deg, float tlim_deg, float fade_s, float abort_deg) {
    ClockTrim* c = new ClockTrim();
    balance::TrimConfig cfg;
    cfg.kp = kp;
    cfg.ki = ki;
    cfg.lim_rad = lim_deg / 57.29578f;
    cfg.t_lim_rad = tlim_deg / 57.29578f;
    cfg.fade_s = fade_s;
    cfg.abort_rad = abort_deg / 57.29578f;
    if (balance::validateTrim(cfg) != nullptr) {
        delete c;
        return nullptr;
    }
    c->trim.configure(cfg);
    return c;
}

void sil_trim_free(void* h) { delete static_cast<ClockTrim*>(h); }

// T in rad (hike convention)
void sil_trim_set_learned(void* h, char side, float rad) { static_cast<ClockTrim*>(h)->trim.setLearned(side, rad); }

// out[6] = hip_L, hip_R (joint rad), hike, T_L, T_R, state
void sil_trim_step(void* h, float phase, int lift, float roll, float dt, float* out6) {
    ClockTrim* c = static_cast<ClockTrim*>(h);
    const gaitplay::Support sup = gaitplay::supportFromPhase(phase, lift != 0);
    balance::SupportIn in;
    in.stance = sup.stance;
    in.progress = sup.progress;
    in.lowering = sup.lowering;
    balance::StepReport rep;
    c->driver.step(in, roll, dt, &rep);
    const bool on = c->trim.engaged();
    const float hip = on ? c->trim.hipJoint() : 0.0f;
    out6[0] = on && c->trim.side() == 'L' ? hip : 0.0f;
    out6[1] = on && c->trim.side() == 'R' ? hip : 0.0f;
    if (c->driver.residualSide() != 0) out6[c->driver.residualSide() == 'L' ? 0 : 1] += c->driver.residualHip();
    out6[2] = c->trim.hike();
    out6[3] = c->trim.learned('L');
    out6[4] = c->trim.learned('R');
    out6[5] = static_cast<float>(c->trim.state());
}

}  // extern "C"
