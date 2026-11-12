// The gait track and the stance trim behind it, for the software-in-the-loop sim (#297 B2): the firmware's
// own components (gaitplay/track, balance/stance_trim) behind a small C boundary, so the Python twin plays a
// sway gait through exactly the code the robot runs (experiments/roll-balance/sil_gait.py).
#include <math.h>
#include <string.h>

#include "balance/stance_trim.h"
#include "gaitplay/track.h"

namespace {

struct Gait {
    gaitplay::Track track;
    balance::StanceTrim trim;
    balance::TrimDriver driver{trim};
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

int sil_gait_start(void* h) { return static_cast<Gait*>(h)->track.start() ? 0 : -1; }
void sil_gait_stop(void* h) { static_cast<Gait*>(h)->track.stop(); }
uint32_t sil_gait_start_time(void* h) { return static_cast<Gait*>(h)->track.startTime(); }
int sil_gait_queued(void* h) { return static_cast<Gait*>(h)->track.queued(); }

// One 50 Hz tick at track time t_ms with the torso roll (rad; NaN = none). Out: the 12 goals (ticks,
// kLegJoints order) with the stance hip roll correction NOT applied (hip_joint_rad and stance say where it
// goes), the phase, the track state; report[7] = stance, lean_in, lean_out, e_mid, t_mid, hike_mid, aborted
// (rad), filled when a single support ended (return bit 1). Bit 0 of the return: the track is done.
int sil_gait_tick(void* h, uint32_t t_ms, float roll_rad, float dt_s, int32_t* q12, float* phi,
                  float* hip_joint_rad, int* stance, int* trim_state, float* report7) {
    Gait* g = static_cast<Gait*>(h);
    gaitplay::Sample s;
    const gaitplay::State st = g->track.sample(t_ms, &s);
    memcpy(q12, s.q, sizeof s.q);
    *phi = s.phi;
    const gaitplay::Support sup = gaitplay::supportFromPhase(s.phi, s.lift);
    balance::SupportIn in;
    in.stance = sup.stance;
    in.progress = sup.progress;
    in.lowering = sup.lowering;
    balance::StepReport rep;
    const bool ended = g->driver.step(in, roll_rad, dt_s, &rep);
    *hip_joint_rad = g->trim.engaged() ? g->trim.hipJoint() : 0.0f;
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
    }
    return (st == gaitplay::State::kDone ? 1 : 0) | (ended ? 2 : 0);
}

}  // extern "C"
