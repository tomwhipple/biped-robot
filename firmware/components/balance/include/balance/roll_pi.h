// Stance-leg roll balance: the pure math behind the bench `pose ... B<L|R>`
// mode (main/cli.cpp). No hardware, no clock, no globals -- the caller feeds
// it one roll sample per tick and reads the correction back, which is what
// lets host/ test every property below.
//
//   err = roll - roll_ref            roll_ref is captured when feedback engages
//                                    (the shifted pose, before the lift), so the
//                                    absolute IMU roll zero never matters
//   u   = clamp(kp*err + ki*I + kd*w) I += err*dt, frozen while the output is
//                                    saturated and err pushes further (anti-windup),
//                                    and |ki*I| <= i_lim_rad (the integral's share of u)
//                                    w = body roll rate from the gyro (rad/s, + =
//                                    roll increasing = d err/dt): derivative on the
//                                    measurement, never a differenced angle
//   stance ankle roll += ka * u      the caller adds these (radians, SIM joint
//   stance hip roll   += kh * u      frame) to the STANCE leg's two roll targets
//
// Identical for either stance: roll is the torso's lean about the body x axis
// (positive = toward the robot's right), the sim's joint frame mirrors left
// and right, and u is odd in err -- so the same ka/kh serve L and R and there
// are no per-side constants. experiments/one-leg-stand/balance.py is the sim
// twin of this file; the host test pins the C++ to numbers that file produced.
//
// ARMED (hw 2026-10-29/30, sim experiments/one-leg-stand/2026-10-30-s2-lift-sim.md):
// during the LIFT the torso tilts 2-4 deg toward the lifting foot with no CoM
// shift behind it (roll play loaded in double support, released at foot-off).
// A PI running through the lift answers it with 7-8 deg of outward push (mostly
// I) and the robot falls outward over the stance foot -- 3/3 on the bench's
// second step. So a fresh engage only ARMS: u stays 0 and nothing integrates,
// but the abort watches |roll - pre-lift roll|. start() at the end of the move
// takes the reference there and runs the PI from zero. ARMED is not blind: the
// lean is sometimes REAL (hw 10-30, the same s1 lift leaned 3 deg one run and
// tipped past 12 deg the next), so past arm_start_rad from the pre-lift roll
// the PI starts at once, its reference taken there -- a small false tilt never
// winds the integrator, a real fall is caught early.
//
// ABORT. Past |err| > abort_rad, or with no usable attitude/rate, the loop stops
// making decisions and runs a fixed, open-loop sequence (chosen in sim,
// experiments/one-leg-stand/2026-10-25-abort-policy-sim.md): the correction u
// fades to zero over abort_decay_s (min-jerk), then the caller streams the pose
// back to the one the B move started from (the double-support pre-lift pose) over
// abort_land_s, lowering the lifted foot. Torque is never released. The sequence
// needs no attitude, so the IMU-fault path runs the same code; the abort is
// idempotent (the first cause and its numbers stand).
#pragma once
#include <stdint.h>

namespace balance {

struct Config {
    // The defaults are the sim-chosen gains (experiments/one-leg-stand/README.md):
    // held from half to 1.5x all three together, falls appear at 2x; kd alone is
    // good to about 2x. Ankle +1 / hip -1 is a lateral pelvis translation with no
    // net torso tilt, i.e. it moves the CoM.
    float kp = 0.3f;          // rad per rad
    float ki = 1.0f;          // 1/s
    float kd = 0.03f;         // rad per rad/s: damping on the gyro roll rate, same sign as kp
    float ka = 1.0f;          // PI output -> stance ankle roll (signed mix)
    float kh = -1.0f;         // PI output -> stance hip roll (signed mix)
    // |u| limit, 10 deg. The roll play beyond the encoders absorbs most of the
    // command: on the robot (2026-10-24, one foot clamped) the loop settled at
    // u 9.2 deg (L stance) / 9.7 deg (R), and a 6 deg clamp saturated with
    // ~2 deg of error left. The sim sweep is unchanged between 6 and 10 deg.
    float clamp_rad = 0.17453f;
    float abort_rad = 0.20944f;   // |err| beyond this aborts, 12 deg
    float abort_decay_s = 0.3f;   // abort: u -> 0 over this ...
    float abort_land_s = 1.0f;    // ... then the pose returns to the pre-lift pose over this
    // |ki*I| limit: the integral's share of u, 3 deg (0 = no limit). The 10-30 sim fit
    // (experiments/one-leg-stand/2026-10-30-sim-fit.md) found the L stance's outer support
    // edge ~1 deg of u beyond its equilibrium: the I term wound to 6.5 deg and the robot fell
    // outward over the stance foot (2 of 3 bench runs). With the integral capped the loop holds
    // a small residual lean toward the lifted foot instead (sim: 144/144 stands across the fitted
    // band, both sides) and kp*err keeps its full authority against a real inward fall.
    float i_lim_rad = 0.05236f;
    float arm_start_rad = 0.06981f;   // ARMED: |err| past this starts the PI there, 4 deg (0 = only at
                                      // move end; at or past the abort the abort wins)
};

// nullptr when the config is usable, else why not. Bounds are wide enough for
// any gain the sweep found stable at 2x and tight enough to refuse a typo.
const char* validate(const Config& c);

enum class State : uint8_t {
    kOff = 0,
    kActive,
    kAbortTip,       // |err| passed abort_rad: correction fades, then the foot lands
    kAbortImu,       // the attitude/rate read failed or was not finite: the same sequence
    kArmed,          // engaged, u held at 0 and no integration; the abort is live (see ARMED)
};

// The open-loop abort sequence at `since_s` after the abort: the fraction of the
// correction still applied (1 -> 0 over abort_decay_s) and how far the pose has
// returned to the pre-lift pose (0 -> 1 over abort_land_s, starting once the
// correction is gone). Both min-jerk. Pure; the sim twin prints golden values.
void abortProfile(const Config& c, float since_s, float* u_scale, float* land_frac);

// One joint's landing target in servo ticks: `from` (its nominal target when the
// abort fired) moves to `start` (where the B stream began) as land_frac goes 0 -> 1;
// exactly `start` at 1. Monotone in between.
int32_t landTarget(int32_t from, int32_t start, float land_frac);

// Whether a new `pose ... B<side>` CONTINUES the running balance instead of
// re-engaging it: the controller is ACTIVE or ARMED (not off, not aborting) on the same
// stance side, and every joint the new pose moves was in the stream already.
// Continuing keeps the integrator, u and the reference roll, starts the new move
// from the previous NOMINAL targets (u keeps riding on top), and keeps the landing
// target at the pose where the B chain began. A restart instead drops u to 0 and
// re-bases the move on the read-back positions -- on a loaded stance leg that
// lets go of both the I term and the servo's sag (hw 2026-10-29: u +8 deg on L
// stance, reset at the swing pose, tipped -12 deg 1.6 s later).
bool continues(State s, char active_side, char new_side, const bool* was_on,
               const bool* now_on, int n);

// Torso roll (rad, + = leans toward the robot's right) from imu::Sample::up,
// the torso z axis in the world frame: roll = atan2(-up_y, up_z). false when
// the vector is unusable (not finite, or tilted past ~60 deg).
bool rollFromUp(const float up[3], float* roll_rad);

class RollBalance {
  public:
    void configure(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    // Start regulating around `roll_ref_rad`. Clears the integrator and u.
    void engage(float roll_ref_rad);
    // Engage ARMED: u 0, no integration, abort against `roll_ref_rad`.
    void arm(float roll_ref_rad);
    // ARMED -> ACTIVE with the reference taken at `roll_ref_rad` (the roll now),
    // integrator and u from zero. false (and no change) when not armed.
    bool start(float roll_ref_rad);
    // Stop; the correction goes to zero (the next move starts from where the
    // servos are, so it does not jump).
    void disengage();
    // One tick with a valid roll sample and the body roll rate (gyro about the
    // torso x axis, rad/s, + = roll increasing). A non-finite rate or roll, or
    // |err| past abort_rad, aborts. Returns the state after the tick. Once
    // aborted the measurements are ignored and the call only advances the abort
    // sequence by dt_s (see abortTick). rate 0 = the PI alone.
    State update(float roll_rad, float dt_s, float roll_rate_rad_s = 0.0f);
    // The attitude could not be read: abort (cause IMU). No-op once aborted.
    void imuFault();
    // Force an abort for `why` (kAbortTip / kAbortImu). Idempotent: false and
    // nothing changes when not active (already aborted, or off).
    bool abort(State why);
    // Advance the abort sequence by dt_s with no measurement (the IMU is gone).
    void abortTick(float dt_s);

    State state() const { return state_; }
    bool engaged() const { return state_ != State::kOff; }
    bool armed() const { return state_ == State::kArmed; }
    bool aborted() const { return state_ == State::kAbortTip || state_ == State::kAbortImu; }
    float abortSince() const { return since_; }           // s since the abort
    float abortU() const { return u_abort_; }             // u when it fired, rad
    float abortErr() const { return err_abort_; }         // err when it fired, rad
    float landFraction() const;                           // 0 until the correction is gone, 1 = landed
    bool landed() const { return aborted() && landFraction() >= 1.0f; }
    float ref() const { return ref_; }
    float err() const { return err_; }
    float integral() const { return i_; }
    float u() const { return u_; }
    float rate() const { return rate_; }
    float ankle() const { return cfg_.ka * u_; }    // rad, add to the stance ankle roll
    float hip() const { return cfg_.kh * u_; }      // rad, add to the stance hip roll

  private:
    Config cfg_;
    State state_ = State::kOff;
    float ref_ = 0.0f, err_ = 0.0f, i_ = 0.0f, u_ = 0.0f, rate_ = 0.0f;
    float since_ = 0.0f, u_abort_ = 0.0f, err_abort_ = 0.0f;
};

const char* stateName(State s);
// "fading" / "landing" / "landed" for an aborted loop, "" otherwise
const char* abortPhase(const RollBalance& b);

}  // namespace balance
