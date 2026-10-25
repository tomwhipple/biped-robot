// Stance-leg roll balance: the pure math behind the bench `pose ... B<L|R>`
// mode (main/cli.cpp). No hardware, no clock, no globals -- the caller feeds
// it one roll sample per tick and reads the correction back, which is what
// lets host/ test every property below.
//
//   err = roll - roll_ref            roll_ref is captured when feedback engages
//                                    (the shifted pose, before the lift), so the
//                                    absolute IMU roll zero never matters
//   u   = clamp(kp*err + ki*I + kd*w) I += err*dt, frozen while the output is
//                                    saturated and err pushes further (anti-windup)
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
};

// nullptr when the config is usable, else why not. Bounds are wide enough for
// any gain the sweep found stable at 2x and tight enough to refuse a typo.
const char* validate(const Config& c);

enum class State : uint8_t {
    kOff = 0,
    kActive,
    kAbortTip,       // |err| passed abort_rad: correction fades, then the foot lands
    kAbortImu,       // the attitude/rate read failed or was not finite: the same sequence
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
