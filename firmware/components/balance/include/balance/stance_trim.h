// Stance-hip roll trim through the lift (#297 step 2): the pure math behind the bench
// `pose ... S<L|R>` mode (main/cli.cpp). No hardware, no clock, no globals.
//
// The design and its evidence: experiments/roll-balance/README.md (Stage A). The host stepper keeps the
// pelvis level by trimming the STANCE hip roll against the measured tilt; this is that trim on the 50 Hz
// tick, with the static part learned per stance side and kept from step to step:
//
//   e    = roll - roll_ref           roll_ref = the torso roll when the chain began (the raise), rad
//   s    = +1 on L stance, -1 on R   (the robot's left / right)
//   hike = clip(r * T[side] + kp * s * e, +-lim)          rad, the stepper's `hike` convention:
//                                    + lifts the OTHER hip (rolls the pelvis toward the stance side)
//   T[side] += ki * s * e * dt       while CLOSED and |hike| < lim (anti-windup), |T| <= t_lim
//   stance hip roll joint += s * hike                     (the sim / firmware joint frame)
//
// r ramps the learned trim in: the caller passes the raise move's own min-jerk progress (0 -> 1) on the
// chain's first move and 1 after it. T stays per side across chains: a fast gait needs no fast integrator.
//
// States. CLOSED: the law above. HOLD: the correction is frozen (the lowering; the touch-down must not
// be answered). FADE: the correction goes linearly to 0 over fade_s, then OFF (the foot is down and the
// next shift owns the pelvis). ABORT (|e| > abort_rad while closed, or no usable attitude): the
// correction is frozen and fades exactly as FADE does -- nothing returns to a pre-lift pose and no other
// joint is touched: in a walking sequence the upper layer owns the foot. T learns only while CLOSED.
//
// Why not the old `B<L|R>` PI (roll_pi.h): its default action is a pelvis translation (ankle +u, hip -u)
// answering a tilt with a CoM shift; at foot-off the released roll play tilts the torso 2-4 deg with no
// CoM behind it, the integral turns that into an outboard push, and the robot falls outward over the
// stance foot (bench 3/3; the Stage A testbed 94 of 96).
#pragma once
#include <stdint.h>

namespace balance {

struct TrimConfig {
    float kp = 0.25f;           // rad of hike per rad of roll error (Stage A: kp >= ~1 limit-cycles)
    float ki = 1.0f;            // 1/s, the learned trim's rate
    float lim_rad = 0.26180f;   // |hike| limit, 15 deg
    float t_lim_rad = 0.24435f; // |T| limit, 14 deg
    float fade_s = 0.6f;        // HOLD/ABORT -> 0 over this
    float abort_rad = 0.26180f; // |e| past this (while CLOSED) aborts, 15 deg
};

// nullptr when usable, else why not.
const char* validateTrim(const TrimConfig& c);

// The codes share the flight record's balance state byte with roll_pi's balance::State (0-4).
enum class TrimState : uint8_t {
    kOff = 0,
    kClosed = 5,
    kHold = 6,
    kFade = 7,
    kAbortTip = 8,     // |e| passed abort_rad: frozen, fading
    kAbortImu = 9,     // no usable attitude: frozen, fading
};

class StanceTrim {
  public:
    void configure(const TrimConfig& c) { cfg_ = c; }
    const TrimConfig& config() const { return cfg_; }

    // Begin a chain on `side` ('L' / 'R') with the roll now as the reference: CLOSED, hike 0. A chain
    // already running on the other side is replaced (its correction is dropped: the caller starts the new
    // move from its nominal targets plus nothing).
    void begin(char side, float roll_ref_rad);
    // One CLOSED / HOLD / FADE / ABORT tick. `ramp` 0..1 is the first move's progress (1 after it).
    // A non-finite roll while CLOSED aborts (IMU). Returns the state after the tick.
    TrimState update(float roll_rad, float dt_s, float ramp);
    // CLOSED -> HOLD (freeze). No-op otherwise.
    void hold();
    // HOLD -> CLOSED (the same chain resumes: reference and T kept). No-op otherwise.
    void resume();
    // CLOSED / HOLD -> FADE. No-op when off, fading or aborted.
    void release();
    // The attitude could not be read: abort (IMU) when CLOSED, else nothing.
    void imuFault();
    // Off at once, correction 0 (an explicit stop: move, home, torque, FALL, arming).
    void stop();

    TrimState state() const { return state_; }
    bool engaged() const { return state_ != TrimState::kOff; }
    bool aborted() const { return state_ == TrimState::kAbortTip || state_ == TrimState::kAbortImu; }
    char side() const { return side_; }
    float ref() const { return ref_; }
    float err() const { return err_; }
    float hike() const { return hike_; }                  // rad, stepper convention
    float hipJoint() const { return sign() * hike_; }     // rad, add to the stance hip roll target
    float abortErr() const { return err_abort_; }
    float fadeLeft() const;                               // 1 -> 0 while fading / aborted, 1 otherwise

    // The learned trims, rad, stepper convention (+ lifts the other hip).
    float learned(char side) const { return side == 'L' ? t_[0] : t_[1]; }
    // false for a bad side or a non-finite value; clipped to +-t_lim.
    bool setLearned(char side, float t_rad);
    void clearLearned() { t_[0] = t_[1] = 0.0f; }

  private:
    float sign() const { return side_ == 'R' ? -1.0f : 1.0f; }
    TrimConfig cfg_;
    TrimState state_ = TrimState::kOff;
    char side_ = 0;
    float t_[2] = {0.0f, 0.0f};
    float ref_ = 0.0f, err_ = 0.0f, hike_ = 0.0f;
    float hike_frozen_ = 0.0f, since_ = 0.0f, err_abort_ = 0.0f;
};

const char* trimStateName(TrimState s);

}  // namespace balance
