// The lift gate (#297): the track player checks the body before it lets a foot leave the floor.
//
// Why. Three falls on 2026-11-12/13 (and the tips before them) lifted a foot with the body already 5-10 deg
// off level -- rocking from the previous step, or sagging after a foot slipped in double support. Every clean
// step lifted within +-2.5 deg of the roll it had at the settled double support. The stance trim zeroes its
// error at lift-off (its reference is the roll THEN), so nothing objected. The gate is the objection.
//
// What. The host marks the last double-support frame before each lift-off with gaitplay::kGate. When the
// track clock reaches such a frame the gate looks at the body once:
//   level  |roll - roll_ds[side]| <= lvl     roll_ds: the roll of the settled double support before the
//                                            lifts of that side (below), rad
//   quiet  |roll rate|            <= rate    the gyro about the roll axis, rad/s (what the trim's roll sees)
// Both hold: the clock goes on, no latency. Otherwise the clock is PARKED at the frame's time (every joint
// keeps its goal; the host's rolling upload goes on, the queue just does not drain) until both have held for
// `dwell_ms` in a row; then the clock goes on from there. A hold longer than `timeout_ms` is a timeout: the
// caller ends the track from this double support (Track::abortToRest) and reports it.
//
// roll_ds. NOT the roll at this instant (that is the hole the trim has). Per stance side, a slow reference
// of the roll the gate found acceptable the last time: seeded with the roll when the track began (the first
// tick, the centred crouch after the lead-in: within +-1.7 deg of the first double supports on 5 of 6
// records of 2026-11-12), the first acceptance of a side takes that side's own roll, later ones move it by
// `ref_alpha` of the difference. Only an accepted roll moves it, so a body that sags a little more each step
// cannot drag the reference after it by more than `lvl` per acceptance.
//
// Pure: no hardware, no clock, no globals. The caller owns the clock: it hands in the unheld wall-derived
// track time and the wall time, and plays the track at the time the gate returns.
#pragma once
#include <stdint.h>

namespace gaitplay {

struct LiftGateConfig {
    bool enabled = true;                 // `liftgate off`: the gate frames are ignored
    float lvl_rad = 0.05236f;            // 3 deg
    float rate_rad_s = 0.26180f;         // 15 deg/s
    uint32_t dwell_ms = 150;             // both conditions this long in a row to leave a hold
    uint32_t timeout_ms = 3000;          // a hold this long ends the track
    float ref_alpha = 0.25f;             // roll_ds moves this fraction of the way to an accepted roll
};

// nullptr when usable, else why not.
const char* validateGate(const LiftGateConfig& c);

enum class GateEvt : uint8_t {
    kNone = 0,
    kPass,         // a gate frame was reached and the body was level and quiet: no hold
    kHoldStart,    // reached, not level or not quiet: the clock is parked
    kHoldEnd,      // the hold ended (dwell met, or the gate was switched off): the clock goes on
    kTimeout,      // the hold ran out: the caller ends the track
};

struct GateIn {
    uint32_t wall_ms = 0;        // monotonic ms (the stream's elapsed time)
    uint32_t t_raw_ms = 0;       // the track time with no hold at all (start + wall - lead-in)
    bool have_gate = false;      // Track::gateAhead found a gate frame at or before the track time ...
    uint32_t t_gate_ms = 0;      // ... this one ...
    char side = 0;               // ... leading into the single support on this stance side
    float roll_rad = 0.0f;       // NaN = no usable attitude (counts as not level)
    float rate_rad_s = 0.0f;     // NaN = none (counts as not quiet)
};

struct GateOut {
    uint32_t t_ms = 0;           // the track time to sample at
    bool holding = false;        // parked: the caller feeds the trim double support (no chain may begin)
    GateEvt ev = GateEvt::kNone;
    uint32_t hold_ms = 0;        // kHoldEnd / kTimeout: how long the hold was
    float dev_rad = 0.0f;        // the gate's last look: roll - roll_ds of `side` ...
    float rate_rad_s = 0.0f;     // ... and the roll rate (set on every event)
    char side = 0;
};

struct GateStats {
    uint32_t gates = 0;          // gate frames evaluated
    uint32_t holds = 0;
    uint32_t held_ms = 0;        // total
    uint32_t timeouts = 0;
};

class LiftGate {
  public:
    void configure(const LiftGateConfig& c) { cfg_ = c; }
    const LiftGateConfig& config() const { return cfg_; }
    // A new track: references, hold and the counters cleared (the config stays).
    void begin();
    // One tick (call every tick of a playing track, gate frame or not: the first valid roll is the seed).
    GateOut tick(const GateIn& in);

    bool holding() const { return holding_; }
    // The track ended while parked (an operator stop / halt): forget the hold. The counters stay.
    void abandon() { holding_ = false; quiet_ = false; }
    // the track time a tick at raw time `t_raw_ms` would play at with the holds so far (to ask the queue for
    // the next gate frame before calling tick())
    uint32_t naturalMs(uint32_t t_raw_ms) const { return t_raw_ms > held_ms_ ? t_raw_ms - held_ms_ : 0; }
    const GateStats& stats() const { return st_; }
    // roll_ds of `side` now (rad); NaN before the first roll
    float ref(char side) const;
    // The hold before the last gate frame, ms (0 = none): what the step's TS line reports.
    uint32_t lastHoldMs() const { return last_hold_ms_; }
    // true once a gate frame has been evaluated since begin() (the TS lines carry G only then)
    bool gated() const { return st_.gates > 0; }

  private:
    bool good(float roll, float rate, char side, float* dev) const;
    void accept(char side, float roll);
    LiftGateConfig cfg_;
    GateStats st_;
    float start_ = 0.0f, ref_[2] = {0.0f, 0.0f};
    bool have_start_ = false, have_ref_[2] = {false, false};
    bool holding_ = false, quiet_ = false;
    uint32_t held_ms_ = 0;               // the clock's total delay so far (track time = raw - held)
    uint32_t hold_t0_ = 0, quiet_t0_ = 0, last_hold_ms_ = 0, last_gate_t_ = 0;
    uint32_t t_gate_ = 0;
    char side_ = 0;
    bool seen_gate_ = false;
};

}  // namespace gaitplay
