// Stance-leg roll balance: the pure math behind the bench `pose ... B<L|R>`
// mode (main/cli.cpp). No hardware, no clock, no globals -- the caller feeds
// it one roll sample per tick and reads the correction back, which is what
// lets host/ test every property below.
//
//   err = roll - roll_ref            roll_ref is captured when feedback engages
//                                    (the shifted pose, before the lift), so the
//                                    absolute IMU roll zero never matters
//   u   = clamp(kp*err + ki*I)       I += err*dt, frozen while the output is
//                                    saturated and err pushes further (anti-windup)
//   stance ankle roll += ka * u      the caller adds these (radians, SIM joint
//   stance hip roll   += kh * u      frame) to the STANCE leg's two roll targets
//
// Identical for either stance: roll is the torso's lean about the body x axis
// (positive = toward the robot's right), the sim's joint frame mirrors left
// and right, and u is odd in err -- so the same ka/kh serve L and R and there
// are no per-side constants. experiments/one-leg-stand/balance.py is the sim
// twin of this file; the host test pins the C++ to numbers that file produced.
#pragma once
#include <stdint.h>

namespace balance {

struct Config {
    // The defaults are the sim-chosen gains (experiments/one-leg-stand/README.md):
    // stable at half and at double, unstable by about 2.4x on kp. Ankle +1 / hip -1
    // is a lateral pelvis translation with no net torso tilt, i.e. it moves the CoM.
    float kp = 0.25f;         // rad per rad
    float ki = 0.5f;          // 1/s
    float ka = 1.0f;          // PI output -> stance ankle roll (signed mix)
    float kh = -1.0f;         // PI output -> stance hip roll (signed mix)
    float clamp_rad = 0.10472f;   // |u| limit, 6 deg
    float abort_rad = 0.20944f;   // |err| beyond this freezes the correction, 12 deg
};

// nullptr when the config is usable, else why not. Bounds are wide enough for
// any gain the sweep found stable at 2x and tight enough to refuse a typo.
const char* validate(const Config& c);

enum class State : uint8_t {
    kOff = 0,
    kActive,
    kFrozenAbort,    // |err| passed abort_rad: correction held at its last value
    kFrozenImu,      // the attitude read failed: correction held at its last value
};

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
    // One tick with a valid roll sample. Returns the state after the tick.
    // While frozen nothing changes.
    State update(float roll_rad, float dt_s);
    // The attitude could not be read: freeze at the last correction.
    void imuFault();

    State state() const { return state_; }
    bool engaged() const { return state_ != State::kOff; }
    float ref() const { return ref_; }
    float err() const { return err_; }
    float integral() const { return i_; }
    float u() const { return u_; }
    float ankle() const { return cfg_.ka * u_; }    // rad, add to the stance ankle roll
    float hip() const { return cfg_.kh * u_; }      // rad, add to the stance hip roll

  private:
    Config cfg_;
    State state_ = State::kOff;
    float ref_ = 0.0f, err_ = 0.0f, i_ = 0.0f, u_ = 0.0f;
};

const char* stateName(State s);

}  // namespace balance
