// The operator command's slew limit (issue #266): a per-channel rate limit on the planar command
// (vx, vy, wz) ahead of the policy's observation. Each control tick a channel moves toward its
// target by at most rate x dt, and lands on the target exactly once it is within that step.
//
// The law is sim/cmd_kernel.py's slew_step, in float32 with the step computed as float32 rate times
// float32 dt; firmware/host/test_cmdslew.cpp pins it bit for bit to cmd_kernel.golden(). A run
// trained with `cmd_slew` never saw a command step, so the deployed firmware ramps the same way.
//
// A rate of 0 turns that channel off (it follows its target at once); all three 0 is the whole
// limiter off, today's behaviour. Pure: no IDF dependency.
#pragma once

namespace cmdslew {

struct Rates {
    float vx = 0.0f;   // m/s^2
    float vy = 0.0f;   // m/s^2
    float wz = 0.0f;   // rad/s^2
};

class CommandSlew {
public:
    // Rates and the control tick (s). Keeps the current state.
    void configure(Rates r, float dt);
    // Jump to (vx, vy, wz): the stand on a (re)arm.
    void reset(float vx = 0.0f, float vy = 0.0f, float wz = 0.0f);
    // One control tick toward the target; vx()/vy()/wz() read the result.
    void step(float tvx, float tvy, float twz);

    float vx() const { return cur_[0]; }
    float vy() const { return cur_[1]; }
    float wz() const { return cur_[2]; }
    // True when any channel has a rate.
    bool enabled() const { return delta_[0] > 0.0f || delta_[1] > 0.0f || delta_[2] > 0.0f; }

private:
    float cur_[3] = {0.0f, 0.0f, 0.0f};
    float delta_[3] = {0.0f, 0.0f, 0.0f};   // rate x dt per tick; 0 = pass through
};

}  // namespace cmdslew
