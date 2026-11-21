#include "cmdslew/cmdslew.h"

namespace cmdslew {

void CommandSlew::configure(Rates r, float dt) {
    const float rate[3] = {r.vx, r.vy, r.wz};
    for (int i = 0; i < 3; ++i) {
        // float32 x float32, as sim/cmd_kernel.py slew_delta.
        const float d = rate[i] * dt;
        delta_[i] = rate[i] > 0.0f ? d : 0.0f;
    }
}

void CommandSlew::reset(float vx, float vy, float wz) {
    cur_[0] = vx;
    cur_[1] = vy;
    cur_[2] = wz;
}

void CommandSlew::step(float tvx, float tvy, float twz) {
    const float tgt[3] = {tvx, tvy, twz};
    for (int i = 0; i < 3; ++i) {
        if (!(delta_[i] > 0.0f)) {
            cur_[i] = tgt[i];
            continue;
        }
        const float d = tgt[i] - cur_[i];
        const float ad = d < 0.0f ? -d : d;
        if (ad <= delta_[i]) {
            cur_[i] = tgt[i];
        } else {
            cur_[i] = cur_[i] + (d > 0.0f ? delta_[i] : -delta_[i]);
        }
    }
}

}  // namespace cmdslew
