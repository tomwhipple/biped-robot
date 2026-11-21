// The slew rates of the deployed run, from the generated command envelope (tools/gen_envelope.py
// writes LINKPROTO_CMD_SLEW and kSlewVx/Vy/Wz into linkproto/cmd_envelope.h only when the run's
// config.json has cmd_slew). Without them every rate is 0: the limiter is off and the command path
// is exactly the unslewed one. The ONE place that selects.
#pragma once

#include "cmdslew/cmdslew.h"
#include "linkproto/cmd_envelope.h"

namespace cmdslew {

#ifdef LINKPROTO_CMD_SLEW
inline constexpr Rates kDeployedRates = {linkproto::kSlewVx, linkproto::kSlewVy, linkproto::kSlewWz};
#else
inline constexpr Rates kDeployedRates = {0.0f, 0.0f, 0.0f};
#endif

}  // namespace cmdslew
