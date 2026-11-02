// `nettime [n]` -- time the policy forward pass on the robot, every way this
// image can run it (docs/firmware-design.md section 6.1).
//
// The control loop's `stat` shows one number (`net`, the forward pass as
// armed). This answers the questions behind it in seconds, with no reflash:
// what the same net costs from RAM instead of flash, with the C or the MAC16
// int8 kernel, and per layer. The passes run on core 1 (the control core,
// below the control task's priority) with the loop benched; `run` is refused
// meanwhile because the housekeeping task is the one waiting. Each variant
// prints min / median / max microseconds over n passes (default 20): the
// minimum is the kernel, the maximum includes control-task preemption.
#pragma once

#include "cli.h"

namespace robot {

void netTime(cli::Sink out, int passes);

}  // namespace robot
