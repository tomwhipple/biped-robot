// The as-built zero calibration, compiled in as a REFERENCE (not a fallback).
//
// Source of truth: docs/servo-map.md "As-built zero calibration" -- measured
// 2026-08-02 on the assembled robot held at the standing pose, re-measured
// after the whole-chain leg-swap errata, all ten direction signs verified by
// driving each joint alone. Standing IS angle zero (kJointDefault is all
// zeros), so these ticks are each joint's zero_steps.
//
// What this table is FOR: automatic v1->v2 calibration migration. A v1 NVS
// blob cannot prove which servo map it was measured under, so it cannot be
// trusted blind -- but it CAN be checked against this table, which was
// measured under the current map. Exact match -> the blob is the known
// calibration and migrates without an operator eyeballing anything; any
// mismatch -> refused, recalibrate. The table is NOT used as a default
// calibration: NVS stays the runtime source of truth, and a robot with no
// valid blob boots uncalibrated with `run` refused.
//
// If the robot is ever re-zeroed, update BOTH this file and servo-map.md in
// the same commit -- they describe one physical measurement.
#pragma once
#include <stdint.h>

#include "obs/obs_spec.h"

namespace robot {

// Joint order (obs_spec kJointNames): L hip yaw/roll/pitch, knee, ankle,
// then R the same.
inline constexpr int32_t kAsBuiltZeroSteps[obs::kNumJoints] = {
    1693,   // L_hip_yaw    (id 10)
    2420,   // L_hip_roll   (id 5)
    3273,   // L_hip_pitch  (id 6)
    1634,   // L_knee       (id 7)
    3516,   // L_ankle      (id 8)
    1803,   // R_hip_yaw    (id 9)
    3533,   // R_hip_roll   (id 1)
    2501,   // R_hip_pitch  (id 2)
    2050,   // R_knee       (id 3)
    3450,   // R_ankle      (id 4)
};

// Roll and knee are inverted on BOTH legs (servos mounted the same way on
// each side, not mirrored); yaw, pitch and ankle are not.
inline constexpr int8_t kAsBuiltDir[obs::kNumJoints] = {
    +1, -1, +1, -1, +1,     // left leg
    +1, -1, +1, -1, +1,     // right leg
};

}  // namespace robot
