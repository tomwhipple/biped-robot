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

// Re-zeroed 2026-09-03 after Tom found and tightened loose servo-horn screws
// on both legs (pitch chains had wandered -3.8/-4.7/+1.7 deg L and -1.9/+1.1/
// -1.1 deg R; yaw and roll < 0.3 deg). Stand set by eye, `cal zero` + `cal save`.
// Joint order (obs_spec kJointNames): L hip yaw/roll/pitch, knee, ankle,
// then R the same.
inline constexpr int32_t kAsBuiltZeroSteps[obs::kNumJoints] = {
    1692,   // L_hip_yaw    (id 10)
    2418,   // L_hip_roll   (id 5)
    2001,   // L_hip_pitch  (id 6)  re-centred 2026-08-03: old zero 3273 left
            //   only +72 deg before the 4095 wrap; `middle` latched standing
            //   as 2048 (4 ticks past zero at the latch -> zero 2044), the
            //   R_knee treatment applied to the second of the joints
            //   servo-map.md predicted would need it
    1581,   // L_knee       (id 7)
    3535,   // L_ankle      (id 8)
    1806,   // R_hip_yaw    (id 9)
    3532,   // R_hip_roll   (id 1)
    2479,   // R_hip_pitch  (id 2)
    2063,   // R_knee       (id 3)
    3437,   // R_ankle      (id 4)
};

// Roll and knee are inverted on BOTH legs (servos mounted the same way on
// each side, not mirrored); yaw, pitch and ankle are not.
//
// The knee -1 was chosen so a NEGATIVE commanded angle flexes the knee the
// human way. Until 2026-08-02 the plant disagreed -- its knee axis made
// negative mean hyperextension -- and sim and robot bent opposite ways for the
// same command. That was fixed in sim/bimo_biped_v3yaw.xml (axis "0 -1 0"), on
// the SIM side: these numbers, the NVS blob and obs_spec's joint limits are all
// unchanged, so the fix needs no reflash and no re-zero.
inline constexpr int8_t kAsBuiltDir[obs::kNumJoints] = {
    +1, -1, +1, -1, +1,     // left leg
    +1, -1, +1, -1, +1,     // right leg
};

}  // namespace robot
