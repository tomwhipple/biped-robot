// Mechanical envelope -- how far each joint may be driven ON THE BENCH.
//
// This is NOT the policy operating range. obs_spec.h's kJointLo/kJointHi are
// generated from the sim plant and define what a policy trains in (and how
// its actions scale -- widening THEM invalidates trained runs). This table is
// the measured/CAD-verified mechanical truth, and only the CLI bench clamp
// (cli.cpp clampToJointRange) reads it. The act path is untouched.
//
// Paired with docs/servo-map.md ("Mechanical envelope" section) the same way
// asbuilt_cal.h pairs with the zero table: change one, change both, one
// commit. Every widening beyond the plant range must name its evidence.
//
// Sign convention: sim joint frame (servo-map.md direction table), radians.
// Joint order: kJointNames -- L_hip_yaw, L_hip_roll, L_hip_pitch, L_knee,
// L_ankle, R_hip_yaw, R_hip_roll, R_hip_pitch, R_knee, R_ankle.
#pragma once

#include "obs/obs_spec.h"

namespace robot {

inline constexpr float kDeg = 0.0174532925f;

// Evidence per row:
//  yaw    +-45   plant range; mechanical stops unrecorded, never probed wider
//  roll   adduction 25 (plant range; leg-leg contact rules first anyway),
//                abduction 55: CAD clearance sweep 2026-08-03 -- hip stack
//                vs carrier+screws clears the 0.5 mm buffer at 55 deg
//                (0.60 mm), goes under at 70 (0.33) and touches by 90
//                (0.00). Creep-verify on hardware before trusting a full
//                55 deg sweep; the clamp is the backstop, not the plan.
//  pitch  -110 plant range; +90 backward: CAD grip-screws-vs-yoke buffer
//                0.70 mm at +90, HARDWARE-VERIFIED 2026-08-03 (both hips to
//                +90 with knees hyperextended 90, loads on the pure-gravity
//                curve throughout, no stall; needed the L_hip_pitch encoder
//                re-centring first -- asbuilt_cal.h)
//  knee   +-95   MEASURED 2026-08-02 (servo 3 full sweeps, <=5.9% load,
//                31 C flat; servo-map.md "Measured knee ROM")
//  ankle  +-40   plant range; toes clear the shin at both extremes (2026-08-03)
inline constexpr float kMechLo[obs::kNumJoints] = {
    -45.0f * kDeg,   // L_hip_yaw
    -25.0f * kDeg,   // L_hip_roll   (adduction; leg-leg contact ~9 deg is
                     //  pose-dependent and NOT this table's job)
    -110.0f * kDeg,  // L_hip_pitch
    -95.0f * kDeg,   // L_knee   flexion, measured
    -40.0f * kDeg,   // L_ankle
    -45.0f * kDeg,   // R_hip_yaw
    -55.0f * kDeg,   // R_hip_roll   abduction, CAD-verified 55 (see above)
    -110.0f * kDeg,  // R_hip_pitch
    -95.0f * kDeg,   // R_knee   flexion, measured
    -40.0f * kDeg,   // R_ankle
};
inline constexpr float kMechHi[obs::kNumJoints] = {
    45.0f * kDeg,    // L_hip_yaw
    55.0f * kDeg,    // L_hip_roll   abduction, CAD-verified 55 (see above)
    90.0f * kDeg,    // L_hip_pitch  backward, CAD-verified at 90
    95.0f * kDeg,    // L_knee   hyperextension, measured
    40.0f * kDeg,    // L_ankle
    45.0f * kDeg,    // R_hip_yaw
    25.0f * kDeg,    // R_hip_roll
    90.0f * kDeg,    // R_hip_pitch  backward, CAD-verified at 90
    95.0f * kDeg,    // R_knee   hyperextension, measured
    40.0f * kDeg,    // R_ankle
};

// The envelope must CONTAIN the policy range: a policy target the act path
// can emit must never be something the bench refuses.
namespace detail {
constexpr bool envelopeContainsPolicyRange() {
    for (int i = 0; i < obs::kNumJoints; ++i) {
        if (kMechLo[i] > obs::kJointLo[i]) return false;
        if (kMechHi[i] < obs::kJointHi[i]) return false;
    }
    return true;
}
}  // namespace detail
static_assert(detail::envelopeContainsPolicyRange(),
              "mech envelope narrower than the plant's policy range");

}  // namespace robot
