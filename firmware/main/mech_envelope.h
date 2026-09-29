// Mechanical envelope -- how far each joint may be driven ON THE BENCH.
//
// This is NOT the policy operating range. obs_spec.h's kJointLo/kJointHi are
// generated from the sim plant and define what a policy trains in (and how
// its actions scale -- widening THEM invalidates trained runs). This table is
// the measured/CAD-verified mechanical truth, and only the CLI bench clamp
// (cli.cpp clampToJointRange: `move`, `pose`, `home`) reads it. The act path
// is untouched.
//
// Indexed by BUS joint (obs/bus_map.h: row b is servo ID b + 1), so every
// servo the robot carries has a row, including the ones no policy drives.
//
// Paired with docs/servo-map.md ("Mechanical envelope" section) the same way
// asbuilt_cal.h pairs with the zero table: change one, change both, one
// commit. Every widening beyond the plant range must name its evidence.
//
// Rows 1-10 are the 10-DOF PROTOTYPE's legs, measured on it. The robot's legs
// are different mechanisms (its knee hyperextends only 5 deg, its hip pitch
// flexes to -125): they need their own rows, probed on the robot, before the
// bench drives them there. Rows 11-17 are the robot's joints at their plant
// ranges and have never been probed.
//
// Sign convention: sim joint frame (servo-map.md direction table), radians.
#pragma once

#include "obs/bus_map.h"
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
//  ankle roll +-25   plant range (sim/bimo_biped_v6ar.xml); not probed
//  neck yaw   +-90   plant range (sim/bimo_biped_v6ar.xml); not probed
//  shoulder   -90..+200  plant range (sim/gen_plant_v6.py: 0 hanging, 90
//                straight back, 180 up along the torso); not probed
//  elbow      +-150  plant range (sim/gen_plant_v6.py); not probed
inline constexpr float kMechLo[obs::kNumBusJoints] = {
    -55.0f * kDeg,   //  1 R_hip_roll   abduction, CAD-verified 55 (see above)
    -110.0f * kDeg,  //  2 R_hip_pitch
    -95.0f * kDeg,   //  3 R_knee       flexion, measured
    -40.0f * kDeg,   //  4 R_ankle
    -25.0f * kDeg,   //  5 L_hip_roll   (adduction; leg-leg contact ~9 deg is
                     //                  pose-dependent and NOT this table's job)
    -110.0f * kDeg,  //  6 L_hip_pitch
    -95.0f * kDeg,   //  7 L_knee       flexion, measured
    -40.0f * kDeg,   //  8 L_ankle
    -45.0f * kDeg,   //  9 R_hip_yaw
    -45.0f * kDeg,   // 10 L_hip_yaw
    -25.0f * kDeg,   // 11 R_ankle_roll
    -25.0f * kDeg,   // 12 L_ankle_roll
    -90.0f * kDeg,   // 13 neck_yaw
    -90.0f * kDeg,   // 14 R_shoulder
    -150.0f * kDeg,  // 15 R_elbow
    -90.0f * kDeg,   // 16 L_shoulder
    -150.0f * kDeg,  // 17 L_elbow
};
inline constexpr float kMechHi[obs::kNumBusJoints] = {
    25.0f * kDeg,    //  1 R_hip_roll
    90.0f * kDeg,    //  2 R_hip_pitch  backward, CAD-verified at 90
    95.0f * kDeg,    //  3 R_knee       hyperextension, measured
    40.0f * kDeg,    //  4 R_ankle
    55.0f * kDeg,    //  5 L_hip_roll   abduction, CAD-verified 55 (see above)
    90.0f * kDeg,    //  6 L_hip_pitch  backward, CAD-verified at 90
    95.0f * kDeg,    //  7 L_knee       hyperextension, measured
    40.0f * kDeg,    //  8 L_ankle
    45.0f * kDeg,    //  9 R_hip_yaw
    45.0f * kDeg,    // 10 L_hip_yaw
    25.0f * kDeg,    // 11 R_ankle_roll
    25.0f * kDeg,    // 12 L_ankle_roll
    90.0f * kDeg,    // 13 neck_yaw
    200.0f * kDeg,   // 14 R_shoulder
    150.0f * kDeg,   // 15 R_elbow
    200.0f * kDeg,   // 16 L_shoulder
    150.0f * kDeg,   // 17 L_elbow
};

// The envelope must CONTAIN the policy range: a policy target the act path
// can emit must never be something the bench refuses.
namespace detail {
constexpr bool envelopeContainsPolicyRange() {
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int b = obs::policyToBus(j);
        if (kMechLo[b] > obs::kJointLo[j]) return false;
        if (kMechHi[b] < obs::kJointHi[j]) return false;
    }
    return true;
}
constexpr bool envelopeIsOrdered() {
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (!(kMechLo[b] < kMechHi[b])) return false;
    }
    return true;
}
}  // namespace detail
static_assert(detail::envelopeContainsPolicyRange(),
              "mech envelope narrower than the plant's policy range");
static_assert(detail::envelopeIsOrdered(), "mech envelope row with lo >= hi");

}  // namespace robot
