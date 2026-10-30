// Mechanical envelope -- how far each joint may be driven ON THE BENCH.
//
// This is NOT the policy operating range. obs_spec.h's kJointLo/kJointHi are
// generated from the sim plant and define what a policy trains in (and how
// its actions scale -- widening THEM invalidates trained runs). This table is
// the measured/CAD-verified mechanical truth, and only the CLI bench clamp
// (cli.cpp clampToJointRange: `move`, `pose`, `home`) reads it. The act path
// is untouched.
//
// Indexed by BUS joint (obs/bus_map.h, bus index order), so every
// servo the robot carries has a row, including the ones no policy drives.
//
// Paired with docs/servo-map.md ("Mechanical envelope" section) the same way
// asbuilt_cal.h pairs with the zero table: change one, change both, one
// commit. Every widening beyond the plant range must name its evidence.
//
// kMechLo/kMechHi: rows 1-10 are the 10-DOF PROTOTYPE's legs, measured on
// it; rows 11-17 are the robot's joints at their plant ranges and have never
// been probed. The robot's legs are different mechanisms (its knee
// hyperextends only 5 deg, its hip pitch flexes to -120): kMechLoV6Legs /
// kMechHiV6Legs below are their rows, at the v6 legs plant's POLICY range
// (sim/bimo_biped_v6legs.xml ctrlrange) and never probed. A firmware whose
// compiled policy drives those legs (a policy with the ankle rolls -- the
// v6legs profile, firmware/policies/v6legs) clamps the bench to them; every
// other build to the prototype's. mechLo()/mechHi() pick.
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

// The v6 robot's legs, bus order like the rows above. Evidence: the v6 legs
// plant's policy range (sim/bimo_biped_v6legs.xml ctrlrange, which the v6legs
// obs_spec.h kJointLo/kJointHi are generated from) -- NOT PROBED on the
// robot. Probe before the bench drives a joint to its extreme, and narrow a
// row the robot cannot reach. Two rows to look at first:
//  ankle pitch  +-45 is the plant's ctrlrange, but its joint stops are +-40:
//               the plant lets a policy command 5 deg past the modelled stop
//  knee         -130..+5: flexion past the prototype's measured -95, and a
//               hyperextension of only 5
// The hip rolls' asymmetry is the plant's: abduction is the outward-positive
// L side's + and the R side's -.
inline constexpr float kMechLoV6Legs[obs::kNumBusJoints] = {
    -45.0f * kDeg,   //  1 R_hip_roll   abduction
    -120.0f * kDeg,  //  2 R_hip_pitch  flexion
    -130.0f * kDeg,  //  3 R_knee       flexion
    -45.0f * kDeg,   //  4 R_ankle
    -30.0f * kDeg,   //  5 L_hip_roll   adduction
    -120.0f * kDeg,  //  6 L_hip_pitch  flexion
    -130.0f * kDeg,  //  7 L_knee       flexion
    -45.0f * kDeg,   //  8 L_ankle
    -45.0f * kDeg,   //  9 R_hip_yaw
    -45.0f * kDeg,   // 10 L_hip_yaw
    -25.0f * kDeg,   // 11 R_ankle_roll
    -25.0f * kDeg,   // 12 L_ankle_roll
    -90.0f * kDeg,   // 13 neck_yaw     (as kMechLo: not on the legs-only body)
    -90.0f * kDeg,   // 14 R_shoulder
    -150.0f * kDeg,  // 15 R_elbow
    -90.0f * kDeg,   // 16 L_shoulder
    -150.0f * kDeg,  // 17 L_elbow
};
inline constexpr float kMechHiV6Legs[obs::kNumBusJoints] = {
    30.0f * kDeg,    //  1 R_hip_roll   adduction
    90.0f * kDeg,    //  2 R_hip_pitch  backward
    5.0f * kDeg,     //  3 R_knee       hyperextension
    45.0f * kDeg,    //  4 R_ankle
    45.0f * kDeg,    //  5 L_hip_roll   abduction
    90.0f * kDeg,    //  6 L_hip_pitch  backward
    5.0f * kDeg,     //  7 L_knee       hyperextension
    45.0f * kDeg,    //  8 L_ankle
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

namespace detail {
constexpr bool policyDrivesTheRobotLegs() {
    for (int j = 0; j < obs::kNumJoints; ++j) {
        if (obs::detail::sameName(obs::kJointNames[j], "L_ankle_roll")) {
            return true;
        }
    }
    return false;
}
}  // namespace detail

// True when the compiled policy is the robot's 6-DOF legs' (it drives the
// ankle rolls): the bench then clamps to kMech*V6Legs.
inline constexpr bool kMechIsV6Legs = detail::policyDrivesTheRobotLegs();

// The envelope of bus joint b that this build's bench clamps to.
constexpr float mechLo(int b) {
    // cppcheck-suppress knownConditionTrueFalse -- generated per build
    return kMechIsV6Legs ? kMechLoV6Legs[b] : kMechLo[b];
}
constexpr float mechHi(int b) {
    // cppcheck-suppress knownConditionTrueFalse -- generated per build
    return kMechIsV6Legs ? kMechHiV6Legs[b] : kMechHi[b];
}

// The envelope must CONTAIN the policy range: a policy target the act path
// can emit must never be something the bench refuses.
namespace detail {
constexpr bool envelopeContainsPolicyRange() {
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int b = obs::policyToBus(j);
        if (mechLo(b) > obs::kJointLo[j]) return false;
        if (mechHi(b) < obs::kJointHi[j]) return false;
    }
    return true;
}
constexpr bool envelopeIsOrdered() {
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (!(kMechLo[b] < kMechHi[b])) return false;
        if (!(kMechLoV6Legs[b] < kMechHiV6Legs[b])) return false;
    }
    return true;
}
}  // namespace detail
static_assert(detail::envelopeContainsPolicyRange(),
              "mech envelope narrower than the plant's policy range");
static_assert(detail::envelopeIsOrdered(), "mech envelope row with lo >= hi");

}  // namespace robot
