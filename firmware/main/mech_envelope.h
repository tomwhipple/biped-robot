// Mechanical envelope -- how far each joint may be driven ON THE BENCH.
//
// This is NOT the policy operating range. obs_spec.h's kJointLo/kJointHi are
// generated from the sim plant and define what a policy trains in (and how
// its actions scale -- widening THEM invalidates trained runs). This table is
// the bench's clamp: the CLI (cli.cpp clampToJointRange: `move`, `pose`,
// `home`) clamps every bus joint to it, and ctrl_task.cpp clamps a HELD
// servo's trained target (the neck, the arms) to it. The act path is
// otherwise untouched.
//
// Indexed by BUS joint (obs/bus_map.h, bus index order), so every
// servo the robot carries has a row, including the ones no policy drives.
//
// Paired with docs/servo-map.md ("Mechanical envelope" section) the same way
// asbuilt_cal.h pairs with the zero table: change one, change both, one
// commit. Every widening beyond the plant range must name its evidence.
//
// kMechLo/kMechHi: the v6 robot's joints. The leg rows (1-12) are the v6 legs
// plant's POLICY range (build/v6/sim/bimo_biped_v6legs.xml ctrlrange), not
// the robot's stops: its measured stops (2026-11-02) lie outside every leg
// row (the v6 knee's +5 deg is a POLICY cap -- the mechanism hyperextends to
// ~95, measured 93.8 on 2026-11-02 -- and its hip pitch's hard stop is -124).
// The 10-DOF prototype's measured leg rows lived in a second table here until
// #81 folded them away: no compiled policy has selected them since 1e60f65,
// and every policy set drives the ankle rolls (static_assert below).
//
// Sign convention: sim joint frame (servo-map.md direction table), radians.
#pragma once

#include "obs/bus_map.h"
#include "obs/obs_spec.h"

namespace robot {

inline constexpr float kDeg = 0.0174532925f;

// Evidence, the legs (rows 1-10; 11-12 below): the v6 legs plant's policy
// range (build/v6/sim/bimo_biped_v6legs.xml ctrlrange, which the v6legs
// obs_spec.h kJointLo/kJointHi are generated from). PROBED on the robot
// 2026-11-02 (hw_sessions/2026-11-02/NOTES.md "ROM pass 3", servo-map.md
// "Measured on the robot"). No row needed narrowing, and the values are
// unchanged (they are the policy range; the containment static_assert ties
// them to kJointLo/kJointHi). Each is inside the measured travel except the
// inward hip roll, where the legs meet first, and R hip pitch +90 backward
// (R reached +84 before a marginal guard trip; L +104.8; CAD clear to 90).
// Measured, sim angles, L / R:
//  hip yaw      free to +-86.7 (no stop; the CAD gap is a constant 1.8 mm)
//  hip roll     outward free to 57.7; inward: the legs meet at ~8 (feet,
//               other leg at 0) -- pose-dependent, not this table's job
//  hip pitch    forward hard stop -125.1 / -123.6; backward +104.8 (L)
//  knee         flexion -130.4 free (CAD ~132); hyperextension +93.8 free
//               (CAD 95) -- the +5 here is the policy's cap, not the joint's
//  ankle pitch  free to +-78 both legs. The plant's "stops +-40" are a CAD
//               design range, not metal: the +-45 rows below (the plant's
//               ctrlrange) are well inside the robot's travel
//  ankle roll   stop on the outer-edge-up side only, +27.9 L / -26.9 R; the
//               inner side free past 46
// build/v6/sim/bimo_biped_v6legs_meas.xml carries those stops with this same policy
// range (under --ctrl-within-stops: ankle +-40, every other row as here).
// The hip rolls' asymmetry is the plant's: abduction is the outward-positive
// L side's + and the R side's -.
//
// Evidence, the rest:
//  ankle roll +-25   plant range (build/v6/sim/bimo_biped_v6ar.xml); measured
//                2026-11-02: the stop is on the outer-edge-up side only,
//                +27.9 L / -26.9 R (the roll and pitch servo cases meet), the
//                inner side free past 46 -- so +-25 sits 2-3 deg inside it
//  neck yaw   +-90   plant range (build/v6/sim/bimo_biped_v6ar.xml); not probed
//  ARMS (14-17): zero is the owner's HOME pose, NOT the plant's 0 (Tom
//                2026-11-16: "elbows backward, forearms pulled up" -- each
//                servo `middle`d there, so home = 2048; cal zero 2048, dir
//                R_sh -1 / R_el +1 / L_sh +1 / L_el -1). Measured on the
//                bench 2026-11-16 (arms on the desk, 20-tick steps, stop at
//                25 ticks lag or 30 % load; docs/servo-map.md 3.2/3.3):
//  shoulder   NO software limit (Tom 2026-11-16): +-360 here never clamps,
//                the encoder's 0..4095 is the only bound. Swept 179 deg free
//                one way; the desk, not the joint, stopped the other at -127
//  elbow      0 .. +179: 0 = home = the FOLD HARD STOP (load 23 % 3.5 deg
//                into it); the open side runs free through +179.3, the end of
//                the encoder range from home, on both elbows
inline constexpr float kMechLo[obs::kNumBusJoints] = {
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
    -90.0f * kDeg,   // 13 neck_yaw     (not on the legs-only body)
    -360.0f * kDeg,  // 14 R_shoulder  no software limit (Tom 2026-11-16)
    0.0f * kDeg,     // 15 R_elbow     fold hard stop = home, measured
    -360.0f * kDeg,  // 16 L_shoulder  no software limit (Tom 2026-11-16)
    0.0f * kDeg,     // 17 L_elbow     fold hard stop = home, measured
};
inline constexpr float kMechHi[obs::kNumBusJoints] = {
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
    360.0f * kDeg,   // 14 R_shoulder
    179.0f * kDeg,   // 15 R_elbow     free through +179.3 (encoder end)
    360.0f * kDeg,   // 16 L_shoulder
    179.0f * kDeg,   // 17 L_elbow     free through +179.3 (encoder end)
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
// The leg rows are the v6 robot's: a policy that does not drive its legs (one
// with no ankle rolls, like the 10-DOF prototype's) has no envelope here.
static_assert(detail::policyDrivesTheRobotLegs(),
              "mech envelope: the compiled policy does not drive the v6 legs");

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
