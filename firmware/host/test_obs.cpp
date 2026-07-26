// Obs assembler, history ring, gait clock, action/angle/tick maps, IMU maths.
//
// The frames in obs_vectors.h came out of walker_env._obs() itself (see
// tools/gen_obs_spec.py). If anybody reorders the observation in the sim and
// regenerates, this test is what goes red.
#include "imu/imu.h"
#include "obs/actuation.h"
#include "obs/assembler.h"
#include "test_util.h"
#include "vectors/obs_vectors.h"

namespace {

void testSpecMatchesVectors() {
    CHECK_EQ(obs::kNumJoints, obs_vectors::kNumJoints);
    CHECK_EQ(obs::kNumCmd, obs_vectors::kNumCmd);
    CHECK_EQ(obs::kFrameDim, obs_vectors::kFrameDim);
    CHECK_EQ(obs::kObsDim, obs::kFrameDim * obs::kHistLen);
    // The deployed plant is the 10-DOF v3yaw one (MEMORY: all sims on
    // bimo_biped_v3yaw.xml), so the joint count is 10, not 8.
    CHECK_EQ(obs::kNumJoints, 10);
    CHECK(obs::kImuObs);
}

void testServoIdMapIsAPermutation() {
    // docs/wiring.md's bus IDs vs. the sim's action order. On the 10-DOF plant
    // this is NOT the identity: L_hip_yaw is action 0 but servo ID 9.
    bool seen[16] = {};
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const uint8_t id = obs::kServoId[i];
        CHECK(id >= 1 && id <= 10);
        CHECK(!seen[id]);
        seen[id] = true;
    }
    CHECK_EQ(obs::kServoId[0], 9);      // L_hip_yaw
    CHECK_EQ(obs::kServoId[1], 1);      // L_hip_roll
    CHECK_EQ(obs::kServoId[5], 10);     // R_hip_yaw
    CHECK_EQ(obs::kServoId[9], 8);      // R_ankle
}

void testFramesMatchTheSim() {
    for (size_t c = 0; c < obs_vectors::kNumCases; ++c) {
        const auto& k = obs_vectors::kCases[c];
        obs::Inputs in{};
        for (int i = 0; i < obs::kNumJoints; ++i) {
            in.q[i] = k.q[i];
            in.dq[i] = k.dq[i];
            in.prev_action[i] = k.prev_action[i];
        }
        for (int i = 0; i < 3; ++i) {
            in.up[i] = k.up[i];
            in.gyro[i] = k.gyro[i];
        }
        for (int i = 0; i < obs::kNumCmd; ++i) in.cmd[i] = k.cmd[i];
        in.phase = k.phase;

        float frame[obs::kFrameDim];
        obs::assembleFrame(in, frame);
        for (int i = 0; i < obs::kFrameDim; ++i) {
            CHECK_NEAR(frame[i], k.frame[i], 1e-6);
        }
        // The two channels the robot cannot sense are zero, on purpose.
        CHECK_EQ(frame[obs::kOffLinVel + 0] == 0.0f, true);
        CHECK_EQ(frame[obs::kOffHeight] == 0.0f, true);
    }
}

void testHistoryOrdering() {
    // walker_env: obs at tick t is [f_t, f_{t-1}, f_{t-2}], and the ring is
    // pushed AFTER the policy has been fed.
    float f[3][obs::kFrameDim];
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < obs::kFrameDim; ++i) {
            f[k][i] = static_cast<float>(k * 1000 + i);
        }
    }
    obs::History h;
    h.fill(f[0]);                       // reset semantics: all slots = f0
    float o[obs::kObsDim];
    h.build(f[0], o);
    for (int k = 0; k < obs::kHistLen; ++k) {
        CHECK_NEAR(o[k * obs::kFrameDim + 5], f[0][5], 0.0);
    }
    h.push(f[0]);

    h.build(f[1], o);
    CHECK_NEAR(o[5], f[1][5], 0.0);
    CHECK_NEAR(o[obs::kFrameDim + 5], f[0][5], 0.0);
    CHECK_NEAR(o[2 * obs::kFrameDim + 5], f[0][5], 0.0);
    h.push(f[1]);

    h.build(f[2], o);
    CHECK_NEAR(o[5], f[2][5], 0.0);
    CHECK_NEAR(o[obs::kFrameDim + 5], f[1][5], 0.0);
    CHECK_NEAR(o[2 * obs::kFrameDim + 5], f[0][5], 0.0);
}

void testGaitClockWraps() {
    // (phase + 2*pi*dt*f + pi) % (2*pi) - pi, i.e. always in (-pi, pi].
    obs::GaitClock clk(1.5f);
    clk.setPhase(3.0f);
    for (int i = 0; i < 500; ++i) {
        clk.advance(0.02f);
        CHECK(clk.phase() >= -3.14159266f && clk.phase() <= 3.14159266f);
    }
    // One full cycle at 1.5 Hz is 1/1.5 s; after that the phase returns.
    obs::GaitClock c2(1.5f);
    c2.setPhase(0.0f);
    for (int i = 0; i < 100; ++i) c2.advance(1.0f / 150.0f);   // exactly 1 rev
    CHECK_NEAR(c2.phase(), 0.0, 1e-3);
}

void testActionToAngles() {
    // action_map "full": action 0 is the standing pose, +-1 the joint limits.
    float action[obs::kNumJoints], angle[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = 0.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointDefault[i], 1e-6);
    }
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = 1.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointHi[i], 1e-6);
    }
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = -1.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointLo[i], 1e-6);
    }
    // Out-of-range actions are clamped, not extrapolated past the mechanism.
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = 5.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointHi[i], 1e-6);
    }
    // Round trip through the inverse.
    const float probe[] = {-1.0f, -0.5f, 0.0f, 0.25f, 0.9f};
    for (float a : probe) {
        for (int i = 0; i < obs::kNumJoints; ++i) action[i] = a;
        obs::actionToAngles(action, angle);
        float back[obs::kNumJoints];
        obs::anglesToAction(angle, back);
        for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(back[i], a, 1e-5);
    }
}

void testAngleToSteps() {
    obs::Calibration cal;
    CHECK_EQ(obs::angleToSteps(0, 0.0f, cal), 2048);
    // 45 deg is an eighth of 4096 ticks. Joint 0 (hip yaw) travels exactly
    // +-45 deg, so a commanded quarter turn clamps to the same tick.
    CHECK_EQ(obs::angleToSteps(0, obs::kJointHi[0], cal), 2048 + 512);
    const float quarter = 1.5707963f;
    CHECK_EQ(obs::angleToSteps(0, quarter, cal), 2048 + 512);
    // Direction and zero offset are honoured.
    cal.dir[3] = -1;
    cal.zero_steps[3] = 1900;
    const float a = -0.5f;
    const int32_t s = obs::angleToSteps(3, a, cal);
    CHECK_NEAR(obs::stepsToAngle(3, s, cal), a, 2e-3);
    CHECK(s > 1900);                            // negative angle, inverted dir
}

void testVelocityEstimator() {
    obs::VelocityEstimator est(1.0f);           // raw difference
    float q[obs::kNumJoints] = {}, v[obs::kNumJoints];
    est.reset(q);
    for (int i = 0; i < obs::kNumJoints; ++i) q[i] = 0.02f;
    est.update(q, 0.02f, v);
    for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(v[i], 1.0, 1e-5);

    obs::Calibration cal;
    // 4096 steps/s is exactly one revolution per second.
    CHECK_NEAR(obs::stepsPerSecToRadPerSec(0, 4096, cal), 6.2831853, 1e-4);
}

void testImuMaths() {
    // Level: the torso z-axis is world +z.
    float up[3];
    imu::upFromQuaternion(1.0f, 0.0f, 0.0f, 0.0f, up);
    CHECK_NEAR(up[0], 0.0, 1e-6);
    CHECK_NEAR(up[1], 0.0, 1e-6);
    CHECK_NEAR(up[2], 1.0, 1e-6);

    // 90 deg pitch about +y tips the body z-axis onto world +x.
    const float s = 0.70710678f;
    imu::upFromQuaternion(s, 0.0f, s, 0.0f, up);
    CHECK_NEAR(up[0], 1.0, 1e-5);
    CHECK_NEAR(up[2], 0.0, 1e-5);

    // The mounting rotation is the firmware twin of walker_env's imu_R @ up.
    imu::Mount m;                      // identity
    const float in[3] = {0.1f, -0.2f, 0.97f};
    float out[3];
    imu::applyMount(m, in, out);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(out[i], in[i], 1e-6);

    m.w = s; m.z = s;                  // +90 deg about z
    imu::applyMount(m, in, out);
    CHECK_NEAR(out[0], 0.2, 1e-5);
    CHECK_NEAR(out[1], 0.1, 1e-5);
    CHECK_NEAR(out[2], 0.97, 1e-5);

    imu::StubImu stub;
    imu::Sample smp{};
    CHECK(stub.init());
    CHECK(stub.read(smp));
    CHECK(smp.valid);
    CHECK_NEAR(smp.up[2], 1.0, 1e-6);
}

}  // namespace

// -- calibration persistence + the steps clamp -----------------------------
// The blob format is plain data, so it is testable here even though the NVS
// side of cal_store only compiles under ESP-IDF.
#include "../main/cal_store.h"

void testCalBlob() {
    obs::Calibration cal;
    for (int i = 0; i < obs::kNumJoints; ++i) {
        cal.zero_steps[i] = 1000 + i * 37;
        cal.dir[i] = (i % 3 == 0) ? -1 : 1;
    }
    robot::CalBlob blob{};
    robot::calPack(cal, blob);

    obs::Calibration back;
    CHECK(robot::calUnpack(blob, back));   // round-trip unpack succeeds
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK(back.zero_steps[i] == cal.zero_steps[i]);   // zero survives
        CHECK(back.dir[i] == cal.dir[i]);   // dir survives
    }

    // Every rejection path must leave the destination untouched, so a bad
    // blob falls back to defaults instead of half-applying.
    obs::Calibration guard;
    const int32_t sentinel = guard.zero_steps[0];

    robot::CalBlob bad = blob;
    bad.crc ^= 0xFFu;
    CHECK(!robot::calUnpack(bad, guard));   // corrupt CRC rejected
    CHECK(guard.zero_steps[0] == sentinel);   // rejected blob did not mutate

    bad = blob; bad.magic = 0xDEADBEEF;
    CHECK(!robot::calUnpack(bad, guard));   // wrong magic rejected

    bad = blob; bad.version = 99;
    CHECK(!robot::calUnpack(bad, guard));   // wrong version rejected

    // The 8-DOF -> 10-DOF plant change is exactly why joints is stored.
    bad = blob; bad.joints = 8;
    robot::calPack(cal, bad); bad.joints = 8;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // joint-count mismatch rejected

    bad = blob; bad.zero_steps[2] = 9999;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // out-of-range zero rejected

    bad = blob; bad.dir[1] = 7;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // invalid dir rejected

    CHECK(robot::calCrc32("123456789", 9) == 0xCBF43926u);   // CRC-32 matches the standard check value
}

void testStepsClamp() {
    // A wrong zero_steps must not produce a target off the encoder range --
    // that is a horn parked on a hard stop drawing stall current.
    obs::Calibration cal;
    for (int i = 0; i < obs::kNumJoints; ++i) cal.zero_steps[i] = 4090;
    float angles[obs::kNumJoints];
    float act[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) act[i] = 1.0f;
    obs::actionToAngles(act, angles);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const int32_t st = obs::angleToSteps(i, angles[i], cal);
        CHECK(st >= 0 && st <= 4095);   // clamped high-side zero stays in range
    }
    for (int i = 0; i < obs::kNumJoints; ++i) cal.zero_steps[i] = 5;
    for (int i = 0; i < obs::kNumJoints; ++i) act[i] = -1.0f;
    obs::actionToAngles(act, angles);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const int32_t st = obs::angleToSteps(i, angles[i], cal);
        CHECK(st >= 0 && st <= 4095);   // clamped low-side zero stays in range
    }
}

int main() {
    testSpecMatchesVectors();
    testServoIdMapIsAPermutation();
    testFramesMatchTheSim();
    testHistoryOrdering();
    testGaitClockWraps();
    testActionToAngles();
    testAngleToSteps();
    testVelocityEstimator();
    testImuMaths();
    testCalBlob();
    testStepsClamp();
    return testutil::report("obs");
}
