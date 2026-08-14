// Attitude fusion, on the host, under sanitizers.
//
// The values here are not invented. Every attitude assertion is cross-checked
// against MuJoCo's framezaxis sensor on the real plant, because the sim is
// what the policy was trained on and therefore what "correct" means:
//
//   sim/bimo_biped_v5body.xml, <framezaxis objtype="site" objname="imu">
//     upright            (0,      0,      1)
//     roll  +20 deg      (0,     -0.342,  0.9397)
//     pitch +20 deg      (0.342,  0,      0.9397)
//     yaw   +90 deg      (0,      0,      1)
//     roll +20, yaw +90  (0.342,  0,      0.9397)   <-- yaw-DEPENDENT
//
// That last line is the one worth having a test for. It is the reason the
// filter has to dead-reckon heading at all, and if anyone ever "fixes" the
// obs to be yaw-invariant, this file is where the change announces itself.
#include <math.h>

#include "imu/fusion.h"
#include "imu/imu.h"
#include "test_util.h"

namespace {

constexpr float kG = 9.80665f;
constexpr float kDeg = 3.14159265358979f / 180.0f;

// Body-frame accelerometer reading for a given roll/pitch, at rest: the
// reaction to gravity, i.e. world +z expressed in the body frame, times g.
void restAccel(float roll, float pitch, float out[3]) {
    const float q[4] = {cosf(roll * 0.5f) * cosf(pitch * 0.5f),
                        sinf(roll * 0.5f) * cosf(pitch * 0.5f),
                        cosf(roll * 0.5f) * sinf(pitch * 0.5f),
                        -sinf(roll * 0.5f) * sinf(pitch * 0.5f)};
    float pg[3];
    imu::projectedGravityFromQuaternion(q[0], q[1], q[2], q[3], pg);
    for (int i = 0; i < 3; ++i) out[i] = pg[i] * kG;
}

// -- the frame convention, pinned to the MuJoCo numbers above ---------------

void testUpMatchesMujoco() {
    float up[3];
    const float r = 20.0f * kDeg;
    const float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);

    imu::upFromQuaternion(1.0f, 0.0f, 0.0f, 0.0f, up);
    CHECK_NEAR(up[0], 0.0, 1e-6);
    CHECK_NEAR(up[1], 0.0, 1e-6);
    CHECK_NEAR(up[2], 1.0, 1e-6);

    // roll +20 about body x
    imu::upFromQuaternion(cr, sr, 0.0f, 0.0f, up);
    CHECK_NEAR(up[0], 0.0, 1e-4);
    CHECK_NEAR(up[1], -0.342, 1e-3);
    CHECK_NEAR(up[2], 0.9397, 1e-3);

    // pitch +20 about body y
    imu::upFromQuaternion(cr, 0.0f, sr, 0.0f, up);
    CHECK_NEAR(up[0], 0.342, 1e-3);
    CHECK_NEAR(up[1], 0.0, 1e-4);
    CHECK_NEAR(up[2], 0.9397, 1e-3);

    // yaw alone leaves the body z-axis on world +z
    const float h = 0.70710678f;
    imu::upFromQuaternion(h, 0.0f, 0.0f, h, up);
    CHECK_NEAR(up[0], 0.0, 1e-4);
    CHECK_NEAR(up[1], 0.0, 1e-4);
    CHECK_NEAR(up[2], 1.0, 1e-4);

    // ...but yaw THEN roll rotates the horizontal part. q = qz(90) * qx(20).
    const float qw = h * cr, qx = h * sr, qy = h * sr, qz = h * cr;
    imu::upFromQuaternion(qw, qx, qy, qz, up);
    CHECK_NEAR(up[0], 0.342, 1e-3);
    CHECK_NEAR(up[1], 0.0, 1e-3);
    CHECK_NEAR(up[2], 0.9397, 1e-3);
}

// projected gravity is the transpose, and it is yaw-INVARIANT. Both facts get
// a test because the whole design hinges on the difference.
void testProjectedGravityIsTheTranspose() {
    const float r = 20.0f * kDeg;
    const float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);
    float pg[3], up[3];

    imu::projectedGravityFromQuaternion(cr, sr, 0.0f, 0.0f, pg);
    imu::upFromQuaternion(cr, sr, 0.0f, 0.0f, up);
    // Under a pure roll the two differ by the SIGN of y -- the exact,
    // silent, one-sided failure the comments warn about.
    CHECK_NEAR(pg[1], 0.342, 1e-3);
    CHECK_NEAR(up[1], -0.342, 1e-3);
    CHECK_NEAR(pg[2], up[2], 1e-6);

    // Yaw invariance: roll +20 alone vs. the same roll at yaw +90.
    const float h = 0.70710678f;
    float pg_yawed[3];
    const float qw = h * cr, qx = h * sr, qy = h * sr, qz = h * cr;
    imu::projectedGravityFromQuaternion(qw, qx, qy, qz, pg_yawed);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(pg_yawed[i], pg[i], 1e-3);
}

// -- the filter ------------------------------------------------------------

void testLevelConverges() {
    imu::Fusion f;
    float a[3];
    restAccel(0.0f, 0.0f, a);
    for (int i = 0; i < 500; ++i) {
        const float g[3] = {0.0f, 0.0f, 0.0f};
        f.updateRate(g, a, 0.02f);
    }
    float up[3];
    f.up(up);
    CHECK(f.levelled());
    CHECK_NEAR(up[0], 0.0, 1e-3);
    CHECK_NEAR(up[1], 0.0, 1e-3);
    CHECK_NEAR(up[2], 1.0, 1e-3);
}

// Held at a fixed tilt with a silent gyro, the filter must WALK to that tilt
// from the accelerometer alone. This is the "someone put the robot down on a
// slope" case, and it is what the complementary gain is for.
void testGravityPullsToTilt() {
    imu::Fusion f;
    const float roll = 20.0f * kDeg;
    float a[3];
    restAccel(roll, 0.0f, a);
    const float g[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 2000; ++i) f.updateRate(g, a, 0.02f);

    float up[3];
    f.up(up);
    CHECK_NEAR(up[1], -0.342, 5e-3);
    CHECK_NEAR(up[2], 0.9397, 5e-3);
}

// Gyro-only propagation must agree with the closed form. This is the channel
// that carries the estimate through a footfall, so it has to be right on its
// own, with no accelerometer to lean on.
void testGyroIntegrates() {
    imu::Fusion f;
    const float rate = 20.0f * kDeg;      // rad/s about body x
    const float g[3] = {rate, 0.0f, 0.0f};
    for (int i = 0; i < 50; ++i) f.updateRate(g, nullptr, 0.02f);   // 1 s

    float up[3];
    f.up(up);
    CHECK_NEAR(up[1], -0.342, 5e-3);
    CHECK_NEAR(up[2], 0.9397, 5e-3);
    CHECK(!f.levelled());                 // never saw an accelerometer
}

// The AE path and the raw path must land in the same place for the same
// motion, or switching between them silently changes the observation.
void testDeltaQuatMatchesRate() {
    const float rate = 20.0f * kDeg;
    const float dt = 0.02f;

    imu::Fusion a, b;
    const float g[3] = {rate, 0.0f, 0.0f};
    const float half = 0.5f * rate * dt;
    const float dq[4] = {cosf(half), sinf(half), 0.0f, 0.0f};
    for (int i = 0; i < 50; ++i) {
        a.updateRate(g, nullptr, dt);
        b.updateDeltaQuat(dq, nullptr);
    }
    float ua[3], ub[3];
    a.up(ua);
    b.up(ub);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(ua[i], ub[i], 1e-3);
}

// Free fall and hard impacts ARE rejected: their magnitude leaves the window,
// so the accelerometer is visibly not measuring gravity and the gyro carries
// the estimate alone.
void testAccelRejectionOnMagnitude() {
    imu::Fusion f;
    float rest[3];
    restAccel(0.0f, 0.0f, rest);
    const float still[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 500; ++i) f.updateRate(still, rest, 0.02f);

    float before[3];
    f.up(before);

    const float freefall[3] = {0.0f, 0.0f, 0.05f * kG};
    for (int i = 0; i < 10; ++i) f.updateRate(still, freefall, 0.02f);
    CHECK(f.rejectStreak() == 10);

    const float slam[3] = {0.0f, 0.0f, 2.0f * kG};
    for (int i = 0; i < 10; ++i) f.updateRate(still, slam, 0.02f);
    CHECK(f.rejectStreak() == 20);

    // Twenty rejected samples in a row moved the estimate by exactly nothing.
    float after[3];
    f.up(after);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(after[i], before[i], 1e-6);
}

// The honest limit of a magnitude gate: a SIDEWAYS shove barely changes |a|,
// so it sails through the window while pointing 27 deg off gravity. This is
// the footfall case, and it is not rejected -- 0.5 g lateral on 1 g vertical
// is 1.12 g, inside a 15% window.
//
// What actually bounds the damage is the gain. At 0.5/s a quarter-second
// transient can only drag the estimate a couple of degrees, and it walks back
// as soon as the shove ends. That is the complementary filter working as
// designed, not a gap in it -- but it does mean the up-vector carries a small
// footfall-rate ripple, which is worth knowing before blaming the policy.
void testLateralShoveIsBoundedNotRejected() {
    imu::Fusion f;
    float rest[3];
    restAccel(0.0f, 0.0f, rest);
    const float still[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 500; ++i) f.updateRate(still, rest, 0.02f);

    const float shove[3] = {0.5f * kG, 0.0f, kG};
    for (int i = 0; i < 12; ++i) f.updateRate(still, shove, 0.02f);
    CHECK(f.rejectStreak() == 0);        // magnitude gate never fires

    float after[3];
    f.up(after);
    const float tilt = sqrtf(after[0] * after[0] + after[1] * after[1]);
    CHECK(tilt > 0.01f);                 // it DID move: no pretending otherwise
    CHECK(tilt < 0.09f);                 // ...but by under ~5 deg

    // And it recovers once the torso settles.
    for (int i = 0; i < 500; ++i) f.updateRate(still, rest, 0.02f);
    f.up(after);
    CHECK_NEAR(after[2], 1.0, 1e-3);
}

// Yaw is unobservable and the filter must not pretend otherwise: gravity
// corrections may move roll and pitch as much as they like, but they must
// leave heading exactly where the gyro put it.
void testGravityCannotCorrectYaw() {
    imu::Fusion f;
    const float rate = 30.0f * kDeg;
    const float spin[3] = {0.0f, 0.0f, rate};      // 1 s of pure yaw
    float rest[3];
    restAccel(0.0f, 0.0f, rest);
    for (int i = 0; i < 50; ++i) f.updateRate(spin, rest, 0.02f);

    float q[4];
    f.quat(q);
    // 30 deg of yaw survives, level is held.
    const float yaw = 2.0f * atan2f(q[3], q[0]) / kDeg;
    CHECK_NEAR(yaw, 30.0, 1.0);

    float up[3];
    f.up(up);
    CHECK_NEAR(up[2], 1.0, 1e-3);

    // ...and no amount of further levelling pulls the heading back to zero.
    const float still[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 1000; ++i) f.updateRate(still, rest, 0.02f);
    f.quat(q);
    const float yaw2 = 2.0f * atan2f(q[3], q[0]) / kDeg;
    CHECK_NEAR(yaw2, 30.0, 1.0);
}

// alignTo must land the estimate on the measured gravity IMMEDIATELY, from
// any starting attitude -- including the inverted case, which is not
// hypothetical: the QMI8658C on this board reads -1 g on its z with the PCB
// flat, so every boot starts 180 deg from the identity.
void testAlignToSnapsToGravity() {
    // Level.
    {
        imu::Fusion f;
        float a[3];
        restAccel(0.0f, 0.0f, a);
        f.alignTo(a);
        float up[3];
        f.up(up);
        CHECK(f.levelled());
        CHECK_NEAR(up[2], 1.0, 1e-4);
    }
    // 20 deg of roll, in one step rather than 2000.
    {
        imu::Fusion f;
        float a[3];
        restAccel(20.0f * kDeg, 0.0f, a);
        f.alignTo(a);
        float up[3];
        f.up(up);
        CHECK_NEAR(up[1], -0.342, 1e-3);
        CHECK_NEAR(up[2], 0.9397, 1e-3);
    }
    // Fully inverted: the arc is ambiguous and must not produce a NaN or a
    // zero quaternion. Any attitude with z pointing down is acceptable.
    {
        imu::Fusion f;
        const float a[3] = {0.0f, 0.0f, -kG};
        f.alignTo(a);
        float q[4], up[3];
        f.quat(q);
        f.up(up);
        const float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        CHECK_NEAR(n, 1.0, 1e-5);
        CHECK_NEAR(up[2], -1.0, 1e-4);
    }
    // The real board's reading, near-inverted but not exactly: this is the
    // case that used to crawl for ten seconds.
    {
        imu::Fusion f;
        const float a[3] = {0.42f, 1.25f, -9.75f};
        f.alignTo(a);
        float up[3];
        f.up(up);
        // One update later it must still be there, not drifting away.
        const float still[3] = {0.0f, 0.0f, 0.0f};
        f.updateRate(still, a, 0.02f);
        float after[3];
        f.up(after);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(after[i], up[i], 1e-3);
        CHECK(up[2] < -0.9f);
    }
}

// -- mounting --------------------------------------------------------------

// The board stands VERTICAL and TRANSVERSE in the pelvis aft recess
// (cad/dimensions.py:999): PCB plane in the robot's y-z, long edge across y,
// bus edge up. That is ~90 deg about y, not a trim -- and the existing
// test_obs coverage only ever exercised identity and 90 deg about z.
void testNinetyDegreeMount() {
    imu::Mount m;
    const float h = 0.70710678f;
    m.w = h; m.y = h;                    // +90 deg about y

    // The chip's +z (PCB normal) comes out pointing along the robot's +x.
    const float sensor_z[3] = {0.0f, 0.0f, 1.0f};
    float body[3];
    imu::applyMount(m, sensor_z, body);
    CHECK_NEAR(body[0], 1.0, 1e-5);
    CHECK_NEAR(body[1], 0.0, 1e-5);
    CHECK_NEAR(body[2], 0.0, 1e-5);

    // ...and the chip's -x comes out as the robot's +z, which is the axis the
    // gravity correction will actually be working with.
    const float sensor_negx[3] = {-1.0f, 0.0f, 0.0f};
    imu::applyMount(m, sensor_negx, body);
    CHECK_NEAR(body[0], 0.0, 1e-5);
    CHECK_NEAR(body[2], 1.0, 1e-5);

    // A mount must not change a vector's length, ever.
    const float skew[3] = {0.3f, -0.5f, 0.81f};
    imu::applyMount(m, skew, body);
    const float n0 = sqrtf(0.3f * 0.3f + 0.5f * 0.5f + 0.81f * 0.81f);
    const float n1 = sqrtf(body[0] * body[0] + body[1] * body[1] +
                           body[2] * body[2]);
    CHECK_NEAR(n1, n0, 1e-5);
}

// End to end with the real mount: a chip lying in the recess, robot upright,
// must produce up = (0,0,1) after the mount is applied and the filter settles.
void testMountedAndLevel() {
    imu::Mount m;
    const float h = 0.70710678f;
    m.w = h; m.y = h;

    // Upright robot => the chip, rotated -90 about y from the body, reads
    // gravity along its own -x.
    const float sensor_accel[3] = {-kG, 0.0f, 0.0f};
    float body_accel[3];
    imu::applyMount(m, sensor_accel, body_accel);
    CHECK_NEAR(body_accel[2], kG, 1e-3);

    imu::Fusion f;
    const float still[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 500; ++i) f.updateRate(still, body_accel, 0.02f);
    float up[3];
    f.up(up);
    CHECK_NEAR(up[2], 1.0, 1e-3);
}

// -- bias ------------------------------------------------------------------

void testBiasEstimator() {
    imu::BiasEstimator b;
    b.reset();
    const float truth[3] = {0.01f, -0.02f, 0.005f};
    int guard = 0;
    while (b.accumulate(truth) && guard++ < 10000) {
    }
    CHECK(b.count() == imu::BiasEstimator::kSamples);

    float out[3];
    b.bias(out);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(out[i], truth[i], 1e-6);

    float spread[3];
    b.spread(spread);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(spread[i], 0.0, 1e-6);

    // A robot that was NOT still has to be visible as spread, because that is
    // the only thing standing between a bad calibration and a drifting run.
    imu::BiasEstimator w;
    w.reset();
    for (int i = 0; i < imu::BiasEstimator::kSamples; ++i) {
        const float wobble[3] = {(i % 2) ? 0.3f : -0.3f, 0.0f, 0.0f};
        w.accumulate(wobble);
    }
    w.spread(spread);
    CHECK(spread[0] > 0.5f);
}

}  // namespace

int main() {
    testUpMatchesMujoco();
    testProjectedGravityIsTheTranspose();
    testLevelConverges();
    testGravityPullsToTilt();
    testGyroIntegrates();
    testDeltaQuatMatchesRate();
    testAccelRejectionOnMagnitude();
    testLateralShoveIsBoundedNotRejected();
    testGravityCannotCorrectYaw();
    testAlignToSnapsToGravity();
    testNinetyDegreeMount();
    testMountedAndLevel();
    testBiasEstimator();
    return testutil::report("fusion");
}
