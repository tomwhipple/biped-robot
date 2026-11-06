// HeadingHold: an outer IMU heading loop around the walking policy (Tom 2026-11-06: "I would
// expect IMU feedback to keep the robot walking in a straight line").
//
// The policy observes the body RATES (gyro) and the gravity direction with yaw stripped, never
// its heading, so it can damp a spin but cannot steer back from one. 2026-11-06 test 3
// (stride1_03e): a left spin at the end of the second swing, peak 6.5 rad/s, +83 deg at +0.78 s,
// settled at +51 deg -- and nothing ever turned it back.
//
// This loop does what the policy cannot. When a walk command begins it captures the fused
// heading (imu::Sample::quat, dead-reckoned yaw: it drifts ~1 deg/s, negligible over a few
// steps). While the walk lasts, the reference advances with the COMMANDED turn rate, and the
// loop adds wz = clamp(-k * wrap(yaw - ref), +-wz_max) to the turn the policy is asked for.
// The policy already follows turn commands (trained: turn_emph, cmd_w_range 1.0).
//
// It cannot stop a 0.1 s spin: the tick is 20 ms and the servos lag 60 - 80 ms. It steers the
// heading back over the following steps. Off when no walk is commanded (no pivoting at a stand),
// and the caller keeps it off once WalkRecovery has taken the command to the stand.
//
// Pure logic, no clock or globals: host/test_flailguard.cpp.
#pragma once
#include <math.h>

namespace flailguard {

class HeadingHold {
  public:
    struct Config {
        float k = 1.0f;          // rad/s of turn command per rad of heading error
        float wz_max = 0.5f;     // |correction| bound, rad/s
    };
    HeadingHold() : C_(Config{}) {}
    explicit HeadingHold(const Config& c) : C_(c) {}

    void reset() { active_ = false; ref_ = 0.0f; err_ = 0.0f; }
    void setGain(float k) { C_.k = k; }

    // One control tick. yaw: fused heading, rad. cmd_wz: the turn the link commands, rad/s.
    // walking: a walk/step command is active (and no recovery override). Returns the turn
    // rate to ADD to the policy's command (0 when not walking).
    float update(float yaw, float cmd_wz, bool walking, float dt_s) {
        if (!walking) {
            active_ = false;
            err_ = 0.0f;
            return 0.0f;
        }
        if (!active_) {                   // a walk begins: hold the heading it begins at
            active_ = true;
            ref_ = yaw;
        } else {
            ref_ = wrap(ref_ + cmd_wz * dt_s);
        }
        err_ = wrap(yaw - ref_);
        float c = -C_.k * err_;
        if (c > C_.wz_max) c = C_.wz_max;
        if (c < -C_.wz_max) c = -C_.wz_max;
        return c;
    }

    float error() const { return err_; }        // rad, + = turned left of the reference

    // Heading (yaw about world z) of a body->world quaternion (w, x, y, z).
    static float yawOf(const float q[4]) {
        return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                      1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    }
    static float wrap(float a) {
        const float kPi = 3.14159265f;
        while (a > kPi) a -= 2.0f * kPi;
        while (a < -kPi) a += 2.0f * kPi;
        return a;
    }

  private:
    Config C_;
    bool active_ = false;
    float ref_ = 0.0f, err_ = 0.0f;
};

}  // namespace flailguard
