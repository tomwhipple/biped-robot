#include "imu/imu.h"

#include <math.h>

namespace imu {

void applyMount(const Mount& m, const float in[3], float out[3]) {
    // v' = q * v * q^-1, expanded (Rodrigues form) to avoid a quaternion type.
    const float t0 = 2.0f * (m.y * in[2] - m.z * in[1]);
    const float t1 = 2.0f * (m.z * in[0] - m.x * in[2]);
    const float t2 = 2.0f * (m.x * in[1] - m.y * in[0]);
    out[0] = in[0] + m.w * t0 + (m.y * t2 - m.z * t1);
    out[1] = in[1] + m.w * t1 + (m.z * t0 - m.x * t2);
    out[2] = in[2] + m.w * t2 + (m.x * t1 - m.y * t0);
}

void upYawStrippedFromQuaternion(float qw, float qx, float qy, float qz,
                                 float out[3]) {
    // yaw (ZYX) of the body->world quaternion, then q_tilt = qz(-yaw) * q
    const float psi = atan2f(2.0f * (qw * qz + qx * qy),
                             1.0f - 2.0f * (qy * qy + qz * qz));
    const float c = cosf(0.5f * psi), s = sinf(0.5f * psi);
    const float tw = c * qw + s * qz;
    const float tx = c * qx + s * qy;
    const float ty = c * qy - s * qx;
    const float tz = c * qz - s * qw;
    upFromQuaternion(tw, tx, ty, tz, out);
}

void upFromQuaternion(float qw, float qx, float qy, float qz, float out[3]) {
    // The obs's "up" is MuJoCo's framezaxis sensor on the torso imu site
    // (sim/bimo_biped_v5body.xml:609 <framezaxis objtype="site" objname="imu">),
    // which reports the site's own z-axis expressed in the WORLD frame -- not
    // gravity in the body frame. For a body->world quaternion that is the
    // third COLUMN of R(q). Upright gives (0, 0, 1), matching the sim.
    out[0] = 2.0f * (qx * qz + qw * qy);
    out[1] = 2.0f * (qy * qz - qw * qx);
    out[2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

void projectedGravityFromQuaternion(float qw, float qx, float qy, float qz,
                                    float out[3]) {
    // Third ROW of R(q) == third column of R(q)^T: the world +z axis in the
    // body frame. Differs from upFromQuaternion by the sign of the two
    // off-diagonal cross terms, which is exactly the transpose.
    out[0] = 2.0f * (qx * qz - qw * qy);
    out[1] = 2.0f * (qy * qz + qw * qx);
    out[2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

bool StubImu::read(Sample& out) {
    applyMount(mount_, up_, out.up);
    applyMount(mount_, gyro_, out.gyro);
    out.t_us = t_us_;
    out.valid = true;
    return true;
}

}  // namespace imu
