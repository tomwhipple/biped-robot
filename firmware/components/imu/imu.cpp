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

void upFromQuaternion(float qw, float qx, float qy, float qz, float out[3]) {
    // The obs's "up" is MuJoCo's framezaxis sensor on the torso imu site
    // (sim/bimo_biped_v3yaw.xml: <framezaxis objtype="site" objname="imu">),
    // which reports the site's own z-axis expressed in the WORLD frame -- not
    // gravity in the body frame. For a body->world quaternion that is the
    // third COLUMN of R(q). Upright gives (0, 0, 1), matching the sim.
    out[0] = 2.0f * (qx * qz + qw * qy);
    out[1] = 2.0f * (qy * qz - qw * qx);
    out[2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

bool StubImu::read(Sample& out) {
    applyMount(mount_, up_, out.up);
    applyMount(mount_, gyro_, out.gyro);
    out.t_us = t_us_;
    out.valid = true;
    return true;
}

void StubImu::set(const float up[3], const float gyro[3]) {
    for (int i = 0; i < 3; ++i) {
        up_[i] = up[i];
        gyro_[i] = gyro[i];
    }
    t_us_ += 20000;
}

}  // namespace imu
