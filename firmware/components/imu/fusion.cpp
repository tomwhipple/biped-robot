#include "imu/fusion.h"

#include <math.h>

#include "imu/imu.h"

namespace imu {
namespace {

// Hamilton product, q = a * b.
void quatMul(const float a[4], const float b[4], float out[4]) {
    const float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    const float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    const float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    const float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

float norm3(const float v[3]) {
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

void Fusion::reset() {
    q_[0] = 1.0f;
    q_[1] = 0.0f;
    q_[2] = 0.0f;
    q_[3] = 0.0f;
    levelled_ = false;
    reject_streak_ = 0;
}

void Fusion::alignTo(const float accel[3]) {
    if (accel == nullptr) return;
    const float mag = norm3(accel);
    if (mag < 1e-6f) return;

    const float inv = 1.0f / mag;
    const float v[3] = {accel[0] * inv, accel[1] * inv, accel[2] * inv};

    // Shortest arc carrying the measured gravity direction onto world +z:
    // q = normalize([1 + v.z, v x zhat]). That gives R*v = zhat, hence
    // R^T*zhat = v -- which is exactly "projected gravity equals what the
    // accelerometer measured", with no yaw component introduced.
    const float dot = v[2];
    if (dot > 0.999999f) {          // already level
        reset();
        levelled_ = true;
        return;
    }
    if (dot < -0.999999f) {         // exactly inverted: the arc is ambiguous,
        q_[0] = 0.0f;               // so pick the x-axis and let the filter
        q_[1] = 1.0f;               // refine from there
        q_[2] = 0.0f;
        q_[3] = 0.0f;
        levelled_ = true;
        return;
    }
    q_[0] = 1.0f + dot;
    q_[1] = v[1];
    q_[2] = -v[0];
    q_[3] = 0.0f;
    normalize();
    levelled_ = true;
    reject_streak_ = 0;
}

void Fusion::normalize() {
    float n = sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] +
                    q_[3] * q_[3]);
    if (n < 1e-9f) {   // numerically dead; the identity is the safe answer
        reset();
        return;
    }
    const float inv = 1.0f / n;
    for (int i = 0; i < 4; ++i) q_[i] *= inv;
    // Keep the scalar part non-negative so the quaternion never wanders onto
    // the far cover of SO(3). Same rotation, but it keeps `quat()` readable
    // and small-angle reasoning valid downstream.
    if (q_[0] < 0.0f) {
        for (int i = 0; i < 4; ++i) q_[i] = -q_[i];
    }
}

void Fusion::applyGravityCorrection(const float accel[3], float dt) {
    if (accel == nullptr) return;

    const float mag = norm3(accel);
    if (mag < 1e-6f) return;

    // Free fall, an impact, or a hard torso acceleration: the accelerometer is
    // not measuring gravity right now, so refusing to look at it is more
    // accurate than averaging it in.
    const float lo = cfg_.g * (1.0f - cfg_.accel_tol);
    const float hi = cfg_.g * (1.0f + cfg_.accel_tol);
    if (mag < lo || mag > hi) {
        ++reject_streak_;
        return;
    }
    reject_streak_ = 0;

    const float inv = 1.0f / mag;
    const float ax = accel[0] * inv;
    const float ay = accel[1] * inv;
    const float az = accel[2] * inv;

    // Where the current estimate says world +z sits in the body frame. At
    // rest the accelerometer measures reaction to gravity, so a levelled
    // sensor reads +z -- the same vector. Any disagreement is attitude error.
    float pred[3];
    projectedGravityFromQuaternion(q_[0], q_[1], q_[2], q_[3], pred);

    // Rotation that carries `pred` onto the measurement.
    //
    // The order is measured x predicted, and it is not arbitrary. Updating
    // q <- q * dq post-multiplies, so R -> R*Rd and the predicted gravity
    // R^T z transforms as Rd^-1 pred -- the correction rotates `pred` by
    // MINUS delta. Solving delta from "rotate pred by -delta onto meas" gives
    // delta = meas x pred. Writing pred x meas instead does not merely
    // converge slowly: it inverts the feedback, making the true attitude an
    // unstable equilibrium and the 180-degree antipode the stable one. The
    // filter then settles, confidently, upside down.
    const float ex = ay * pred[2] - az * pred[1];
    const float ey = az * pred[0] - ax * pred[2];
    const float ez = ax * pred[1] - ay * pred[0];

    // Note what this cross product cannot do: it is orthogonal to both
    // vectors, so it has no component about the gravity direction. Heading is
    // untouched, by construction -- which is the honest statement of what a
    // 6-axis part can observe.

    const float k = cfg_.accel_gain * dt;
    // Small-angle: dq ~ (1, k*e/2). Exact normalisation happens below, so the
    // approximation costs nothing at the gains and rates used here.
    const float dq[4] = {1.0f, 0.5f * k * ex, 0.5f * k * ey, 0.5f * k * ez};
    float out[4];
    quatMul(q_, dq, out);
    for (int i = 0; i < 4; ++i) q_[i] = out[i];
    levelled_ = true;
}

void Fusion::updateDeltaQuat(const float dq[4], const float accel[3]) {
    float out[4];
    quatMul(q_, dq, out);
    for (int i = 0; i < 4; ++i) q_[i] = out[i];
    normalize();

    // The increment carries its own interval. Recovering dt from its rotation
    // angle would be circular, so the correction is applied at the nominal
    // tick -- the gain is a soft blend, not a physical rate, and it is
    // insensitive to a few percent of dt error.
    applyGravityCorrection(accel, 0.02f);
    normalize();
}

void Fusion::updateRate(const float gyro[3], const float accel[3], float dt) {
    if (dt > 0.0f) {
        const float hx = 0.5f * gyro[0] * dt;
        const float hy = 0.5f * gyro[1] * dt;
        const float hz = 0.5f * gyro[2] * dt;
        // First-order integration. At 50 Hz and the rates a walking torso
        // reaches, the truncation error is far below the gyro's own noise --
        // and when the AttitudeEngine is available this path is not used at
        // all, because the chip integrates at 1 kHz with coning compensation.
        const float dq[4] = {1.0f, hx, hy, hz};
        float out[4];
        quatMul(q_, dq, out);
        for (int i = 0; i < 4; ++i) q_[i] = out[i];
        normalize();
    }
    applyGravityCorrection(accel, dt);
    normalize();
}

void Fusion::quat(float out[4]) const {
    for (int i = 0; i < 4; ++i) out[i] = q_[i];
}

void Fusion::up(float out[3]) const {
    // yaw-stripped: see imu.h. The raw column-3 form is upFromQuaternion.
    upYawStrippedFromQuaternion(q_[0], q_[1], q_[2], q_[3], out);
}

// -- BiasEstimator ---------------------------------------------------------

void BiasEstimator::reset() {
    for (int i = 0; i < 3; ++i) {
        sum_[i] = 0.0f;
        min_[i] = 0.0f;
        max_[i] = 0.0f;
    }
    n_ = 0;
}

bool BiasEstimator::accumulate(const float gyro[3]) {
    if (n_ >= kSamples) return false;
    for (int i = 0; i < 3; ++i) {
        sum_[i] += gyro[i];
        if (n_ == 0 || gyro[i] < min_[i]) min_[i] = gyro[i];
        if (n_ == 0 || gyro[i] > max_[i]) max_[i] = gyro[i];
    }
    ++n_;
    return n_ < kSamples;
}

void BiasEstimator::bias(float out[3]) const {
    const float inv = (n_ > 0) ? 1.0f / static_cast<float>(n_) : 0.0f;
    for (int i = 0; i < 3; ++i) out[i] = sum_[i] * inv;
}

void BiasEstimator::spread(float out[3]) const {
    for (int i = 0; i < 3; ++i) out[i] = max_[i] - min_[i];
}

}  // namespace imu
