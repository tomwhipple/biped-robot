// Attitude fusion: the job the BNO085 would have done on-chip.
//
// The QMI8658C is a raw part. It reports rates and specific force; it does
// not report orientation. Something has to turn those into the obs's
// up-vector, and on this board that something is the ESP32. This is it.
//
// Deliberately a complementary filter and not a Kalman: the whole estimator
// is one quaternion, the correction is a cross product, and it costs a few
// hundred flops in a tick with ~15 ms spare. A Kalman filter would buy
// nothing the policy can use -- domain randomisation already trains against
// 2 deg of misalignment and 0.03 rad/s of gyro bias, which is far larger
// than the difference between the two estimators.
//
//
// WHAT THIS CAN AND CANNOT OBSERVE
//
// Gravity pins roll and pitch. Nothing pins yaw. The correction below is a
// cross product with the measured gravity direction, so it has no component
// about the gravity axis -- by construction it cannot and does not touch
// heading. Yaw is pure dead-reckoned gyro integration and it WILL drift.
//
// That matters here more than it usually does, because obs `up` is the body
// z-axis in the WORLD frame and is therefore yaw-dependent (see the note on
// imu::Sample::up). A heading error of psi rotates the horizontal part of
// `up` by psi. Upright, the horizontal part is ~0 and drift is invisible;
// at 20 deg of tilt it is 0.34, and 5 deg of accumulated yaw error moves it
// by ~0.03 -- about the size of the up-noise the policy trains against.
//
// So the drift budget is roughly: keep heading error inside ~5 deg for the
// length of a run. After a still-bias calibration the residual gyro bias sets
// that; at 0.001 rad/s it is ~1.7 deg after 30 s, at 0.01 rad/s it is 17 deg
// and the horizontal channel is worthless. calibrateBias() is not optional.
//
// The clean fix is not in this file. If the sim emitted projected gravity
// (world +z in the BODY frame -- imu::projectedGravityFromQuaternion) instead
// of framezaxis, the observation would be yaw-INVARIANT, drift would stop
// mattering entirely, and this filter would be exactly enough. That is what
// most locomotion stacks feed their policies. It is a sim change plus a
// retrain, so it is a decision, not a refactor -- but it is the right one,
// and this filter is written so the switch is one line in the driver.
#pragma once

namespace imu {

// All inputs are BODY frame. The driver applies the mounting rotation before
// anything reaches here, so this file never knows how the board is screwed in.
class Fusion {
  public:
    struct Config {
        // Complementary blend rate toward the measured gravity direction,
        // in 1/s. Higher tracks the accelerometer faster and trusts the gyro
        // less. 0.5 puts the crossover near 0.08 Hz: slow enough that a
        // footfall does not tip the estimate, fast enough that a real lean is
        // tracked within a couple of seconds.
        float accel_gain = 0.5f;

        // Reject the gravity correction whenever |accel| leaves
        // [1-tol, 1+tol] * g. A walking robot's accelerometer reads gravity
        // PLUS whatever the torso is doing, and during a footfall that second
        // term is not small. Rejecting those samples is what stops the
        // estimate from being dragged sideways every step; the gyro carries
        // the estimate through, which is the whole point of a complementary
        // filter.
        float accel_tol = 0.15f;

        // Standard gravity, m/s^2 -- the magnitude accel_tol is measured
        // against.
        float g = 9.80665f;
    };

    Fusion() { reset(); }
    explicit Fusion(const Config& cfg) : cfg_(cfg) { reset(); }

    // Level, facing +x, no accumulated heading.
    void reset();

    // Snap the estimate straight to whatever attitude the accelerometer
    // implies, with heading left at zero. Called once on the first valid
    // sample, and it is not a nicety.
    //
    // reset() starts at the identity, which asserts "the board is level and
    // its z points at the sky". If the part is actually mounted any other way
    // -- and on this board it reads -1 g on z with the PCB flat, i.e. 180 deg
    // out -- then the complementary filter has to walk the whole way round at
    // its 2 s time constant, and near 180 deg the correction cross product is
    // nearly zero, so it crawls. Ten-plus seconds of confidently wrong
    // attitude on every boot, right where bring-up looks at it.
    //
    // Aligning from the first sample removes that entirely: the estimate
    // starts correct in roll and pitch and the filter only ever has to track.
    void alignTo(const float accel[3]);

    // AE path. dq is the QMI8658C AttitudeEngine's quaternion INCREMENT over
    // the interval -- already coning-compensated at 1 kHz on-chip, which is
    // strictly better than anything we can do from 50 Hz samples. accel is
    // the specific force over the same interval (m/s^2); pass nullptr to
    // propagate with no gravity correction.
    void updateDeltaQuat(const float dq[4], const float accel[3]);

    // Raw path, for when the AttitudeEngine is unavailable. gyro in rad/s,
    // accel in m/s^2 (nullptr to skip the correction), dt in seconds.
    void updateRate(const float gyro[3], const float accel[3], float dt);

    // Body->world orientation, (w, x, y, z).
    void quat(float out[4]) const;

    // Convenience: obs `up` (body z-axis in world) from the current estimate.
    void up(float out[3]) const;

    // True once a gravity correction has actually been applied, i.e. the
    // estimate is levelled rather than merely integrated from the reset pose.
    bool levelled() const { return levelled_; }

    // How many consecutive updates have rejected the accelerometer. A run of
    // these means the robot is being thrown around and roll/pitch are
    // coasting on the gyro alone -- worth surfacing rather than hiding.
    int rejectStreak() const { return reject_streak_; }

  private:
    void applyGravityCorrection(const float accel[3], float dt);
    void normalize();

    Config cfg_;
    float q_[4];
    bool levelled_;
    int reject_streak_;
};

// Gyro zero-rate bias, averaged with the robot held still. Feeding a biased
// gyro into any of the above is the single fastest way to make the estimate
// useless, and the QMI8658C's untrimmed bias is well inside the range that
// does it.
class BiasEstimator {
  public:
    void reset();
    // Returns true while more samples are still wanted.
    bool accumulate(const float gyro[3]);
    // Peak-to-peak spread seen per axis. Large means the robot was NOT still
    // and the resulting bias is not trustworthy.
    void spread(float out[3]) const;
    void bias(float out[3]) const;
    int count() const { return n_; }

    // ~2 s at 50 Hz. Long enough to average out noise, short enough that a
    // human will actually hold the robot still for it.
    static constexpr int kSamples = 100;

  private:
    float sum_[3];
    float min_[3];
    float max_[3];
    int n_;
};

}  // namespace imu
