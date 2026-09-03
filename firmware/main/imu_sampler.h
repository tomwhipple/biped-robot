// High-rate IMU sampler (2026-09-03, Tom: "increase the IMU sample and control
// loop rate as much as practical"). The control loop stays at 50 Hz -- the
// policy network alone is ~7.5 ms of the 20 ms tick and the policy was trained
// at that step -- but the IMU no longer waits for it: a task on core 0 reads
// the sensor every kImuPeriodMs, runs the attitude filter at that rate, and
// hands the loop the latest up-vector plus the gyro AVERAGED over the tick
// (anti-aliased, instead of one instantaneous sample every 20 ms). It also
// takes the IMU read off the control tick's budget.
#pragma once
#include <stdint.h>
#include "imu/imu.h"

namespace robot {
constexpr int kImuPeriodMs = 4;                 // 250 Hz
void startImuSampler(imu::Imu& imu);
void imuSamplerSetSource(imu::Imu& imu);   // after a runtime re-init
// Latest attitude + tick-averaged gyro since the previous take. False until
// the first sample has landed (caller keeps its held sample).
bool takeImu(imu::Sample& out);
uint32_t imuSampleTotal();
// Defined in app_main.cpp: recover + re-init the live QMI (bench `imu reinit`).
bool imuReinitLive();
// Tick-averaging of the gyro handed to the control loop (default on). Off =
// the latest 4 ms sample, i.e. the pre-sampler behaviour with 5x the rate.
void imuSetAverage(bool on);
bool imuAverage();
uint32_t imuSampleFailures();
// Ringing meter (Tom 2026-09-03: "if this is important we should design
// another way to measure it"): the sampler keeps the last ~4 s of body-frame
// gyro; imuRing() summarises the last `ms` -- per-50 ms RMS envelope of the
// horizontal (pitch/roll) rate, peak, dominant frequency from zero crossings
// of the livelier axis, and decay = RMS(last quarter)/RMS(first quarter).
struct RingStats {
    int n = 0;
    float peak = 0.0f;            // rad/s
    float rms_first = 0.0f, rms_last = 0.0f;
    float f_hz = 0.0f;
    int nbins = 0;
    float env[80];                // per-50 ms RMS, oldest first (<= 4 s)
    float integ[3];               // integrated body-frame gyro over the window, rad
    float mean[3];                // mean body-frame gyro over the window, rad/s
};
bool imuRing(int ms, RingStats& out);
}  // namespace robot
