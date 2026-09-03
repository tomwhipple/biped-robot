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
// Latest attitude + tick-averaged gyro since the previous take. False until
// the first sample has landed (caller keeps its held sample).
bool takeImu(imu::Sample& out);
uint32_t imuSampleTotal();
uint32_t imuSampleFailures();
}  // namespace robot
