#pragma once
#include "imu/imu.h"

namespace robot {

// Starts the 50 Hz control task on core 1 (statically allocated) and the
// esp_timer that paces it. Call once, after the bus and IMU are up.
void startCtrlTask(imu::Imu& imu);

}  // namespace robot
