#include "imu_sampler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace robot {
namespace {
imu::Imu* s_imu = nullptr;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
imu::Sample s_latest{};
float s_gsum[3] = {0.0f, 0.0f, 0.0f};
uint32_t s_gcount = 0;
uint32_t s_total = 0;
uint32_t s_fail = 0;
bool s_have = false;
constexpr uint32_t kStackWords = 3072;
StackType_t s_stack[kStackWords];
StaticTask_t s_tcb;

void imuTask(void*) {
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        imu::Sample smp{};
        if (s_imu != nullptr && s_imu->read(smp)) {
            portENTER_CRITICAL(&s_mux);
            s_latest = smp;
            for (int i = 0; i < 3; ++i) s_gsum[i] += smp.gyro[i];
            ++s_gcount;
            ++s_total;
            s_have = true;
            portEXIT_CRITICAL(&s_mux);
        } else {
            ++s_fail;
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(kImuPeriodMs));
    }
}
}  // namespace

void startImuSampler(imu::Imu& imu) {
    s_imu = &imu;
    // Core 0 with the link/housekeeping tasks; priority above housekeeping
    // (2) and below the link task (5) so a busy CLI never starves it.
    xTaskCreateStaticPinnedToCore(imuTask, "imu", kStackWords, nullptr, 4,
                                  s_stack, &s_tcb, 0);
}

bool takeImu(imu::Sample& out) {
    portENTER_CRITICAL(&s_mux);
    if (!s_have) {
        portEXIT_CRITICAL(&s_mux);
        return false;
    }
    out = s_latest;
    if (s_gcount > 0) {
        for (int i = 0; i < 3; ++i) out.gyro[i] = s_gsum[i] / static_cast<float>(s_gcount);
    }
    for (int i = 0; i < 3; ++i) s_gsum[i] = 0.0f;
    s_gcount = 0;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

uint32_t imuSampleTotal() { return s_total; }
uint32_t imuSampleFailures() { return s_fail; }
}  // namespace robot
