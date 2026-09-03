#include "imu_sampler.h"

#include <math.h>

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
constexpr int kRingN = 1024;                    // 4.1 s at 4 ms
float s_ring[kRingN][3];
uint32_t s_ring_w = 0;                          // total written
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
            for (int i = 0; i < 3; ++i) {
                s_gsum[i] += smp.gyro[i];
                s_ring[s_ring_w % kRingN][i] = smp.gyro[i];
            }
            ++s_ring_w;
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

bool imuRing(int ms, RingStats& out) {
    int n = ms / kImuPeriodMs;
    if (n > kRingN) n = kRingN;
    static float g[kRingN][3];      // 12 kB: never on a task stack
    portENTER_CRITICAL(&s_mux);
    const uint32_t w = s_ring_w;
    if (static_cast<int>(w) < n) n = static_cast<int>(w);
    for (int k = 0; k < n; ++k) {
        const uint32_t idx = (w - n + k) % kRingN;
        for (int i = 0; i < 3; ++i) g[k][i] = s_ring[idx][i];
    }
    portEXIT_CRITICAL(&s_mux);
    out = RingStats{};
    if (n < 8) return false;
    out.n = n;
    // bias: mean over the window (a settle window is mostly still)
    float m[3] = {0, 0, 0};
    for (int k = 0; k < n; ++k) for (int i = 0; i < 3; ++i) m[i] += g[k][i];
    for (int i = 0; i < 3; ++i) m[i] /= static_cast<float>(n);
    float var[2] = {0, 0};
    const int per_bin = 50 / kImuPeriodMs;
    int bin = 0; float acc = 0.0f; int cnt = 0;
    for (int k = 0; k < n; ++k) {
        const float x = g[k][0] - m[0], y = g[k][1] - m[1];
        const float mag2 = x * x + y * y;
        const float mag = sqrtf(mag2);
        if (mag > out.peak) out.peak = mag;
        var[0] += x * x; var[1] += y * y;
        acc += mag2; ++cnt;
        if (cnt == per_bin && bin < 80) { out.env[bin++] = sqrtf(acc / cnt); acc = 0; cnt = 0; }
        if (k < n / 4) out.rms_first += mag2;
        if (k >= n - n / 4) out.rms_last += mag2;
    }
    if (cnt > 0 && bin < 80) out.env[bin++] = sqrtf(acc / cnt);
    out.nbins = bin;
    out.rms_first = sqrtf(out.rms_first / (n / 4));
    out.rms_last = sqrtf(out.rms_last / (n / 4));
    const int ax = var[0] >= var[1] ? 0 : 1;
    int zc = 0;
    for (int k = 1; k < n; ++k) {
        const float a = g[k - 1][ax] - m[ax], b = g[k][ax] - m[ax];
        if ((a < 0 && b >= 0) || (a >= 0 && b < 0)) ++zc;
    }
    out.f_hz = static_cast<float>(zc) / 2.0f /
               (static_cast<float>(n) * kImuPeriodMs / 1000.0f);
    return true;
}

uint32_t imuSampleTotal() { return s_total; }
uint32_t imuSampleFailures() { return s_fail; }
}  // namespace robot
