#include "net_bench.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "policy/mlp.h"
#include "policy/q8.h"
#include "shared.h"

#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace robot {
namespace {

constexpr int kMaxPasses = 200;
constexpr int kMaxRows = 16;
constexpr uint32_t kCpuMhz = 240;
constexpr uint32_t kRamCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;

struct Row {
    char name[40];
    bool ran;
    char note[48];
    uint32_t min_us, med_us, max_us;
};

// Everything the bench needs, in one heap block that exists only while
// `nettime` runs: the image keeps no .bss for a bench command (the deployed
// image's free heap is what WiFi lives on). Written by the bench task,
// printed by the CLI task after the semaphore -- the Sink is the
// housekeeping task's, never called from core 1.
struct Mem {
    Row rows[kMaxRows];
    uint32_t samples[kMaxPasses];
    float obs[policy::kObsDim];
    float act[policy::kActDim];
#ifndef POLICY_WEIGHTS_Q8
    float fa[policy::kMaxWidth], fb[policy::kMaxWidth];
#endif
    float qf[policy::scratchQ8Floats(policy::kMaxWidth)];
    alignas(4) int16_t qq[policy::scratchQ8Int16(policy::kMaxWidth)];
    int32_t qa[policy::scratchQ8Int32(policy::kMaxWidth)];
};
Mem* g_m = nullptr;
int g_nrows = 0;
int g_passes = 20;
SemaphoreHandle_t g_done = nullptr;
StaticSemaphore_t g_done_buf;
volatile float g_sink;     // keeps the activation loops from being elided

Row* addRow(const char* name) {
    if (g_nrows >= kMaxRows) return nullptr;
    Row* r = &g_m->rows[g_nrows++];
    memset(r, 0, sizeof *r);
    snprintf(r->name, sizeof r->name, "%s", name);
    return r;
}

void skipRow(const char* name, const char* why) {
    Row* r = addRow(name);
    if (r) snprintf(r->note, sizeof r->note, "%s", why);
}

int cmpU32(const void* a, const void* b) {
    const uint32_t x = *static_cast<const uint32_t*>(a);
    const uint32_t y = *static_cast<const uint32_t*>(b);
    return x < y ? -1 : (x > y ? 1 : 0);
}

template <typename F>
void timeRow(const char* name, const char* note, F&& fn) {
    Row* r = addRow(name);
    if (!r) return;
    fn();                                     // one untimed pass
    for (int k = 0; k < g_passes; ++k) {
        const uint32_t c0 = esp_cpu_get_cycle_count();
        fn();
        g_m->samples[k] = esp_cpu_get_cycle_count() - c0;
    }
    qsort(g_m->samples, static_cast<size_t>(g_passes), sizeof g_m->samples[0], cmpU32);
    r->ran = true;
    r->min_us = g_m->samples[0] / kCpuMhz;
    r->med_us = g_m->samples[g_passes / 2] / kCpuMhz;
    r->max_us = g_m->samples[g_passes - 1] / kCpuMhz;
    if (note) snprintf(r->note, sizeof r->note, "%s", note);
}

// A fixed obs inside the policy's distribution: mean + std * z, z in
// [-1, 1] and never 0 (the float kernel skips zero inputs, so an all-mean
// obs would time a sparse net). Frozen channels stay at their mean, as
// pinFrozen() feeds them.
void makeObs() {
    uint32_t s = 20261102u;
    for (int i = 0; i < policy::kObsDim; ++i) {
        s = s * 1664525u + 1013904223u;
        float z = static_cast<float>(s >> 8) / 8388608.0f - 1.0f;
        if (fabsf(z) < 0.05f) z = 0.05f;
        g_m->obs[i] = policy::isFrozen(i) ? policy::kNormMean[i]
                                       : policy::kNormMean[i] + policy::kNormStd[i] * z;
    }
}

policy::NetQ8 netQ8(const policy::DenseQ8* layers, int n) {
    policy::NetQ8 net;
    net.obs_dim = policy::kObsDim;
    net.act_dim = policy::kActDim;
    net.num_layers = n;
    net.layers = layers;
    net.norm_mean = policy::kNormMean;
    net.norm_std = policy::kNormStd;
    net.xf = g_m->qf;
    net.xq = g_m->qq;
    net.acc = g_m->qa;
    return net;
}

void benchActivations() {
    timeRow("swish x512, expf (float nets)", nullptr, [] {
        float acc = 0.0f;
        for (int i = 0; i < 512; ++i) {
            const float y = static_cast<float>(i - 256) * 0.02f;
            acc += y / (1.0f + expf(-y));
        }
        g_sink = acc;
    });
    timeRow("swish x512, swishFast (int8 nets)", nullptr, [] {
        float acc = 0.0f;
        for (int i = 0; i < 512; ++i) {
            acc += policy::swishFast(static_cast<float>(i - 256) * 0.02f);
        }
        g_sink = acc;
    });
}

#ifdef POLICY_WEIGHTS_Q8

size_t layerBytes(const policy::DenseQ8& L) {
    return static_cast<size_t>(L.out) * L.in_pad + 4u;
}

void benchQ8Build() {
    using policy::Kernel;
    using policy::kLayersQ8;
    using policy::kNumLayers;
    timeRow("q8 c, flash", nullptr, [] {
        policy::forwardNetQ8(netQ8(kLayersQ8, kNumLayers), g_m->obs, g_m->act, Kernel::kC);
    });
    if (policy::haveMac16()) {
        timeRow("q8 mac16, flash", policy::status().mac16_matches ? nullptr
                                                                   : "MAC16 != C: not used",
                [] {
            policy::forwardNetQ8(netQ8(kLayersQ8, kNumLayers), g_m->obs, g_m->act,
                                 Kernel::kMac16);
        });
    }
    // All layers in RAM, as many as the heap gives (the placement init()
    // would need a bigger budget for).
    policy::DenseQ8 ram[kNumLayers];
    int8_t* copies[kNumLayers] = {};
    int in_ram = 0;
    for (int l = 0; l < kNumLayers; ++l) {
        ram[l] = kLayersQ8[l];
        copies[l] = static_cast<int8_t*>(heap_caps_malloc(layerBytes(kLayersQ8[l]), kRamCaps));
        if (copies[l]) {
            memcpy(copies[l], kLayersQ8[l].w, layerBytes(kLayersQ8[l]));
            ram[l].w = copies[l];
            ++in_ram;
        }
    }
    char note[48];
    snprintf(note, sizeof note, "%d/%d layers in RAM", in_ram, kNumLayers);
    const Kernel k = policy::status().kernel;
    timeRow(k == Kernel::kMac16 ? "q8 mac16, RAM" : "q8 c, RAM", note, [&ram, k] {
        policy::forwardNetQ8(netQ8(ram, kNumLayers), g_m->obs, g_m->act, k);
    });
    // Per layer, kernel only: flash vs RAM is the cost of streaming it.
    policy::quantizeActs(g_m->obs, policy::kObsDim, policy::kMaxWidth, g_m->qq);
    for (int l = 0; l < kNumLayers; ++l) {
        char name[40];
        snprintf(name, sizeof name, "  layer %d (%u B) %s, flash", l,
                 static_cast<unsigned>(layerBytes(kLayersQ8[l])), policy::kernelName(k));
        const policy::DenseQ8 L = kLayersQ8[l];
        timeRow(name, nullptr, [L, k] {
            if (k == Kernel::kMac16) {
                policy::q8RowsMac16(g_m->qq, L.w, L.in_pad, L.out, g_m->qa);
            } else {
                policy::q8RowsC(g_m->qq, L.w, L.in_pad, L.out, g_m->qa);
            }
        });
        snprintf(name, sizeof name, "  layer %d (%u B) %s, RAM", l,
                 static_cast<unsigned>(layerBytes(kLayersQ8[l])), policy::kernelName(k));
        if (!copies[l]) {
            skipRow(name, "no heap block that big");
            continue;
        }
        const policy::DenseQ8 R = ram[l];
        timeRow(name, nullptr, [R, k] {
            if (k == Kernel::kMac16) {
                policy::q8RowsMac16(g_m->qq, R.w, R.in_pad, R.out, g_m->qa);
            } else {
                policy::q8RowsC(g_m->qq, R.w, R.in_pad, R.out, g_m->qa);
            }
        });
    }
    for (int8_t* p : copies) heap_caps_free(p);
}

#else  // float build

void benchFloatBuild() {
    using policy::kLayers;
    using policy::kNumLayers;
    // The same float net with its weights in RAM, layer by layer as the
    // heap allows.
    policy::Dense fl[kNumLayers];
    float* fcopies[kNumLayers] = {};
    int in_ram = 0;
    for (int l = 0; l < kNumLayers; ++l) {
        fl[l] = kLayers[l];
        const size_t n = static_cast<size_t>(kLayers[l].in) * kLayers[l].out;
        fcopies[l] = static_cast<float*>(heap_caps_malloc(4u * n, kRamCaps));
        if (fcopies[l]) {
            memcpy(fcopies[l], kLayers[l].w, 4u * n);
            fl[l].w = fcopies[l];
            ++in_ram;
        }
    }
    char note[48];
    snprintf(note, sizeof note, "%d/%d layers in RAM", in_ram, kNumLayers);
    timeRow("f32, RAM", note, [&fl] {
        policy::Net net;
        net.obs_dim = policy::kObsDim;
        net.act_dim = policy::kActDim;
        net.num_layers = kNumLayers;
        net.layers = fl;
        net.norm_mean = policy::kNormMean;
        net.norm_std = policy::kNormStd;
        net.scratch_a = g_m->fa;
        net.scratch_b = g_m->fb;
        policy::forwardNet(net, g_m->obs, g_m->act);
    });
    for (float* p : fcopies) heap_caps_free(p);

    // This net quantized to int8 here (round-to-nearest, policy::
    // quantizeDense -- the timing of an int8 build of it, not its accuracy:
    // a real int8 profile is GPTQ-rounded offline).
    policy::DenseQ8 q[kNumLayers];
    void* blocks[3 * kNumLayers] = {};
    bool ok = true;
    for (int l = 0; l < kNumLayers && ok; ++l) {
        const int keep = l + 1 == kNumLayers ? policy::kActDim : kLayers[l].out;
        const int in_pad = policy::pad4(kLayers[l].in);
        auto* w = static_cast<int8_t*>(
            heap_caps_malloc(static_cast<size_t>(keep) * static_cast<size_t>(in_pad) + 4u, kRamCaps));
        auto* s = static_cast<float*>(heap_caps_malloc(4u * static_cast<size_t>(keep), kRamCaps));
        auto* b = static_cast<float*>(heap_caps_malloc(4u * static_cast<size_t>(keep), kRamCaps));
        blocks[3 * l] = w;
        blocks[3 * l + 1] = s;
        blocks[3 * l + 2] = b;
        if (!w || !s || !b) {
            ok = false;
            break;
        }
        policy::quantizeDense(kLayers[l], keep, w, s, b);
        q[l] = policy::DenseQ8{kLayers[l].in, static_cast<uint16_t>(in_pad),
                               static_cast<uint16_t>(keep), w, s, b};
    }
    if (!ok) {
        skipRow("q8 (this net, RTN), RAM", "no heap for the int8 copy");
    } else {
        timeRow("q8 c (this net, RTN), RAM", nullptr, [&q] {
            policy::forwardNetQ8(netQ8(q, kNumLayers), g_m->obs, g_m->act, policy::Kernel::kC);
        });
        if (policy::haveMac16()) {
            timeRow("q8 mac16 (this net, RTN), RAM", nullptr, [&q] {
                policy::forwardNetQ8(netQ8(q, kNumLayers), g_m->obs, g_m->act,
                                     policy::Kernel::kMac16);
            });
        }
    }
    for (void* p : blocks) heap_caps_free(p);
}

#endif

void benchTask(void*) {
    g_nrows = 0;
    makeObs();
    timeRow("as run: policy::forward()", nullptr, [] { policy::forward(g_m->obs, g_m->act); });
#ifdef POLICY_WEIGHTS_Q8
    benchQ8Build();
#else
    benchFloatBuild();
#endif
    benchActivations();
    xSemaphoreGive(g_done);
    vTaskDelete(nullptr);
}

void say(cli::Sink out, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void say(cli::Sink out, const char* fmt, ...) {
    char line[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    out(line);
}

}  // namespace

void netTime(cli::Sink out, int passes) {
    if (g_mode_request.load() != Mode::kBench || g_ctrl_owns_bus.load()) {
        out("nettime: refused while the loop is armed -- `bench` first "
            "(it runs on the control core)\r\n");
        return;
    }
    if (passes < 3) passes = 3;
    if (passes > kMaxPasses) passes = kMaxPasses;
    g_passes = passes;
    if (!g_done) g_done = xSemaphoreCreateBinaryStatic(&g_done_buf);
    const size_t heap_free = heap_caps_get_free_size(kRamCaps);
    const size_t heap_largest = heap_caps_get_largest_free_block(kRamCaps);

    const policy::Status& st = policy::status();
    say(out, "nettime: %s, %d obs -> %d act, %s, %d passes/variant on core 1\r\n",
        policy::kWeightsRun, policy::kObsDim, policy::kActDim,
        st.quantized ? "int8" : "float", passes);
    say(out, "  policy: %u B of weights, %d/%d layers in RAM (%u B); kernel %s%s; "
             "self-test %s (err %.2g)\r\n",
        static_cast<unsigned>(st.weight_bytes), st.layers_in_ram, st.num_layers,
        static_cast<unsigned>(st.ram_bytes), policy::kernelName(st.kernel),
        !st.quantized ? " (n/a: float)"
                      : (!st.mac16_checked ? ""
                                           : (st.mac16_matches ? " (MAC16 == C)"
                                                               : " (MAC16 != C, using C)")),
        !st.selftest_ran ? "n/a" : (st.selftest_ok ? "ok" : "FAILED"),
        static_cast<double>(st.selftest_err));
    say(out, "  heap (internal, 8-bit): %u B free, largest block %u B\r\n",
        static_cast<unsigned>(heap_free), static_cast<unsigned>(heap_largest));

    g_m = static_cast<Mem*>(heap_caps_calloc(1, sizeof(Mem), kRamCaps));
    if (!g_m) {
        say(out, "nettime: no heap for the bench (%u B)\r\n", static_cast<unsigned>(sizeof(Mem)));
        return;
    }
    // Below the control task (which keeps ticking, benched) on its core.
    if (xTaskCreatePinnedToCore(benchTask, "nettime", 6144, nullptr, 2, nullptr, 1) != pdPASS) {
        out("nettime: could not start the bench task (heap)\r\n");
        heap_caps_free(g_m);
        g_m = nullptr;
        return;
    }
    if (xSemaphoreTake(g_done, pdMS_TO_TICKS(120000)) != pdTRUE) {
        out("nettime: timed out\r\n");     // the task may still use g_m: keep it
        return;
    }
    say(out, "  %-36s %7s %7s %7s\r\n", "variant", "min us", "med us", "max us");
    for (int i = 0; i < g_nrows; ++i) {
        const Row& r = g_m->rows[i];
        if (r.ran) {
            say(out, "  %-36s %7lu %7lu %7lu  %s\r\n", r.name,
                static_cast<unsigned long>(r.min_us), static_cast<unsigned long>(r.med_us),
                static_cast<unsigned long>(r.max_us), r.note);
        } else {
            say(out, "  %-36s    skip: %s\r\n", r.name, r.note);
        }
    }
    heap_caps_free(g_m);
    g_m = nullptr;
}

}  // namespace robot
