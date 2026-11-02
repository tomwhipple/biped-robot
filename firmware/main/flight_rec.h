// The flight recorder: one record per control tick, over UDP, to a collector
// on mira (tools/flight_collector.py). docs/flight-recorder.md is the manual;
// components/flightrec/record.h is the wire format.
//
// WHAT IS RECORDED, AND BY WHOM
// -----------------------------
//   ctrl (core 1), armed     every 50 Hz tick, from the SYNC READ the loop
//                            already does: no extra bus traffic at all.
//   housekeeping, benched    every bench smooth-stream tick (`pose ... s`,
//                            `pose ... B<L|R>` balance, `home`): one extra
//                            SYNC READ of the fitted servos per tick, AFTER
//                            the goal write; and, with nothing streaming, an
//                            idle sample at `rec idle <hz>` (default 10).
//   housekeeping             events: every CLI line executed and every line
//                            it printed, link ARM/home edges, mode handovers,
//                            balance state changes.
//
// NOTHING HERE BLOCKS A PRODUCER. Each producer encodes straight into a slot
// of its own lock-free single-producer ring (flightrec::Ring); a full ring
// drops the record and counts it. A sender task on core 0 drains both rings
// to the socket; no WiFi means drop and count. Every packet carries a
// sequence number from one per-boot counter, so the collector sees every loss.
//
// TIMING. The bench SYNC READ costs ~0.28 ms per fitted servo (measured,
// firmware-design.md section 4: 12 fitted ~3.4 ms). The housekeeping loop's
// cadence is "work, then vTaskDelay(10 ms)", so time spent here would stretch
// the bench pose/balance period one-for-one. It does not: recorderTick()
// returns the time it spent and link_task.cpp shortens that iteration's sleep
// by exactly that much, so the pose stream's tick phase is what it was
// without the recorder (to < 1 ms, the FreeRTOS tick).
#pragma once
#include <stdint.h>

#include "flightrec/record.h"
#include "imu/imu.h"
#include "obs/actuation.h"
#include "scsbus/bus.h"

namespace robot {
namespace flight {

// app_main, after startWifiLink(): NVS config, then the sender task.
void start();

// One relaxed load. Producers check it first and return when off.
bool enabled();

// -- ctrl (core 1) ----------------------------------------------------------
// Everything the loop holds for one tick, by pointer, so ctrl_task.cpp's hook
// is a struct fill and a call. Bus-indexed arrays are obs::kNumBusJoints long.
struct CtrlView {
    int64_t t0_us = 0;          // tick start, esp_timer
    uint32_t tick = 0;
    int64_t read0_us = 0, read1_us = 0;   // the SYNC READ's start / end
    uint8_t link_state = 0;     // the reported linkproto::LinkState
    bool torque_on = false;
    bool fallen = false;
    const imu::Sample* imu = nullptr;     // the sample the tick used
    int nfit = 0;
    const int* fit_bus = nullptr;         // fitted slot -> bus index
    const int* fit_pol = nullptr;         // fitted slot -> policy index or -1
    const scsbus::Feedback* fb = nullptr; // bus-indexed, last reply kept
    const bool* fb_ok = nullptr;          // bus-indexed, answered THIS tick
    int nwr = 0;                          // goals written this tick (0 = limp)
    const uint8_t* wr_id = nullptr;
    const int32_t* wr_steps = nullptr;
    const uint16_t* wr_speed = nullptr;   // nullptr = written without speeds
    const float* action = nullptr;        // raw policy action, nullptr on limp ticks
    const float* cmd = nullptr;           // ext_cmd as the policy saw it
    int ncmd = 0;
    float phase = 0.0f;
    uint16_t vbat_mv = 0;
    uint8_t batt = 0;                     // 0 ok, 1 landing, 2 limp
    const obs::Calibration* cal = nullptr;
};
void recordCtrl(const CtrlView& v);

// -- housekeeping (core 0) --------------------------------------------------
// Bind the housekeeping task as the house ring's one producer. Calls from any
// other task are dropped (and counted), never raced.
void bindHouseTask();

// One bench tick from the smooth streamer, or an idle sample (on/nominal/goal
// nullptr). Bus-indexed arrays. Does the SYNC READ itself; returns the
// microseconds it spent (the caller gives them back to its sleep).
struct BenchView {
    flightrec::Source source = flightrec::Source::kBenchIdle;
    int64_t t0_us = 0;            // the stream tick's start (idle: now)
    uint32_t work_us = 0;         // the stream tick's own cost
    bool traj_active = false;
    bool home_pending = false;
    uint8_t home_stage = 0;
    bool wrote = false;           // a SYNC WRITE went out this tick
    const bool* on = nullptr;     // joints in the stream
    const int32_t* nominal = nullptr;
    const int32_t* goal = nullptr;     // last goal written per joint
    const uint16_t* speed = nullptr;
    const bool* written = nullptr;     // written THIS tick
    bool bal_engaged = false;
    flightrec::Balance bal;
};
uint32_t recordBench(scsbus::Bus* bus, const BenchView& v);

// Housekeeping events (text may be nullptr). Trailing CR/LF are dropped.
void event(flightrec::Ev code, const char* text, int32_t a = 0, int32_t b = 0);
// The CLI sink wrapper's half: one printed line, split on CR/LF.
void outputText(const char* text);
// Called every housekeeping iteration: mode handover events.
void houseTick();
// The idle sample is due (rate limit + backoff), i.e. recordBench(idle) now.
bool idleDue();

// -- configuration / status (the `rec` CLI) ---------------------------------
struct Status {
    bool enabled;
    uint32_t ip;          // host order
    uint16_t port;
    int idle_hz;
    bool wifi;
    bool utc;
    uint32_t boot_id, seq, sent, drop_ring, drop_net, drop_task;
    uint32_t max_push_ctrl_us, max_push_house_us;
    uint32_t bench_read_us, bench_read_max_us, bench_reads, bench_backoffs;
    uint32_t ctrl_records, bench_records, idle_records, events;
};
void getStatus(Status& out);
// save == false applies in RAM only (armed: an NVS write stalls the tick).
bool configure(uint32_t ip, uint16_t port, bool save);
void setEnabled(bool on, bool save);
void setIdleHz(int hz, bool save);

}  // namespace flight
}  // namespace robot
