// Flight recorder runtime: the two producer rings, the sender task, NVS
// config. See flight_rec.h for the contract and docs/flight-recorder.md.
#include "flight_rec.h"

#include <string.h>

#include <atomic>
#include <new>

#include "esp_app_desc.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_sampler.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "shared.h"
#include "timesync.h"
#include "wifi_link.h"

namespace robot {
namespace flight {
namespace {

using flightrec::Ev;
using flightrec::Source;

static_assert(obs::kNumBusJoints <= flightrec::kMaxJoints,
              "the record carries at most kMaxJoints joints");

constexpr const char* kNs = "bimo";
constexpr const char* kKeyIp = "rec_ip";
constexpr const char* kKeyPort = "rec_port";
constexpr const char* kKeyOn = "rec_on";
constexpr const char* kKeyIdle = "rec_idle";

constexpr int kDefaultIdleHz = 10;
// A bench read slower than this means a servo is silent and the SYNC READ is
// sitting out its per-servo timeouts (3 ms each): stop reading for a second
// rather than stall the pose stream.
constexpr uint32_t kBenchReadSlowUs = 8000;
constexpr int64_t kBackoffUs = 1000000;
constexpr int kSenderPeriodMs = 4;
constexpr int64_t kHelloPeriodUs = 1000000;

// Rings: 16 x 578 B each. ctrl makes 50/s, the sender drains every 4 ms, so
// 16 slots is 320 ms of WiFi/scheduler stall before ctrl drops anything.
flightrec::Ring<16> g_ring_ctrl;
flightrec::Ring<16> g_ring_house;

std::atomic<bool> g_enabled{false};
std::atomic<uint32_t> g_ip{0};          // host order
std::atomic<uint16_t> g_port{flightrec::kDefaultPort};
std::atomic<int> g_idle_hz{kDefaultIdleHz};
uint32_t g_boot_id = 0;
TaskHandle_t g_house_task = nullptr;

std::atomic<uint32_t> g_seq{0};
std::atomic<uint32_t> g_sent{0};
std::atomic<uint32_t> g_drop_ring{0};
std::atomic<uint32_t> g_drop_net{0};
std::atomic<uint32_t> g_drop_task{0};
std::atomic<uint32_t> g_max_push_ctrl{0};
std::atomic<uint32_t> g_max_push_house{0};
std::atomic<uint32_t> g_bench_read_us{0};
std::atomic<uint32_t> g_bench_read_max{0};
std::atomic<uint32_t> g_bench_reads{0};
std::atomic<uint32_t> g_bench_backoffs{0};
std::atomic<uint32_t> g_ctrl_records{0};
std::atomic<uint32_t> g_bench_records{0};
std::atomic<uint32_t> g_idle_records{0};
std::atomic<uint32_t> g_events{0};

// Housekeeping-only state.
int64_t g_backoff_until = 0;
int64_t g_next_idle = 0;
uint32_t g_house_ticks = 0;
uint8_t g_last_bal_state = 0;
int g_last_mode = -1;
// ctrl-only state.
uint8_t g_last_link_state = 0xFF;
// The records being built, one per producer, in static storage rather than
// on the producers' stacks (housekeeping has 4 KB, and a Tick is ~0.6 KB).
// Each is touched by its one producer task only; placement new resets it.
flightrec::Tick g_ctrl_tick;
flightrec::Tick g_house_tick;
scsbus::Feedback g_house_fb[obs::kNumBusJoints];

constexpr int kSenderStackWords = 3072;
StackType_t g_sender_stack[kSenderStackWords];
StaticTask_t g_sender_tcb;

flightrec::Header header(int64_t t_boot_us) {
    flightrec::Header h;
    h.boot_id = g_boot_id;
    h.seq = g_seq.fetch_add(1, std::memory_order_relaxed);
    h.t_boot_us = static_cast<uint64_t>(t_boot_us);
    h.t_utc_us = timeFromBootUs(t_boot_us);
    h.drop_ring = g_drop_ring.load(std::memory_order_relaxed);
    h.drop_net = g_drop_net.load(std::memory_order_relaxed);
    return h;
}

void noteMax(std::atomic<uint32_t>& m, uint32_t v) {
    if (v > m.load(std::memory_order_relaxed)) m.store(v, std::memory_order_relaxed);
}

template <size_t N>
bool pushTick(flightrec::Ring<N>& ring, const flightrec::Tick& t, int64_t t0) {
    // The seq is taken even when the ring is full: the hole it leaves is how
    // the collector learns of the drop.
    const flightrec::Header h = header(t0);
    uint8_t* slot = ring.claim();
    if (!slot) {
        g_drop_ring.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    ring.publish(flightrec::encodeTick(slot, flightrec::kMaxPacket, h, t));
    return true;
}

template <size_t N>
void pushEvent(flightrec::Ring<N>& ring, const flightrec::Event& e) {
    const flightrec::Header h = header(esp_timer_get_time());
    uint8_t* slot = ring.claim();
    if (!slot) {
        g_drop_ring.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    ring.publish(flightrec::encodeEvent(slot, flightrec::kMaxPacket, h, e));
    g_events.fetch_add(1, std::memory_order_relaxed);
}

bool onHouseTask() {
    if (g_house_task != nullptr && xTaskGetCurrentTaskHandle() == g_house_task) {
        return true;
    }
    g_drop_task.fetch_add(1, std::memory_order_relaxed);
    return false;
}

void fillImu(flightrec::Tick& t, const imu::Sample& s, bool valid, int64_t t0) {
    if (!valid) return;
    t.flags |= flightrec::kTfImuValid;
    t.imu_age_us = static_cast<int32_t>(t0 - static_cast<int64_t>(s.t_us));
    memcpy(t.accel, s.accel, sizeof t.accel);
    memcpy(t.gyro, s.gyro, sizeof t.gyro);
    memcpy(t.quat, s.quat, sizeof t.quat);
    memcpy(t.up, s.up, sizeof t.up);
    flightrec::rollPitchFromQuat(t.quat, &t.roll, &t.pitch);
}

void fillFeedback(flightrec::Joint& j, const scsbus::Feedback& f) {
    j.status = f.status;
    j.temp_c = f.temperature_c;
    j.voltage_dv = f.voltage_dv;
    j.moving = f.moving;
    j.pos = flightrec::sat16(f.position);
    j.speed = flightrec::sat16(f.speed);
    j.load = flightrec::sat16(f.load);
    j.current = flightrec::sat16(f.current);
}

// -- NVS ---------------------------------------------------------------------
void loadConfig() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return;
    uint32_t ip = 0;
    uint16_t port = 0;
    uint8_t on = 0, idle = 0;
    if (nvs_get_u32(h, kKeyIp, &ip) == ESP_OK) g_ip.store(ip);
    if (nvs_get_u16(h, kKeyPort, &port) == ESP_OK && port != 0) g_port.store(port);
    if (nvs_get_u8(h, kKeyIdle, &idle) == ESP_OK) g_idle_hz.store(idle);
    if (nvs_get_u8(h, kKeyOn, &on) == ESP_OK) g_enabled.store(on != 0 && g_ip.load() != 0);
    nvs_close(h);
}

void saveConfig() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, kKeyIp, g_ip.load());
    nvs_set_u16(h, kKeyPort, g_port.load());
    nvs_set_u8(h, kKeyOn, g_enabled.load() ? 1 : 0);
    nvs_set_u8(h, kKeyIdle, static_cast<uint8_t>(g_idle_hz.load()));
    nvs_commit(h);
    nvs_close(h);
}

// -- the sender ----------------------------------------------------------------
size_t buildHello(uint8_t* out, size_t cap) {
    flightrec::Hello m;
    const esp_app_desc_t* d = esp_app_get_description();
    strncpy(m.fw, d->version, sizeof m.fw - 1);
    TimeStatus ts;
    timeGetStatus(ts);
    m.utc_synced = ts.synced ? 1 : 0;
    m.ntp_syncs = ts.syncs;
    m.enabled = g_enabled.load() ? 1 : 0;
    m.idle_hz = static_cast<uint8_t>(g_idle_hz.load());
    m.cal_ok = g_cal_from_nvs.load() ? 1 : 0;
    m.produced = g_seq.load();
    m.sent = g_sent.load();
    m.drop_ring = g_drop_ring.load();
    m.drop_net = g_drop_net.load();
    m.max_push_ctrl_us = g_max_push_ctrl.load();
    m.max_push_house_us = g_max_push_house.load();
    m.bench_read_us = g_bench_read_us.load();
    m.ctrl_ticks = g_telemetry.ticks.load();
    m.ctrl_late = g_telemetry.overruns.load();
    m.ctrl_worst_us = g_telemetry.worst_tick_us.load();
    m.us_read = g_telemetry.us_read.load();
    m.us_imu = g_telemetry.us_imu.load();
    m.us_obs = g_telemetry.us_obs.load();
    m.us_net = g_telemetry.us_net.load();
    m.us_write = g_telemetry.us_write.load();
    m.us_other = g_telemetry.us_other.load();
    m.uptime_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    // A torn read against a `cal` write is a one-hello glitch; the collector
    // keeps the latest hello per session and says so.
    const obs::Calibration& cal = calibration();
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        m.id[b] = obs::kBusServoId[b];
        m.fitted[b] = cal.fitted[b];
        m.dir[b] = cal.dir[b];
        m.zero[b] = flightrec::sat16(cal.zero_steps[b]);
    }
    return flightrec::encodeHello(out, cap, header(esp_timer_get_time()), m);
}

bool sendOne(int sock, const uint8_t* p, size_t len) {
    const uint32_t ip = g_ip.load();
    if (sock < 0 || ip == 0 || !wifiConnected()) return false;
    sockaddr_in to = {};
    to.sin_family = AF_INET;
    to.sin_addr.s_addr = lwip_htonl(ip);
    to.sin_port = lwip_htons(g_port.load());
    return lwip_sendto(sock, p, len, MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&to),
                       sizeof to) == static_cast<int>(len);
}

template <size_t N>
void drain(flightrec::Ring<N>& ring, int sock) {
    size_t len = 0;
    while (const uint8_t* p = ring.peek(&len)) {
        if (sendOne(sock, p, len)) {
            g_sent.fetch_add(1, std::memory_order_relaxed);
        } else {
            g_drop_net.fetch_add(1, std::memory_order_relaxed);
        }
        ring.pop();
    }
}

void senderTask(void*) {
    int sock = -1;
    int64_t next_hello = 0;
    static uint8_t hello[flightrec::kMaxPacket];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(kSenderPeriodMs));
        if (sock < 0 && wifiConnected()) {
            sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        }
        // Drained even while disabled: a record produced just before `rec off`
        // still goes out (or is counted), and the rings start the next
        // recording empty.
        drain(g_ring_ctrl, sock);
        drain(g_ring_house, sock);
        const int64_t now = esp_timer_get_time();
        if (g_enabled.load() && now >= next_hello) {
            next_hello = now + kHelloPeriodUs;
            const size_t n = buildHello(hello, sizeof hello);
            if (n && sendOne(sock, hello, n)) {
                g_sent.fetch_add(1, std::memory_order_relaxed);
            } else {
                g_drop_net.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

}  // namespace

bool enabled() { return g_enabled.load(std::memory_order_relaxed); }

void start() {
    g_boot_id = esp_random();
    if (g_boot_id == 0) g_boot_id = 1;
    loadConfig();
    // Below the IMU sampler (4) and the link tasks (5), above housekeeping (2):
    // the rings drain promptly while the CLI is busy, and nothing that keeps
    // the robot safe ever waits on a send.
    xTaskCreateStaticPinnedToCore(senderTask, "flightrec", kSenderStackWords, nullptr,
                                  3, g_sender_stack, &g_sender_tcb, 0);
}

// -- ctrl (core 1) -------------------------------------------------------------
void recordCtrl(const CtrlView& v) {
    if (!enabled()) return;
    const int64_t t_in = esp_timer_get_time();
    flightrec::Tick& t = *new (&g_ctrl_tick) flightrec::Tick();
    t.source = Source::kCtrl;
    t.link_state = v.link_state;
    t.tick = v.tick;
    t.work_us = static_cast<uint32_t>(t_in - v.t0_us);
    t.fb_off_us = static_cast<int32_t>((v.read0_us + v.read1_us) / 2 - v.t0_us);
    t.fb_read_us = static_cast<uint32_t>(v.read1_us - v.read0_us);
    if (v.torque_on) t.flags |= flightrec::kTfTorque;
    if (v.fallen) t.flags |= flightrec::kTfFallen;
    if (v.nwr > 0) t.flags |= flightrec::kTfWrote;
    if (v.nfit > 0) t.flags |= flightrec::kTfFbRead;
    if (v.imu) fillImu(t, *v.imu, v.imu->valid, v.t0_us);
    for (int i = 0; i < v.ncmd && i < flightrec::kNumCmd; ++i) t.ctrl.cmd[i] = v.cmd[i];
    t.ctrl.phase = v.phase;
    t.ctrl.vbat_mv = v.vbat_mv;
    t.ctrl.batt = v.batt;

    // The policy's own target for each of its joints: the raw action through
    // the trained action map, before the shaper and the takeover ramp.
    int32_t nominal[obs::kNumBusJoints];
    bool nom_ok[obs::kNumBusJoints] = {};
    if (v.action && v.cal) {
        float ang[obs::kNumJoints];
        obs::actionToAngles(v.action, ang);
        for (int j = 0; j < obs::kNumJoints; ++j) {
            const int b = obs::policyToBus(j);
            nominal[b] = obs::angleToSteps(j, ang[j], *v.cal);
            nom_ok[b] = true;
        }
    }
    // bus index -> its slot in this tick's SYNC WRITE, -1 = not written
    int wr_of_bus[obs::kNumBusJoints];
    for (int b = 0; b < obs::kNumBusJoints; ++b) wr_of_bus[b] = -1;
    for (int w = 0; w < v.nwr; ++w) {
        const int b = obs::busIndexOfId(v.wr_id[w]);
        if (b >= 0) wr_of_bus[b] = w;
    }
    int n = 0;
    for (int k = 0; k < v.nfit && n < flightrec::kMaxJoints; ++k) {
        const int b = v.fit_bus[k];
        flightrec::Joint& j = t.joint[n++];
        j.bus = static_cast<uint8_t>(b);
        if (v.fit_pol[k] >= 0) j.flags |= flightrec::kJfDriven;
        if (v.fb_ok[b]) j.flags |= flightrec::kJfFbOk;
        fillFeedback(j, v.fb[b]);
        const int w = wr_of_bus[b];
        if (w >= 0) {
            j.goal = flightrec::sat16(v.wr_steps[w]);
            j.goal_speed = v.wr_speed ? v.wr_speed[w] : 0;
            j.flags |= flightrec::kJfGoalValid | flightrec::kJfWritten;
        }
        if (nom_ok[b]) {
            j.nominal = flightrec::sat16(nominal[b]);
            j.flags |= flightrec::kJfNomValid;
        } else if (j.flags & flightrec::kJfGoalValid) {
            // a HELD servo: its target is the hold the loop wrote
            j.nominal = j.goal;
            j.flags |= flightrec::kJfNomValid;
        }
    }
    t.njoint = static_cast<uint8_t>(n);
    if (pushTick(g_ring_ctrl, t, v.t0_us)) g_ctrl_records.fetch_add(1, std::memory_order_relaxed);
    if (v.link_state != g_last_link_state) {
        flightrec::Event e;
        e.code = Ev::kLinkState;
        e.src = 1;
        e.a = g_last_link_state == 0xFF ? -1 : g_last_link_state;
        e.b = v.link_state;
        g_last_link_state = v.link_state;
        pushEvent(g_ring_ctrl, e);
    }
    noteMax(g_max_push_ctrl, static_cast<uint32_t>(esp_timer_get_time() - t_in));
}

// -- housekeeping (core 0) -----------------------------------------------------
void bindHouseTask() { g_house_task = xTaskGetCurrentTaskHandle(); }

void event(Ev code, const char* text, int32_t a, int32_t b) {
    if (!enabled() || !onHouseTask()) return;
    const int64_t t_in = esp_timer_get_time();
    // static, like g_house_tick: this runs deep inside CLI commands (via the
    // sink), on a 4 KB stack. Housekeeping only (checked above).
    static flightrec::Event e;
    new (&e) flightrec::Event();
    e.code = code;
    e.a = a;
    e.b = b;
    flightrec::setText(e, text);
    pushEvent(g_ring_house, e);
    noteMax(g_max_push_house, static_cast<uint32_t>(esp_timer_get_time() - t_in));
}

void outputText(const char* text, int32_t chan) {
    if (!enabled() || text == nullptr || !onHouseTask()) return;
    // One event per line; empty lines are dropped.
    const char* p = text;
    static char line[flightrec::kMaxText + 1];   // housekeeping only, 4 KB stack
    while (*p) {
        size_t n = strcspn(p, "\r\n");
        if (n > 0) {
            const size_t m = n < sizeof line - 1 ? n : sizeof line - 1;
            memcpy(line, p, m);
            line[m] = 0;
            event(Ev::kOut, line, chan);
        }
        p += n;
        while (*p == '\r' || *p == '\n') ++p;
    }
}

void houseTick() {
    if (!enabled()) {
        g_last_mode = -1;
        return;
    }
    const int mode = (g_mode_request.load() == Mode::kRun ? 1 : 0) |
                     (g_ctrl_owns_bus.load() ? 2 : 0);
    if (mode != g_last_mode) {
        g_last_mode = mode;
        event(Ev::kMode, (mode & 1) ? "run" : "bench", mode, g_arm_result.load());
    }
}

bool idleDue() {
    const int hz = g_idle_hz.load();
    if (!enabled() || hz <= 0) return false;
    const int64_t now = esp_timer_get_time();
    if (now < g_next_idle) return false;
    g_next_idle = now + 1000000 / hz;
    return true;
}

uint32_t recordBench(scsbus::Bus* bus, const BenchView& v) {
    if (!enabled() || !onHouseTask()) return 0;
    const int64_t t_in = esp_timer_get_time();
    flightrec::Tick& t = *new (&g_house_tick) flightrec::Tick();
    t.source = v.source;
    t.link_state = static_cast<uint8_t>(linkproto::LinkState::kBench);
    t.tick = ++g_house_ticks;
    t.work_us = v.work_us;
    if (v.traj_active) t.flags |= flightrec::kTfTraj;
    if (v.home_pending) t.flags |= flightrec::kTfHome;
    if (v.bal_engaged) t.flags |= flightrec::kTfBal;
    if (v.wrote) t.flags |= flightrec::kTfWrote;
    t.ctrl.home_stage = v.home_stage;
    t.bal = v.bal;
    t.ctrl.vbat_mv = g_telemetry.vbat_mv.load();
    {
        imu::Sample s{};
        uint32_t age_ms = 0;
        const bool have = peekImu(s, &age_ms);
        fillImu(t, s, have && s.valid, v.t0_us);
    }

    // The fitted set from the live calibration (only the CLI writes it, on
    // this very task).
    int fit_bus[obs::kNumBusJoints];
    uint8_t fit_id[obs::kNumBusJoints];
    const int nfit = fittedBusJoints(calibration(), fit_bus, fit_id);
    scsbus::Feedback* fb = g_house_fb;
    bool ok[obs::kNumBusJoints] = {};
    uint32_t spent = 0;
    const int64_t now = esp_timer_get_time();
    if (bus && nfit > 0 && now >= g_backoff_until) {
        const int64_t r0 = esp_timer_get_time();
        bus->syncReadFeedback(fit_id, static_cast<size_t>(nfit), fb, ok);
        const int64_t r1 = esp_timer_get_time();
        spent = static_cast<uint32_t>(r1 - r0);
        t.flags |= flightrec::kTfFbRead;
        t.fb_off_us = static_cast<int32_t>((r0 + r1) / 2 - v.t0_us);
        t.fb_read_us = spent;
        g_bench_read_us.store(spent);
        noteMax(g_bench_read_max, spent);
        g_bench_reads.fetch_add(1, std::memory_order_relaxed);
        if (spent > kBenchReadSlowUs) {
            g_backoff_until = r1 + kBackoffUs;
            g_bench_backoffs.fetch_add(1, std::memory_order_relaxed);
            event(Ev::kFbBackoff, "bench feedback read too slow (a servo silent?): "
                  "reads paused 1 s", static_cast<int32_t>(spent));
        }
    }
    int n = 0;
    for (int k = 0; k < nfit && n < flightrec::kMaxJoints; ++k) {
        const int b = fit_bus[k];
        flightrec::Joint& j = t.joint[n++];
        j.bus = static_cast<uint8_t>(b);
        if (ok[k]) {
            j.flags |= flightrec::kJfFbOk;
            fillFeedback(j, fb[k]);
        }
        if (v.on && v.on[b]) {
            j.flags |= flightrec::kJfDriven;
            if (v.nominal) {
                j.nominal = flightrec::sat16(v.nominal[b]);
                j.flags |= flightrec::kJfNomValid;
            }
            if (v.goal) {
                j.goal = flightrec::sat16(v.goal[b]);
                j.flags |= flightrec::kJfGoalValid;
            }
            if (v.speed) j.goal_speed = v.speed[b];
            if (v.written && v.written[b]) j.flags |= flightrec::kJfWritten;
        }
    }
    t.njoint = static_cast<uint8_t>(n);
    if (v.bal.state != g_last_bal_state) {
        event(Ev::kBalState, nullptr, g_last_bal_state, v.bal.state);
        g_last_bal_state = v.bal.state;
    }
    if (pushTick(g_ring_house, t, v.t0_us)) {
        (v.source == Source::kBenchIdle ? g_idle_records : g_bench_records)
            .fetch_add(1, std::memory_order_relaxed);
    }
    const uint32_t total = static_cast<uint32_t>(esp_timer_get_time() - t_in);
    noteMax(g_max_push_house, total - spent);
    return total;
}

// -- configuration -------------------------------------------------------------
void getStatus(Status& out) {
    out.enabled = g_enabled.load();
    out.ip = g_ip.load();
    out.port = g_port.load();
    out.idle_hz = g_idle_hz.load();
    out.wifi = wifiConnected();
    out.utc = timeNowUs() != 0;
    out.boot_id = g_boot_id;
    out.seq = g_seq.load();
    out.sent = g_sent.load();
    out.drop_ring = g_drop_ring.load();
    out.drop_net = g_drop_net.load();
    out.drop_task = g_drop_task.load();
    out.max_push_ctrl_us = g_max_push_ctrl.load();
    out.max_push_house_us = g_max_push_house.load();
    out.bench_read_us = g_bench_read_us.load();
    out.bench_read_max_us = g_bench_read_max.load();
    out.bench_reads = g_bench_reads.load();
    out.bench_backoffs = g_bench_backoffs.load();
    out.ctrl_records = g_ctrl_records.load();
    out.bench_records = g_bench_records.load();
    out.idle_records = g_idle_records.load();
    out.events = g_events.load();
}

bool configure(uint32_t ip, uint16_t port, bool save) {
    if (ip == 0) return false;
    g_ip.store(ip);
    g_port.store(port ? port : flightrec::kDefaultPort);
    g_enabled.store(true);
    if (save) saveConfig();
    return true;
}

void setEnabled(bool on, bool save) {
    g_enabled.store(on && g_ip.load() != 0);
    if (save) saveConfig();
}

void setIdleHz(int hz, bool save) {
    if (hz < 0) hz = 0;
    if (hz > 50) hz = 50;
    g_idle_hz.store(hz);
    if (save) saveConfig();
}

}  // namespace flight
}  // namespace robot
