// The flight recorder's wire format -- pure, no I/O, host-testable.
//
// One UDP datagram = one packet = one record. Every hardware run sends, per
// control tick, what the robot was TOLD to do (the nominal trajectory target),
// what it WROTE (the goal after balance correction / shaping / ramps), and what
// the servos and the IMU SAID back -- so a session can be laid against the sim
// and the discrepancy is a subtraction (docs/flight-recorder.md).
//
// All multi-byte fields are LITTLE-endian, floats are IEEE-754 binary32. The
// layout is spelled out field by field in docs/flight-recorder.md and mirrored
// in tools/flightrec.py; the host test (firmware/host/test_flightrec.cpp)
// checks this encoder byte-for-byte against vectors generated from the Python
// side (firmware/host/vectors/flightrec_vectors.h), so the two cannot drift.
//
// Loss is detectable: every packet carries one sequence number from a single
// per-boot counter, so a hole is a lost packet wherever it was lost, and the
// header's two drop counters say how many of those the robot itself dropped
// (ring full / send failed) -- the rest went missing on the network.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <atomic>

namespace flightrec {

constexpr uint8_t kMagic0 = 'F';
constexpr uint8_t kMagic1 = 'R';
constexpr uint8_t kVersion = 1;
// The collector's default UDP port (4210 commands, 4211 telemetry, 4212 sim
// pose: the next free one).
constexpr uint16_t kDefaultPort = 4220;

enum class Type : uint8_t {
    kTick = 1,     // one control tick (ctrl loop, bench pose stream, idle)
    kEvent = 2,    // a discrete event: a command line, a CLI reply, an edge
    kHello = 3,    // ~1 Hz: firmware, calibration, recorder counters
};

// Who produced a TICK.
enum class Source : uint8_t {
    kCtrl = 1,        // the 50 Hz control loop (core 1), armed
    kBenchPose = 2,   // the bench smooth-pose streamer (poseTick: pose/home/balance)
    kBenchIdle = 3,   // benched, nothing streaming: the low-rate idle sample
};

// TICK flags.
constexpr uint8_t kTfTorque = 1u << 0;     // ctrl: torque engaged this tick
constexpr uint8_t kTfTraj = 1u << 1;       // bench: a smooth stream is active
constexpr uint8_t kTfHome = 1u << 2;       // bench: a home is in progress
constexpr uint8_t kTfBal = 1u << 3;        // bench: balance engaged (any state but OFF)
constexpr uint8_t kTfImuValid = 1u << 4;   // the IMU block holds a real sample
constexpr uint8_t kTfFbRead = 1u << 5;     // a servo feedback read ran this tick
constexpr uint8_t kTfWrote = 1u << 6;      // goals went on the wire this tick
constexpr uint8_t kTfFallen = 1u << 7;     // ctrl: the fall latch is set

// Per-joint flags.
constexpr uint8_t kJfFbOk = 1u << 0;       // the servo answered THIS tick's read
constexpr uint8_t kJfNomValid = 1u << 1;   // `nominal` means something
constexpr uint8_t kJfGoalValid = 1u << 2;  // `goal` means something
constexpr uint8_t kJfWritten = 1u << 3;    // `goal` went on the wire this tick
constexpr uint8_t kJfDriven = 1u << 4;     // part of the stream / policy-driven

// Event codes.
enum class Ev : uint8_t {
    kCmd = 1,         // a CLI line as executed (text = the line, args and all)
    kOut = 2,         // one line the CLI printed (balance ON/ARMED/ABORT/landed, refusals, ...)
    kArmEdge = 3,     // wireless ARM edge consumed: a = wanted level
    kHomeEdge = 4,    // wireless "reset the servos" edge consumed
    kRec = 5,         // recorder on/off/re-pointed (text says)
    kMode = 6,        // bench/run handover: a = Mode, b = linkproto::ArmResult
    kBalState = 7,    // balance state change: a = old, b = new (balance::State)
    kLinkState = 8,   // ctrl's reported LinkState changed: a = old, b = new
    kFbBackoff = 9,   // bench feedback read too slow (a servo silent): a = us it took
};

constexpr int kMaxJoints = 17;      // == obs::kNumBusJoints (static_assert in main)
constexpr int kMaxText = 180;       // event text, bytes (a CLI line is <= 128)
constexpr int kNumCmd = 7;          // ext_cmd: vx vy wz crouch lift foot_dx foot_dz

constexpr size_t kHeaderLen = 36;
constexpr size_t kTickFixedLen = 164;
constexpr size_t kJointLen = 20;
constexpr size_t kTickMaxLen = kHeaderLen + kTickFixedLen + kJointLen * kMaxJoints;
constexpr size_t kEventFixedLen = 12;
constexpr size_t kEventMaxLen = kHeaderLen + kEventFixedLen + kMaxText;
constexpr size_t kHelloFixedLen = 116;   // fw[32], 4 flags, 19 u32 counters, 4 (count + pad)
constexpr size_t kHelloLen = kHeaderLen + kHelloFixedLen + 5 * kMaxJoints;
constexpr size_t kMaxPacket = 576;
static_assert(kTickMaxLen <= kMaxPacket, "a full tick must fit one slot");
static_assert(kEventMaxLen <= kMaxPacket, "an event must fit one slot");
static_assert(kHelloLen <= kMaxPacket, "a hello must fit one slot");

struct Header {
    Type type = Type::kTick;
    uint32_t boot_id = 0;      // random per boot: splits sessions across a reset
    uint32_t seq = 0;          // one counter for every packet this boot
    uint64_t t_utc_us = 0;     // NTP epoch us at the tick start; 0 = not synced
    uint64_t t_boot_us = 0;    // esp_timer us at the tick start (always valid)
    uint32_t drop_ring = 0;    // cumulative: packets the robot dropped, ring full
    uint32_t drop_net = 0;     // cumulative: packets the robot dropped, send failed/no WiFi
};

struct Joint {
    uint8_t bus = 0;           // bus index (obs/bus_map.h kBusJointNames)
    uint8_t flags = 0;         // kJf*
    uint8_t status = 0;        // servo error bits (reg 65 | reply ERR byte)
    uint8_t temp_c = 0;
    uint8_t voltage_dv = 0;    // 0.1 V
    uint8_t moving = 0;
    int16_t nominal = 0;       // steps: the trajectory/policy target before corrections
    int16_t goal = 0;          // steps: the goal written (incl. balance / shaper / ramp)
    uint16_t goal_speed = 0;   // steps/s written with it (0 = servo max)
    int16_t pos = 0;           // steps: PRESENT position
    int16_t speed = 0;         // steps/s, present
    int16_t load = 0;          // 0.1 % duty, signed
    int16_t current = 0;       // 6.5 mA units
};

struct Balance {
    uint8_t state = 0;         // balance::State
    uint8_t side = 0;          // 'L' / 'R' stance, 0 = none
    int8_t ankle_bus = -1;     // bus index of the corrected ankle roll
    int8_t hip_bus = -1;       // ... and hip roll
    float ref = 0, err = 0, integral = 0, u = 0, rate = 0;   // rad, rad, -, rad, rad/s
    float roll = 0;            // the torso roll the PI last used, rad
    float ankle_rad = 0, hip_rad = 0;    // correction added to the stance joints
    int16_t ankle_ticks = 0, hip_ticks = 0;   // same, in servo ticks (dir applied)
    uint32_t clamps = 0;       // writes the envelope clamp cut short (cumulative)
};

struct Ctrl {
    float cmd[kNumCmd] = {0, 0, 0, 0, 0, 0, 0};   // ext_cmd as the policy saw it
    float phase = 0;           // gait clock phase
    uint16_t vbat_mv = 0;
    uint8_t batt = 0;          // 0 ok, 1 land (crouch ramp), 2 limp
    uint8_t home_stage = 0;    // bench: 0 slew, 1..4 hip play take-up
};

struct Tick {
    Source source = Source::kCtrl;
    uint8_t flags = 0;         // kTf*
    uint8_t link_state = 0;    // linkproto::LinkState as reported
    uint8_t njoint = 0;
    uint32_t tick = 0;         // the producer's own tick counter
    uint32_t work_us = 0;      // the producer's tick work so far (ctrl: tick body)
    int32_t fb_off_us = 0;     // mid-point of the servo read, relative to t_boot_us
    uint32_t fb_read_us = 0;   // how long that read took
    int32_t imu_age_us = 0;    // t_boot_us minus the IMU sample's own time
    float accel[3] = {0, 0, 0};    // m/s^2, body frame (mount applied)
    float gyro[3] = {0, 0, 0};     // rad/s, body frame (ctrl: tick-averaged)
    float quat[4] = {1, 0, 0, 0};  // body->world, w x y z (fusion)
    float up[3] = {0, 0, 1};       // the policy's up vector
    float roll = 0, pitch = 0;     // rad, ZYX Euler from quat
    Balance bal;
    Ctrl ctrl;
    Joint joint[kMaxJoints];
};

struct Event {
    Ev code = Ev::kCmd;
    uint8_t src = 0;           // 0 = housekeeping (core 0), 1 = ctrl (core 1)
    int32_t a = 0, b = 0;
    uint16_t len = 0;
    char text[kMaxText] = {};
};

struct Hello {
    char fw[32] = {};          // esp_app_desc version (git describe)
    uint8_t utc_synced = 0;
    uint8_t enabled = 0;
    uint8_t idle_hz = 0;
    uint8_t cal_ok = 0;        // calibration came from NVS
    uint32_t produced = 0;     // packets produced (seq handed out)
    uint32_t sent = 0;
    uint32_t drop_ring = 0, drop_net = 0;
    uint32_t max_push_ctrl_us = 0;   // worst encode+push cost on the ctrl tick
    uint32_t max_push_house_us = 0;
    uint32_t bench_read_us = 0;      // last bench feedback read
    uint32_t ctrl_ticks = 0, ctrl_late = 0, ctrl_worst_us = 0;
    uint32_t us_read = 0, us_imu = 0, us_obs = 0, us_net = 0, us_write = 0, us_other = 0;
    uint32_t uptime_ms = 0;
    uint32_t ntp_syncs = 0;
    uint32_t reserved = 0;
    // per bus joint, kMaxJoints rows: servo id, fitted, dir, zero steps
    uint8_t id[kMaxJoints] = {};
    uint8_t fitted[kMaxJoints] = {};
    int8_t dir[kMaxJoints] = {};
    int16_t zero[kMaxJoints] = {};
};

// Encoders: write into `out` (capacity `cap`) and return the length, 0 if it
// does not fit. h.type is set by each encoder.
size_t encodeTick(uint8_t* out, size_t cap, Header h, const Tick& t);
size_t encodeEvent(uint8_t* out, size_t cap, Header h, const Event& e);
size_t encodeHello(uint8_t* out, size_t cap, Header h, const Hello& m);

// Decoders, for the host tests (the collector decodes in Python). false on a
// wrong magic/version/type or a short/inconsistent length.
bool decodeHeader(const uint8_t* in, size_t len, Header& h);
bool decodeTick(const uint8_t* in, size_t len, Header& h, Tick& t);
bool decodeEvent(const uint8_t* in, size_t len, Header& h, Event& e);
bool decodeHello(const uint8_t* in, size_t len, Header& h, Hello& m);

// Event text: copy at most kMaxText bytes of `s`, dropping trailing CR/LF.
void setText(Event& e, const char* s);

// Roll / pitch (rad) from a body->world quaternion (w, x, y, z), ZYX Euler.
void rollPitchFromQuat(const float q[4], float* roll, float* pitch);

// Saturating float/int -> int16 for the step fields.
int16_t sat16(int32_t v);

// Single-producer single-consumer ring of fixed-size packet slots. The
// producer encodes straight into the slot it claimed (no second copy) and
// publishes it; the consumer reads the oldest published slot and releases it.
// Lock-free: one atomic index per side, each written by one side only, so a
// producer on core 1 never waits on the consumer on core 0. Full = the
// producer DROPS (and the caller counts it) -- it never blocks.
template <size_t N>
class Ring {
  public:
    static_assert(N >= 2 && (N & (N - 1)) == 0, "N must be a power of two");
    // Producer: a slot to encode into, or nullptr if the ring is full.
    uint8_t* claim() {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= N) return nullptr;
        return slot_[h & (N - 1)].data;
    }
    // Producer: publish the claimed slot with `len` bytes (0 = abandon it).
    void publish(size_t len) {
        if (len == 0) return;
        const uint32_t h = head_.load(std::memory_order_relaxed);
        slot_[h & (N - 1)].len = static_cast<uint16_t>(len);
        head_.store(h + 1, std::memory_order_release);
    }
    // Consumer: the oldest published packet, or nullptr when empty.
    const uint8_t* peek(size_t* len) const {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);
        if (t == h) return nullptr;
        *len = slot_[t & (N - 1)].len;
        return slot_[t & (N - 1)].data;
    }
    // Consumer: release the slot peek() returned.
    void pop() {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        tail_.store(t + 1, std::memory_order_release);
    }
    size_t size() const {
        return head_.load(std::memory_order_acquire) -
               tail_.load(std::memory_order_acquire);
    }

  private:
    struct Slot {
        uint8_t data[kMaxPacket];
        uint16_t len;
    };
    Slot slot_[N] = {};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
};

}  // namespace flightrec
