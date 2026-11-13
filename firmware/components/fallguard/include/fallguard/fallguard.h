// Fall detection and pick-up recovery: ONE module for both control paths.
//
// Tom 2026-11-01: "we need to be working towards a full policy driven system.
// I don't like the distinction between the two" (the scripted bench path and
// the armed policy loop). Until today each path carried its own detector with
// its own numbers (ctrl_task: up_z < linkproto::kFallUpZ = 0.4, i.e. 66 deg,
// for 10 ticks; cli.cpp: 30 deg for 100 ms). Now both run this code, with the
// constants below, and the recovery (pick-up -> home) is the same whichever
// path the robot fell in.
//
//   detect    torso tilt past the motion's limit, continuously, for its
//             debounce -> Event::kFell, latched. The caller stops motion and
//             releases torque (limp): a downed robot must not grind its
//             servos into the floor, and nothing scripted or learned is
//             meant to recover from past these lines.
//   recover   while latched: within kPickupTiltDeg of upright AND
//             |gyro| < kPickupGyroRadS, continuously for kPickupHoldMs
//             (25 deg, 1.0 rad/s, 1 s) -> Event::kPickedUp (upright for a
//             moment mid-tumble does not count). The caller homes the servos (slow,
//             ends released) and the latch clears with clear(). An operator
//             can turn that off (autohome off); the latch then waits for a
//             typed `home`.
//
// Why the pick-up rule is loose (Tom 2026-11-11: "once the robot returns to
// upright for 1 sec, it automatically homes"): the flight record of three bench
// falls showed a robot held in the hand is never still to 0.35 rad/s for 2 s
// (|gyro| 1-3 rad/s at 0-15 deg), and one set down limp at 21.7 deg, perfectly
// still, never met the old 10 deg line, so it never homed. 25 deg stays below
// the 30 deg bench fall line, so a robot lying on its side never qualifies.
//
// THE LIMITS (one table, documented once):
//
//   Motion::kPolicy   cos 45 deg = 0.7071, 200 ms   the armed policy loop
//   Motion::kBench    cos 30 deg = 0.8660, 100 ms   scripted bench motion and
//                                                   the benched idle robot
//
// Why 45 deg under the policy (Tom 2026-11-01): the training episode ends at
// walker_env fall_up_z = 0.4 (66 deg; linkproto::kFallUpZ, which stays the
// SIM's number). Past 45 deg on hardware the torso is going over, and torque
// held from there to 66 deg only drives the joints into the floor for longer;
// 45 deg limps the robot 21 deg sooner. (A get-up policy, which works lying
// down, will need its own rule when it is deployed.) The 200 ms debounce is
// the old one, unchanged, so a footfall transient still cannot trip it.
// Why tighter on the bench: scripted motion never leans on purpose -- the
// one-leg stands peak at the 12 deg balance abort -- so 30 deg is already
// unrecoverable there, and a 100 ms debounce stops the stream sooner.
//
// Pure: no hardware, no clock (now_ms arrives with every call, uint32
// milliseconds, wrap-safe), no globals -- host/test_fallguard.cpp asserts the
// properties. Each path owns its own Detector (ctrl_task on core 1, the CLI
// on core 0; no sharing across cores): an ARMED fall disarms the loop to
// bench and is handed to the bench Detector with latch(), so the same pick-up
// rule drives the autohome (cli.cpp fallTick, ctrl_task.cpp).
#pragma once
#include <stdint.h>

namespace fallguard {

enum class Motion : uint8_t {
    kBench = 0,      // scripted bench motion / benched idle (the CLI owns the bus)
    kPolicy = 1,     // the armed policy loop (ctrl_task owns the bus)
};

enum class Event : uint8_t {
    kNone = 0,
    kFell = 1,       // the latch tripped on this sample
    kPickedUp = 2,   // latched, and upright + still for kPickupHoldMs: home now
    kCleared = 3,    // (for logs) the latch was cleared by an operator / a fresh arm
};

struct Limits {
    float fall_up_z;          // torso up_z below this ...
    uint32_t debounce_ms;     // ... continuously for this long = fallen
};

constexpr Limits kPolicyLimits = {0.70711f, 200u};   // cos 45 deg, 200 ms
constexpr Limits kBenchLimits = {0.86603f, 100u};    // cos 30 deg, 100 ms
constexpr float kPickupTiltDeg = 25.0f;
constexpr float kPickupUpZ = 0.90631f;               // cos 25 deg
constexpr float kPickupGyroRadS = 1.0f;              // |w|, rad/s
constexpr uint32_t kPickupHoldMs = 1000u;

constexpr Limits limits(Motion m) { return m == Motion::kPolicy ? kPolicyLimits : kBenchLimits; }

// Tilt from vertical, degrees, of a torso whose z axis in the world is `up`
// (imu::Sample::up). 0 upright, 90 on its side, 180 upside down.
float tiltDeg(const float up[3]);

const char* motionName(Motion m);
const char* eventName(Event e);

class Detector {
  public:
    // One attitude sample. `valid` false (no fresh, finite attitude): no
    // decision either way -- a gap neither trips nor clears anything, and it
    // breaks a run of samples (the debounce and the pick-up hold restart), so
    // a stale attitude can never be the reason the robot is declared fallen
    // or picked up. Returns what happened on this sample.
    Event update(Motion m, bool valid, const float up[3], const float gyro[3], uint32_t now_ms);

    // A fall detected by the OTHER path (the armed loop disarmed itself):
    // latch it here, so this Detector's pick-up rule recovers it. Idempotent
    // while latched (the first fall's numbers stand). No Event: the caller
    // already knows.
    void latch(Motion m, float tilt_deg);

    // Clear the latch and both candidate runs (a typed `home`, a fresh arm,
    // `fall clear`, or after the pick-up home started).
    void clear();

    bool fallen() const { return fallen_; }
    Motion fellIn() const { return fell_in_; }          // the path the latched fall happened in
    float fallTiltDeg() const { return fall_tilt_deg_; }  // tilt when it latched
    uint32_t falls() const { return falls_; }           // since construction
    // ms the current candidate has lasted (0 = none): fall while not fallen,
    // pick-up while fallen. For `fall` status.
    uint32_t candidateMs(uint32_t now_ms) const;

  private:
    bool fallen_ = false;
    Motion fell_in_ = Motion::kBench;
    float fall_tilt_deg_ = 0.0f;
    uint32_t falls_ = 0;
    bool fall_cand_ = false, up_cand_ = false;
    uint32_t fall_since_ = 0, up_since_ = 0;
};

}  // namespace fallguard
