// The power-lite servo rail: when SERVO_EN may be high, and what ESTOP_N and
// RAIL_V mean (pcb/power-lite/README.md; issue #218).
//
// Relay K1 closes only while the E-stop toggle is at RUN AND the ESP32 holds
// SERVO_EN high. This class decides SERVO_EN and when the servo bus may be
// talked to. Pressing the E-stop opens the coil circuit without any firmware:
// this class only has to notice, latch, and not power the rail again by itself.
//
//   off ---requestOn()---> rising ---RAIL_V above 8 V---> settling ---500 ms---> on
//    ^                       |                               |                    |
//    |                       | no rail in 250 ms: fault      | drop()             | drop()
//    +-----------------------+-------------------------------+--------------------+
//
// Only `on` may talk to the servos (busUp()). Any drop() or a confirmed ESTOP_N
// high returns to `off` at once.
//
// THE E-STOP LATCH mirrors linkproto::Watchdog's: ESTOP_N going high latches,
// flipping the toggle back to RUN changes nothing, and the latch clears only on
// a disabled command (disabledCommand()) seen after the toggle is back at RUN.
// That only makes an arm possible again; powering the rail still takes a fresh
// requestOn(). An ARM bit held high through all this makes no edge and so no
// request. A floating or unplugged lead reads as stop, and so does the time
// before the first low reading: nothing here assumes the E-stop is released.
//
// THE WELDED-CONTACT CHECK: with SERVO_EN low (or ESTOP_N high) the rail must
// fall. It cannot be judged by a fixed time alone, because how fast it falls
// depends on what is plugged in:
//
//   R3 + R4 (the divider)   57 kOhm, always
//   R8 + LED2               4.7 kOhm and ~1.9 V, while LED2 conducts
//   C1                      1000 uF, EEU-FR1E102, +/-20 %: 1200 uF at most
//   the servos              0-2000 uF (the README's simulated range), 30 mA idle each
//
//   12.6 V to 8 V                                       time
//   8 servos fitted, 3200 uF, 240 mA                    0.06 s
//   no servos, C1 at 1200 uF, LED2 lit                  2.9 s    (a normal bench)
//   3200 uF with no servo draw, LED2 lit                7.7 s    (the bound nothing beats)
//   no servos, 1200 uF, LED2 open: 57 kOhm alone        31 s
//   3200 uF, no draw, LED2 open: 57 kOhm alone          83 s     (pathological)
//
// A false alarm on an ordinary disarm is the failure to avoid: it would teach
// the operator to ignore the one message that says the E-stop no longer works.
// So a rail that is still FALLING is never called welded, however long it takes.
// The fault needs kStuckMs with the rail above kRailOnMv and with no new low of
// kStuckDecayMv or more. The slowest row above still falls 44 mV/s at 8 V (8 V /
// 182 s), so 350 mV in kStuckMs against kStuckDecayMv = 150 mV, with room for
// the ADC's noise. A welded rail sits at the pack voltage and makes no new lows
// (noise can only lower the reference once or twice), so it faults kStuckMs after
// the drop. kStuckMs alone also beats 7.7 s, the slowest healthy row. Back-driving
// the servos by hand pumps the rail through the H-bridge diodes only while the
// joints are moving fast (above 3 rad/s for 8 V): not for 8 s.
//
// The same check runs from the first sample after boot, so a relay welded before
// power-up is caught, and a reboot mid-run with the rail still falling is not
// mistaken for one. requestOn() also refuses while the rail is above kRailOnMv
// with SERVO_EN low, so every arm starts from a rail seen to have fallen: K1 has
// proved it opened since last time, and "above 8 V" means this closing did it.
//
// Pure, like battguard and fallguard: no hardware, no clock (now_ms arrives with
// every sample, uint32 milliseconds, wrap-safe), no globals. host/test_railguard.cpp
// asserts the properties; main/power_rail.cpp is the hardware half.
#pragma once
#include <stdint.h>

namespace railguard {

// RAIL_V is SERVO+ through 47 k / 10 k: 5.7:1 (2.21 V at 12.6 V).
constexpr uint32_t kDividerX10 = 57;
// SERVO+ millivolts from the millivolts at the pin. 3.3 V at the pin is 18.8 V,
// so this fits a uint16_t for anything the ADC can report.
constexpr uint16_t railMvFromPinMv(uint32_t pin_mv) {
    return static_cast<uint16_t>((pin_mv * kDividerX10) / 10u);
}

// The rail counts as up above this (issue #218). 3S at the 9.0 V floor under a
// 12 A get-up keeps SERVO+ >= 8.66 V (README, Checks). 1404 mV at the pin,
// mid-range for the ADC at 11 dB, where it is good from 150 to 2450 mV.
constexpr uint16_t kRailOnMv = 8000;
// ... and has been lost below this for kRailLostMs: a blown F1 or a BMS trip
// (RAIL_V near 0 with SERVO_EN high and the E-stop released). Lower than
// kRailOnMv so ADC error on a sagging 9 V pack is not a fault, and long enough
// to match battguard::kConfirmTicks: a sag is the pack guard's business first.
constexpr uint16_t kRailLostMv = 7000;
constexpr uint32_t kRailLostMs = 500;
// SERVO_EN high to RAIL_V confirmed: K1's operate time (TE's figure is not in the
// repo; a relay of this class takes about 10 ms, and the bench check measures it),
// C1 and the servos charging (under 1 ms in the simulation, 52-132 us above 100 A),
// R3||R4 into C2 (8.2 kOhm x 100 nF = 0.8 ms) and two 20 ms samples: about 60 ms.
// A generous bound costs nothing: a coil left waiting on a dead rail harms nothing.
constexpr uint32_t kRiseTimeoutMs = 250;
// RAIL_V confirmed to the first bus frame: the servos' MCUs booting. NOT measured
// yet: the bench check does (pcb/power-lite/README.md, "The firmware").
constexpr uint32_t kServoBootMs = 500;
// A single high sample is noise (K1's coil transient is on the same board); two
// in a row, 20 ms apart, is a press. It does not have to be faster than this: the
// contact has already cut the rail. Back to RUN must hold longer to count.
constexpr uint32_t kEstopPressMs = 20;
constexpr uint32_t kEstopReleaseMs = 100;
// The welded-contact check: see above.
constexpr uint32_t kStuckMs = 8000;
constexpr uint16_t kStuckDecayMv = 150;
// A commander's ESTOP frame refuses the rail until its next frame without the flag,
// or until this long has passed with no frame at all: a console that held ESTOP and
// went away must not lock the tether out of `rail on` for good.
constexpr uint32_t kCommanderEstopHoldMs = 500;

enum class State : uint8_t {
    kOff = 0,       // SERVO_EN low
    kRising = 1,    // SERVO_EN high, waiting for RAIL_V
    kSettling = 2,  // RAIL_V up, the servos are booting: still no bus traffic
    kOn = 3,        // the bus may be used
};

// Why SERVO_EN last went low.
enum class DropWhy : uint8_t {
    kNone = 0,          // never been on
    kDisarm = 1,        // a disarm, `rail off`, or the end of a bench home
    kOperatorStop = 2,  // kFlagEstop on a kFlagStop frame (tools/estop.py)
    kLinkRelax = 3,     // the link watchdog's RELAX
    kLinkEstop = 4,     // the link watchdog's ESTOP
    kBattSafe = 5,      // battguard latched the pack as safe
    kFall = 6,          // fallguard, either path
    kEstopInput = 7,    // ESTOP_N went high
    kRiseTimeout = 8,   // no RAIL_V after SERVO_EN (also Fault::kRiseTimeout)
    kRailLost = 9,      // RAIL_V collapsed while on (also Fault::kRailLost)
};

enum class Fault : uint8_t {
    kNone = 0,
    kRiseTimeout = 1,  // SERVO_EN high, no rail: F1, the relay, the lead. Next arm retries.
    kRailLost = 2,     // the rail collapsed while on. Next arm retries.
    kStuck = 3,        // RAIL_V up with SERVO_EN low: K1 welded. Refuses arming until cleared.
};

enum class Verdict : uint8_t {
    kOk = 0,            // rising, or already there
    kEstopPressed = 1,  // ESTOP_N is high (or not yet seen low)
    kEstopLatched = 2,  // back at RUN, but no disabled command since
    kStuckFault = 3,    // Fault::kStuck stands: clearFault() after looking at K1
    kRailStillUp = 4,   // still discharging from the last drop: try again shortly
    kNoReading = 5,     // no sample yet, or the ADC read failed
    kCommanderEstop = 6,  // the commander is sending ESTOP: it has to let go first
};

struct Sample {
    uint32_t now_ms = 0;
    bool estop_high = true;   // the ESTOP_N pin (high is stop)
    bool rail_valid = false;  // false: rail_mv means nothing; hold every RAIL_V decision
    uint16_t rail_mv = 0;     // SERVO+ in mV (railMvFromPinMv)
};

class Guard {
  public:
    // One control tick, in every mode.
    void update(const Sample& s);

    // A deliberate request for the rail (an arm, `rail on`, a home).
    Verdict requestOn();

    // SERVO_EN low now. No-op when already off: the first reason stands.
    void drop(DropWhy why);

    // A disabled command arrived: clears the E-stop latch if the toggle is back at
    // RUN, and otherwise does nothing. It never powers the rail.
    void disabledCommand();

    // Every valid frame from the commander, in every mode. ESTOP set: SERVO_EN low
    // now (kLinkEstop), and requestOn() refuses while it stays set. It latches nothing
    // here: the flag is a level, and the commander's next frame without it ends this.
    void commanderFrame(bool estop);

    // Clear a fault. Refused (false) while Fault::kStuck's rail is still up.
    bool clearFault();

    bool servoEn() const { return state_ != State::kOff; }
    bool busUp() const { return state_ == State::kOn; }
    State state() const { return state_; }
    Fault fault() const { return fault_; }
    DropWhy lastDrop() const { return last_drop_; }
    bool estopPressed() const { return pressed_; }
    bool estopLatched() const { return latched_; }
    uint16_t railMv() const { return rail_mv_; }
    bool railValid() const { return rail_valid_; }

  private:
    void updateEstop(const Sample& s);
    void updateState(const Sample& s);
    void updateStuck(const Sample& s);

    State state_ = State::kOff;
    Fault fault_ = Fault::kNone;
    DropWhy last_drop_ = DropWhy::kNone;
    bool have_sample_ = false;
    bool rail_valid_ = false;
    uint16_t rail_mv_ = 0;
    uint32_t now_ms_ = 0;
    uint32_t state_since_ = 0;

    bool pressed_ = true;   // every unknown reads as stop
    bool latched_ = false;
    bool high_cand_ = false, low_cand_ = false;
    uint32_t high_since_ = 0, low_since_ = 0;

    bool rise_cand_ = false;
    bool lost_cand_ = false;
    uint32_t lost_since_ = 0;

    bool stuck_cand_ = false;
    uint16_t stuck_low_mv_ = 0;
    uint32_t stuck_since_ = 0;

    bool cmdr_estop_ = false;
    uint32_t cmdr_estop_ms_ = 0;
};

// Why the armed loop went limp: the same ranking as ctrl_task's safety block (the
// pack, then the fall, then the link; not RELAX means the link's ESTOP).
constexpr DropWhy limpWhy(bool batt_safe, bool fallen, bool relax) {
    return batt_safe ? DropWhy::kBattSafe
           : fallen  ? DropWhy::kFall
           : relax   ? DropWhy::kLinkRelax
                     : DropWhy::kLinkEstop;
}

// An ARM edge in a frame that also says ESTOP is no request for power: the consoles
// send ARM | ESTOP to release torque. It is refused, not delayed: the commander's ARM
// level stands, so arming after this takes ARM low and a fresh edge.
constexpr bool armEdgeRefused(bool want_armed, bool estop) { return want_armed && estop; }

const char* stateName(State s);
const char* dropName(DropWhy w);
const char* faultName(Fault f);
const char* verdictName(Verdict v);

}  // namespace railguard
