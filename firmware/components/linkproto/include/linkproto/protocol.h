// C++ port of link/protocol.py -- the laptop <-> robot command link.
//
// link/protocol.py is the REFERENCE. This file conforms to it byte for byte,
// never the other way round, and tools/gen_protocol_vectors.py freezes the
// Python side's output into host/vectors/protocol_vectors.h so a divergence is
// a failing test rather than a mystery on the bench. See docs/control-channel.md.
//
// Everything here is pure: no sockets, no clock (the Watchdog takes now_ms as
// an argument exactly like the Python one), so it runs on the host under
// sanitizers.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "linkproto/cmd_envelope.h"   // generated: the trained command envelope

namespace linkproto {

// -- framing ---------------------------------------------------------------
constexpr uint8_t kMagicCmd[2] = {'B', 'M'};   // laptop -> robot
constexpr uint8_t kMagicTlm[2] = {'B', 'T'};   // robot -> laptop
// Protocol version 2 (issue #81): the telemetry frame carries the 17-servo
// bus -- servo_err is a u32 (bit b = bus joint b) and the joint block is
// 17 angles in bus order. The BASE block changed, which a length cannot
// say, so the version is 2 in both directions and a version-1 peer's frames
// are refused (kBadVersion) rather than half-read. protocol.py VERSION.
constexpr uint8_t kVersion = 2;
constexpr size_t kCmdLen = 14;
// Extended command frame (2026-08-31): the same 12-byte prefix, then five
// more int16 milli-channels (vy, crouch, lift, foot_dx, foot_dz) completing
// walker_env.set_command()'s 7-wide ext_cmd vector. LENGTH selects the
// layout, no version bump: a 14 B frame decodes with the extras at their
// trained defaults (0, 1.0, 0, 0, 0), so every pre-extension sender keeps
// working. protocol.py is the reference for the sender-side rule (emit the
// short frame whenever the extras are at defaults).
constexpr size_t kCmdLenExt = 24;
// Telemetry lengths. The 21-byte body (magic, version, state, seq_echo,
// vbat, up_z, vx_est, wz_est, the u32 servo_err, loop_late_pct), then a u64
// t_us (microseconds since the Unix epoch, UTC, from the robot's SNTP-
// disciplined clock; 0 until the first sync), then the optional blocks, then
// the CRC. The stamp rides EVERY beacon -- docs/control-channel.md "Time on
// the wire" says why this one is volunteered where the joints are not.
constexpr size_t kTlmTimeOff = 21;  // the u64 sits right after the body
constexpr size_t kTlmLen = kTlmTimeOff + 8 + 2;   // 31
// The JOINT block (mirror mode): one int16 milli-radian per BUS joint, in
// bus order (obs/bus_map.h; protocol.py JOINT_NAMES), then the CRC. For
// driving the real robot while a sim follows its observed pose.
//
// The robot is the SENDER and every client rejects an unexpected length, so
// a robot that simply began beaconing the long frame would blind every
// existing commander at once. Hence kFlagPose: the long frame is REQUESTED,
// never volunteered. See docs/mirror-mode.md.
constexpr size_t kNumJoints = 17;   // bus order (servo IDs: obs/bus_map.h kBusServoId)
constexpr size_t kTlmLenExt = kTlmLen + 2 * kNumJoints;   // 65

// ATTITUDE: up_x and up_y, int16 milli, appended AFTER the joint block and
// before the CRC. up_z is in the base frame, but alone it is only the tilt
// MAGNITUDE -- it says how far from upright the torso is and nothing about
// which way it fell. All three components are what mirror mode needs to draw
// the robot in the attitude it is actually in rather than upright-and-wrong.
//
// Requested, never volunteered -- kFlagAtt, for exactly the reason kFlagPose
// exists and spelled out above: the robot is the SENDER, and a client that
// gets a length it does not know drops the frame.
//
// Four lengths, and they stay unambiguous because each block has a fixed
// size: 31 base, 35 +att, 65 +joints, 69 +joints+att. The CRC is "over
// len-2, appended at len-2".
constexpr size_t kTlmAttBytes = 4;
constexpr size_t kTlmLenAtt = kTlmLen + kTlmAttBytes;              // 35
constexpr size_t kTlmLenExtAtt = kTlmLenExt + kTlmAttBytes;        // 69
constexpr size_t kTlmLenMax = kTlmLenExtAtt;

constexpr uint16_t kCmdPort = 4210;
constexpr uint16_t kTlmPort = 4211;

// -- command flags ---------------------------------------------------------
constexpr uint8_t kFlagEnable = 1u << 0;   // 0 = stand still regardless of vx/wz
constexpr uint8_t kFlagEstop = 1u << 1;    // latching torque release
constexpr uint8_t kFlagArm = 1u << 2;      // operator wants the loop armed; ArmLatch
// "Beacon joint angles too" (kTlmLenExt). A level, not an edge: stop asking
// and the very next beacon is classic again.
constexpr uint8_t kFlagPose = 1u << 3;
// "Put every joint back at its calibrated zero -- the standing pose -- now."
// (2026-09-02) A RECOVERY action rather than a command, and the difference is
// the whole point: it is honoured while the loop is BENCHED, while the FALL
// latch is tripped and while the E-stop is latched -- exactly the states in
// which every other channel on this wire correctly refuses to move anything,
// and exactly the states an operator is in when the robot is a heap on the
// floor and needs to be stood back up before it can be armed again.
//
// It does not arm and cannot be used to walk: the robot BENCHES first, the
// CLI half of the firmware owns the bus for the move, and the joints slew at
// the bench's gentle speed (cli.cpp cmdHome). Re-arming stays as deliberate
// as it ever was -- a fresh ARM edge.
//
// Acted on by its RISING EDGE (HomeLatch), like kFlagArm and for the same
// two reasons: a level would re-issue the move every frame at 20 Hz, and a
// client that reboots with the bit set must not move a robot nobody is
// watching.
constexpr uint8_t kFlagHome = 1u << 4;

// "Beacon the torso up vector too" (kTlmLenAtt / kTlmLenExtAtt). A LEVEL, not
// a latch, exactly like kFlagPose: stop asking and the very next beacon is
// short again. It requests telemetry and commands nothing.
constexpr uint8_t kFlagAtt = 1u << 5;

// OPERATOR STOP, out of band (2026-11-01). A frame carrying this bit comes
// from a stop button (tools/estop.py), not from a commander, and the robot
// acts on it the moment it is decoded, in EVERY mode -- including a tethered
// `run` with no commander at all, which is how the first armed policy stand
// was run (and flailed, with no way to stop it but a grab).
//
// Out of band means it is NEVER fed to the ArmLatch, the HomeLatch or the
// Watchdog. A second sender's frames would otherwise interleave with a
// running commander's: its ARM level would make arm/bench edges against the
// commander's (each re-arm a fresh Watchdog, so a fresh run), its seq could be
// rejected as stale, and the commander's next ENABLE-off frame would clear a
// plain ESTOP latch. None of that can happen to a frame nobody else reads.
//
// What it does is a LEVEL (every such frame asks again; repeats are harmless):
//   with kFlagEstop: HARD stop -- torque released now (ctrl's tick, <= 20 ms
//     after the frame), the run ends (disarmed to bench, kDisarmedStop);
//     benched, any bench motion stops and torque is released;
//   without: SOFT stop -- the policy stops and every servo holds its present
//     position, torque ON, the run ends (kDisarmedStop) and the bench homes
//     the robot once it is upright and still -- the flail guard's own stop;
//     benched, any bench motion stops where it is.
// Re-arming is a fresh ARM edge or `run`, as after a fall. An old firmware
// ignores the bit, so the stop tool also sets kFlagEstop with ARM clear on a
// hard stop: the old meaning (watchdog ESTOP latch) still releases torque.
constexpr uint8_t kFlagStop = 1u << 6;

// -- timing ----------------------------------------------------------------
constexpr float kSendHz = 20.0f;
constexpr float kStaleMs = 250.0f;         // -> decay to the trained stand
constexpr float kRelaxMs = 5000.0f;        // -> torque off entirely
constexpr uint32_t kSeqResyncGap = 1000;   // bigger backwards jump = restart

// -- trained command envelope ----------------------------------------------
// The deployed policy's envelope -- forward, backward and lateral speed
// bands, turn limit, crouch floor -- is GENERATED from its run's config.json
// by tools/gen_envelope.py (`make deploy RUN=...`) into cmd_envelope.h, and
// link/cmd_envelope.py is its Python twin. A policy profile
// (firmware/policies/<profile>/linkproto/) carries its own, first on the
// include path. Never edit them by hand. clampToEnvelope() below enforces it.
// Not from a run's config: the swing-foot offsets are scripted, not drawn
// (protocol.py FOOT_D_MAX).
constexpr float kFootDMax = 0.05f;  // traj_radius hi: foot offsets, m

enum class Err : uint8_t {
    kOk = 0,
    kBadLength,
    kBadMagic,
    kBadVersion,
    kBadCrc,
    kBadState,       // telemetry only: link-state byte out of range
};

enum class LinkState : uint8_t {
    // Order matters: this is the on-the-wire encoding, and it is
    // list(LinkState) order in protocol.py.
    kLive = 0,
    kStand = 1,
    kRelax = 2,
    kEstop = 3,
    // Pack under-voltage, latched by battguard::Guard. Unlike every state
    // above, these are decided by the ROBOT and no command clears them.
    kLowBattLand = 4,   // crouching down under control on a flat pack
    kLowBattSafe = 5,   // crouch finished, torque off, and it stays off
    // Torso down (tilt past 45 deg for 200 ms: fallguard::kPolicyLimits, NOT
    // kFallUpZ, which is the sim's episode end), robot-latched: torque off
    // so a downed robot does not grind its servos against the floor. A fall
    // DISARMS the robot (ctrl_task): the state is reported for the fall tick,
    // then the loop benches. Re-arming is deliberate -- a fresh wireless ARM
    // edge or a tethered `run` -- never automatic.
    kFallen = 6,
    // Control loop benched: CLI owns the bus, the link commands nothing,
    // torque is off. Boot state. Left via an ArmLatch edge or the tethered
    // `run`.
    kBench = 7,
    kMaxState = kBench,
};

// -- bench diagnostics (2026-08-30) ----------------------------------------
// Why the loop is NOT armed, carried to the operator over the radio.
//
// The refusal used to exist only as a line of UART text, so a robot that
// declined a wireless ARM just sat in BENCH saying nothing and the only way
// to find out why was to plug the tether. This byte is that answer on the
// wire. It rides in `seq_echo` and ONLY while the reported state is kBench:
// seq_echo means "the last command seq the loop APPLIED", and a benched loop
// applies nothing, so in exactly that state the field carries no information
// to destroy. No version bump, no length change -- every commander built
// before this still decodes every frame (docs/control-channel.md).
//
// Bit layout, APPEND-ONLY like the LinkState enum: reserved bits are sent
// zero and a client must ignore the ones it does not know.
constexpr uint8_t kDiagRun = 1u << 0;      // mode is kRun (0 = benched)
constexpr uint8_t kDiagCalOk = 1u << 1;    // as-built calibration came from NVS
// bits 2-3 reserved, sent zero
constexpr uint8_t kDiagArmShift = 4;       // bits 4-7: the ArmResult below
constexpr uint8_t kDiagArmMask = 0xF0;

// What became of the last mode request -- a wireless ARM edge or a typed
// `run`/`bench`. APPEND ONLY: a future refusal reason takes the next value,
// and a client that does not know it must fall back to a generic message
// rather than mis-report an older reason.
enum class ArmResult : uint8_t {
    kNone = 0,           // nothing has asked for a mode change yet
    kAccepted = 1,       // the last request was honoured (arm or disarm)
    kRefusedNoCal = 2,   // arm refused: no as-built calibration in NVS
    kDisarmedFall = 3,   // the FALL latch tripped: run over, robot disarmed
    kDisarmedHome = 4,   // a kFlagHome request benched the loop and homed it
    // ... and the three ways that request can fail. They exist for the same
    // reason kRefusedNoCal does: a reset that silently does nothing sends an
    // operator to look for a broken button, a dead radio or a seized servo,
    // when the robot knew the answer all along (docs/control-channel.md,
    // "Saying WHY, over the radio").
    kHomeNoCal = 5,      // reset REFUSED: no as-built calibration in NVS
    kHomeLowBatt = 6,    // reset REFUSED: the pack guard has torque latched off
    kHomeBusFailed = 7,  // reset FAILED: the servo bus did not accept it
    // (2026-09-03) A reset that "succeeded" once moved nothing: the goal
    // write is a broadcast with no reply, so kDisarmedHome only ever meant
    // "written". Now the write reports kHomePending, and the housekeeping
    // loop reads every joint back after the slew (cli::homeVerify) and
    // stores kDisarmedHome only when all ten sit at their zeros -- or this:
    kHomePending = 8,    // reset WRITTEN: joints slewing, readback not yet done
    kHomeNotReached = 9, // reset FAILED: readback found joints off their zeros
    // Plan B (issue #81): the position-loop gains live in each servo's
    // EEPROM, and a factory reset or a swapped spare silently comes back at
    // P = 32. The robot reads registers 21/22 back before every arm
    // (scsbus::checkPositionGains) and refuses on any difference from the
    // compiled expected table (main/servo_gains.h).
    kRefusedGains = 10,  // arm refused: a servo's P/D is not the expected value
    // A policy profile (firmware/policies/, BIMO_POLICY) can compile in an
    // UNTRAINED test net while its plant has no deployable policy
    // (policy::kWeightsArePlaceholder). It exists to golden-test the
    // firmware path; driving servos with it is never what anyone meant.
    kRefusedPolicy = 11, // arm refused: the compiled-in net is not a policy
    // (2026-11-01) The armed loop's flail guard (components/flailguard) found
    // the policy doing something clearly wrong for its command -- targets far
    // from the stand, the torso thrashing or tilting, the targets chattering
    // -- stopped it, froze every servo where it was with torque ON, and ended
    // the run. The bench homes it once it is upright and still.
    kDisarmedFlail = 12,
    // An operator stop (kFlagStop) ended the run: torque released (hard) or
    // held where it was (soft).
    kDisarmedStop = 13,
};

// True for every verdict a kFlagHome request can produce. A console that
// asked for a reset uses this to tell "the robot answered me" from "the robot
// is talking about something else" -- an old firmware that never heard of the
// request still reports kAccepted for the disarm that rode in with it.
inline bool isHomeResult(ArmResult r) {
    return r == ArmResult::kDisarmedHome || r == ArmResult::kHomeNoCal ||
           r == ArmResult::kHomeLowBatt || r == ArmResult::kHomeBusFailed ||
           r == ArmResult::kHomePending || r == ArmResult::kHomeNotReached;
}

// The verdict an operator stop leaves while BENCHED (firmware cli.cpp
// flailTickImpl), given the one standing and whether a servo reset was
// still under way. A stop during a reset ends the reset where it is: no
// readback follows, so the kHomePending it reported would stand until the
// next arm or reset, and every client reads that as bench motion still in
// progress -- a soft stop (HOLD) would never confirm and would escalate to a
// hard one. The stop is what happened, so it says kDisarmedStop, the verdict
// tools/estop.py and the consoles confirm a stop on. kHomeNotReached would
// not do: it reports a reset whose readback failed (an operator sent to
// `scan` for a fault that is not there), and estop.py does not confirm a
// soft stop on it. With no reset under way the standing verdict is kept: the
// stop changed nothing it reports.
inline ArmResult benchStopResult(ArmResult current, bool reset_pending) {
    return reset_pending ? ArmResult::kDisarmedStop : current;
}

uint8_t packDiag(bool run, bool cal_ok, ArmResult result);
ArmResult diagArmResult(uint8_t diag);
// One line an operator can act on. Never null, and safe for a diag byte from
// a firmware newer than this client.
const char* diagReason(uint8_t diag);

// -- fall detection ----------------------------------------------------------
// kFallUpZ is walker_env's own fall_up_z: the SIM terminates an episode below
// it (66 deg). It stays here as the training contract's number, for the twin
// and sim code. The HARDWARE no longer uses it: both control paths run
// components/fallguard (fallguard.h has the one table), and the armed loop
// trips at fallguard::kPolicyLimits = cos 45 deg (0.707) for 200 ms -- 21 deg
// before the episode end, because torque held past 45 deg only grinds the
// servos into the floor (Tom 2026-11-01).
constexpr float kFallUpZ = 0.4f;
constexpr float kFallDebounceMs = 200.0f;   // the sim-side debounce; fallguard's policy debounce matches it
// The firmware latch no longer auto-clears (a fall DISARMS the robot,
// ctrl_task.cpp); these two remain for the python twin's Watchdog and any
// host-side "is it righted yet" display.
constexpr float kUprightUpZ = 0.7f;     // hysteresis: righted is > this ...
constexpr float kUprightMs = 2000.0f;   // ... held this long, + ENABLE off

struct Command {
    uint32_t seq = 0;
    float vx = 0.0f;       // body-frame forward velocity, m/s
    float wz = 0.0f;       // yaw rate, rad/s
    uint8_t flags = 0;
    // extended channels (kCmdLenExt frames); a classic frame decodes to
    // exactly these defaults -- walker_env ext_cmd layout beyond vx/wz
    float vy = 0.0f;       // lateral velocity, m/s
    float crouch = 1.0f;   // stance-height fraction command (cmd[3])
    float lift = 0.0f;     // swing-foot selector: -1 left, +1 right
    float foot_dx = 0.0f;  // swing-foot target x offset, m
    float foot_dz = 0.0f;  // swing-foot target extra height, m

    bool enabled() const { return (flags & kFlagEnable) != 0; }
    bool estop() const { return (flags & kFlagEstop) != 0; }
    bool arm() const { return (flags & kFlagArm) != 0; }
    bool pose() const { return (flags & kFlagPose) != 0; }
    bool home() const { return (flags & kFlagHome) != 0; }
    bool att() const { return (flags & kFlagAtt) != 0; }
    bool stop() const { return (flags & kFlagStop) != 0; }
};

struct Telemetry {
    uint32_t seq_echo = 0;
    LinkState state = LinkState::kLive;   // 0, as value-initialisation gave
    float vbat_v = 0.0f;
    float up_z = 0.0f;       // torso up-vector z; 1.0 = perfectly upright
    float vx_est = 0.0f;
    float wz_est = 0.0f;
    uint32_t servo_err = 0;  // bitmask, bit b = bus joint b faulted
    uint8_t loop_late_pct = 0;
    // Measured joint angles, radians, bus order.
    // n_joints is 0 on a classic frame -- which is every frame, unless a
    // commander asked with kFlagPose.
    uint8_t n_joints = 0;
    float joints[kNumJoints] = {0};
    // Microseconds since the Unix epoch (UTC) on the robot's clock. 0 = not
    // synced yet, or a legacy frame. With joints present it is the instant
    // they were read off the bus; otherwise the instant the beacon was
    // assembled -- either way within one 20 ms tick of the values.
    uint64_t t_us = 0;
    // Torso up vector, x and y (z is up_z above). The robot's own z axis
    // EXPRESSED IN THE WORLD FRAME -- obs_spec kOffUp / imu::Sample::up, the
    // same convention MuJoCo's framezaxis uses. have_att is 0 on any frame
    // that was not asked for it, which is every frame unless a commander set
    // kFlagAtt.
    //
    // Worth knowing before trusting these: up_z is drift-immune, but up_x and
    // up_y are yaw-DEPENDENT, and this robot's heading is dead-reckoned with
    // no magnetometer to correct it. They give the lean direction in a frame
    // that slowly rotates. Good enough to draw an attitude; not a heading.
    uint8_t have_att = 0;
    float up_x = 0.0f;
    float up_y = 0.0f;
};

// The bench diagnostic byte this frame carries, or 0 when it carries none.
// Reading seq_echo directly would be a bug waiting to happen: the byte is
// only meaningful while the state is kBench.
uint8_t telemetryDiag(const Telemetry& t);

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
// crc16Ccitt("123456789") == 0x29B1 -- the standard check value, asserted in
// the host tests so this port is diffed against a number, not against
// "whatever Python said".
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// Snap a raw command into the region the policy was actually trained on
// (cmd_envelope.h): vx into its forward or backward band, a speed under the
// band's minimum to a stand, wz to +-kWMax.
// Enforced robot-side because the robot is what eats a bad command.
void clampToEnvelope(float vx, float wz, float& out_vx, float& out_wz);

// The extended channels, snapped into their trained draws (in place): |vy|
// into the lateral band (under its minimum to 0), crouch into [kCrouchMin, 1].
void clampExtToEnvelope(Command& pkt);

// Encode into `out` (kCmdLen / kTlmLen bytes). Returns bytes written.
size_t encodeCommand(uint8_t* out, uint32_t seq, float vx, float wz,
                     uint8_t flags);
// Extended frame: always kCmdLenExt bytes (senders wanting the short-frame
// default rule use protocol.py; firmware-side encoding exists for the twins
// and tests).
size_t encodeCommandExt(uint8_t* out, const Command& cmd);
// Accepts kCmdLen (extras -> defaults) and kCmdLenExt frames.
Err decodeCommand(const uint8_t* buf, size_t len, Command& out);

// Emits the blocks that are set: the u64 timestamp ALWAYS, then joints when
// t.n_joints == kNumJoints, then the up vector when t.have_att. `out` must
// have room for kTlmLenMax. Returns the bytes written -- USE IT: sending
// kTlmLen of a long frame truncates it and every CRC downstream fails.
size_t encodeTelemetry(uint8_t* out, const Telemetry& t);
// Accepts the four lengths above. n_joints is kNumJoints on the two long
// lengths, else 0; have_att is 1 on the two +att lengths.
Err decodeTelemetry(const uint8_t* buf, size_t len, Telemetry& out);

}  // namespace linkproto
