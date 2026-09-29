// The 50 Hz control loop (firmware-design section 1), pinned to core 1.
//
// Nothing here allocates, blocks on core 0, or takes a lock. The task blocks
// on a direct-to-task notification from a hardware esp_timer, so the wake
// jitter is microseconds rather than scheduler ticks.
#include "ctrl_task.h"

#include <math.h>
#include <string.h>

#include "battguard/guard.h"
#include "esp_attr.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_sampler.h"
#include "linkproto/watchdog.h"
#include "mech_envelope.h"
#include "obs/actuation.h"
#include "obs/assembler.h"
#include "policy/mlp.h"
#include "shared.h"

namespace robot {
namespace {

constexpr int kTickUs = 20000;                 // 20 ms == obs::kControlDt
constexpr uint32_t kOverrunUs = 2 * kTickUs;   // trips torque release

// Statically allocated task (firmware-design section 4: no dynamic task
// creation after init).
constexpr int kStackWords = 6144;
StackType_t g_stack[kStackWords];
StaticTask_t g_tcb;
TaskHandle_t g_task = nullptr;
esp_timer_handle_t g_tick_timer = nullptr;

// Loop-owned state. Single writer, never shared.
imu::Imu* g_imu = nullptr;
linkproto::Watchdog g_dog;
// Outlives g_dog across bench/run handovers -- it has to see the ARM edge
// that ENDS a run (linkproto/watchdog.h).
linkproto::ArmLatch g_arm;
// The "reset the servos" request. Fed in both modes, exactly like g_arm --
// its whole point is to work while benched, fallen or E-stopped.
linkproto::HomeLatch g_home;
battguard::Guard g_batt;
obs::Calibration g_cal;
obs::History g_hist;
obs::GaitClock g_clock(1.5f);
obs::VelocityEstimator g_vel;

float g_prev_action[obs::kActDim];
float g_frame[obs::kFrameDim];
float g_obs[obs::kObsDim];
float g_action[obs::kActDim];
float g_q[obs::kNumJoints];
float g_dq[obs::kNumJoints];
int32_t g_target_steps[obs::kNumJoints];
uint16_t g_target_speed[obs::kNumJoints];

// C2 command shaping (obs/actuation.h): the policy's raw 50 Hz targets go
// through three cascaded lags so the trajectory the servos chase has
// continuous velocity and acceleration, and each SYNC WRITE carries the speed
// that reaches the shaped target by the next tick -- instead of the max-speed
// slam-and-wait that put a 50 Hz velocity square wave on every joint.
// prev_action stays the RAW policy output: that is what the policy saw in
// training; the shaper is downstream of the policy's world.
obs::CommandShaper g_shaper;
static_assert(obs::kGoalSpeedMax == scsbus::kMaxGoalSpeed,
              "obs's speed ceiling must match the servo's");
// The BUS side (obs/bus_map.h). The loop reads and writes every FITTED bus
// joint (Calibration::fitted), not just the policy's: the policy's joints
// get the policy's targets, and every other fitted servo -- the robot's
// ankle rolls, neck and arms under a 10-joint policy -- is HELD where it was
// measured at takeover. Unfitted servos (IDs 11-17 on the prototype) are not
// on the wire at all: a SYNC READ that waits on an absent servo costs its
// whole timeout every tick. The fitted list is taken from the calibration
// at the arm handover, when the CLI cannot be changing it.
scsbus::Feedback g_fb[obs::kNumBusJoints];   // bus-indexed, last reply kept
bool g_fb_ok[obs::kNumBusJoints];            // answered THIS tick
float g_qbus[obs::kNumBusJoints];            // measured, rad; 0 if never read
int g_nfit = 0;
int g_fit_bus[obs::kNumBusJoints];           // fitted slot -> bus index
int g_fit_pol[obs::kNumBusJoints];           // fitted slot -> policy index, -1
uint8_t g_fit_id[obs::kNumBusJoints];        // fitted slot -> servo ID
scsbus::Feedback g_fit_fb[obs::kNumBusJoints];
bool g_fit_ok[obs::kNumBusJoints];
// The hold of a fitted servo the policy does not drive: its measured ticks at
// the first acting tick after an arm (or the first tick it answers). A servo
// the deployed run holds at a trained target (obs_spec.h kHeld*) ramps from
// there to that target; any other stays where it was measured.
int32_t g_hold_steps[obs::kNumBusJoints];
bool g_hold_valid[obs::kNumBusJoints];
// One SYNC WRITE's worth, in fitted-slot order.
uint8_t g_wr_id[obs::kNumBusJoints];
int32_t g_wr_steps[obs::kNumBusJoints];
uint16_t g_wr_speed[obs::kNumBusJoints];
bool g_torque_on = false;
bool g_primed = false;

// Takeover ramp (issue #29, loop-owned). The tick the policy takes the bus
// after a bench->run handover it used to SNAP the joints from the bench's
// rigid zero hold straight to its own targets -- an actuation step training
// never produced, into a policy whose history ring holds nothing but its
// first frame. Two armed ground sessions fell within seconds of exactly that
// handover on 2026-08-31. So for the first kArmRampMs of ACTING ticks the
// written targets blend from the pose MEASURED at takeover toward the
// policy's shaped targets: target = mix(q_hold, shaped, r), r 0->1. The
// blend sits after the shaper and before angleToSteps, so the goal-speed
// streaming chases the blended trajectory; prev_action stays the RAW policy
// output, same rule as the shaper -- the ramp is downstream of the policy's
// world. Deliberately NOT mirrored in sil_lib.cpp: the SIL harness has no
// bench/run handover (sil_reset IS its episode start) and its column
// comparability must not move.
constexpr int kArmRampMs = 1000;
constexpr int kArmRampTicks =
    static_cast<int>(kArmRampMs / (obs::kControlDt * 1000.0f) + 0.5f);
float g_ramp_hold[obs::kNumJoints];   // measured pose at the takeover tick
int g_ramp_ticks = kArmRampTicks;     // >= kArmRampTicks == ramp done
bool g_ramp_held = false;             // g_ramp_hold captured yet?

// Fall latch (loop-owned). Thresholds and the rationale live in
// linkproto/protocol.h next to the kFallen state they produce.
constexpr int kFallDebounceTicks =
    static_cast<int>(linkproto::kFallDebounceMs / (obs::kControlDt * 1000.0f));
bool g_fallen = false;
int g_fall_streak = 0;

// Rolling late-tick statistics for telemetry.
uint32_t g_ticks = 0, g_late = 0;

void IRAM_ATTR onTick(void*) {
    BaseType_t woke = pdFALSE;
    vTaskNotifyGiveFromISR(g_task, &woke);
    if (woke) portYIELD_FROM_ISR();
}

// Release every servo in one broadcast frame. This is the failsafe path, so it
// must be the cheapest thing in the file.
void releaseAll() {
    if (g_bus) g_bus->torqueEnable(scsbus::kBroadcastId, false);
    g_torque_on = false;
}

void engageAll() {
    if (g_bus) g_bus->torqueEnable(scsbus::kBroadcastId, true);
    g_torque_on = true;
}

uint32_t fittedMask() {
    uint32_t m = 0;
    for (int k = 0; k < g_nfit; ++k) m |= 1u << g_fit_bus[k];
    return m;
}

// Read every fitted servo in one SYNC READ, fill g_qbus and the policy's
// g_q / g_dq, and return the fault bitmask (bit b = bus joint b).
uint32_t readJoints(float dt) {
    uint32_t faults = 0;
    for (int b = 0; b < obs::kNumBusJoints; ++b) g_fb_ok[b] = false;
    if (!g_bus || g_nfit == 0) return fittedMask();
    g_bus->syncReadFeedback(g_fit_id, static_cast<size_t>(g_nfit), g_fit_fb,
                            g_fit_ok);
    for (int k = 0; k < g_nfit; ++k) {
        const int b = g_fit_bus[k];
        if (!g_fit_ok[k]) {
            faults |= 1u << b;
            continue;                       // hold the last angle for this one
        }
        g_fb[b] = g_fit_fb[k];
        g_fb_ok[b] = true;
        g_qbus[b] = obs::busStepsToAngle(b, g_fb[b].position, g_cal);
        if (g_fb[b].status) faults |= 1u << b;
    }
    // The policy's view: its joints are fitted (cmdMode refuses to arm
    // otherwise), so each one's angle is its bus joint's.
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const int b = obs::policyToBus(i);
        if (g_fb_ok[b]) g_q[i] = obs::stepsToAngle(i, g_fb[b].position, g_cal);
    }
    // Velocity source: the servo's own register 58. The finite-difference
    // alternative is kept alive (and fed) so the sys-ID open question in
    // firmware-design section 8 can be settled by flipping one branch.
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const int b = obs::policyToBus(i);
        g_dq[i] = g_fb_ok[b]
                      ? obs::stepsPerSecToRadPerSec(i, g_fb[b].speed, g_cal)
                      : 0.0f;
    }
    float fd[obs::kNumJoints];
    g_vel.update(g_q, dt, fd);
    (void)fd;
    return faults;
}

void publish(linkproto::LinkState st, uint32_t faults, const float up[3],
             uint8_t vbat_dv, uint32_t tick_us) {
    g_telemetry.state.store(static_cast<uint8_t>(st));
    g_telemetry.seq_echo.store(g_dog.lastSeq());
    g_telemetry.servo_err.store(faults);
    // All three, not just z: z alone is how FAR from upright, never which
    // way. A commander that asks with kFlagAtt gets the vector.
    g_telemetry.up_x.store(up[0]);
    g_telemetry.up_y.store(up[1]);
    g_telemetry.up_z.store(up[2]);
    g_telemetry.vbat_mv.store(static_cast<uint16_t>(vbat_dv) * 100u);
    g_telemetry.ticks.store(g_ticks);
    g_telemetry.overruns.store(g_late);
    if (tick_us > g_telemetry.worst_tick_us.load()) {
        g_telemetry.worst_tick_us.store(tick_us);
    }
    g_telemetry.loop_late_pct.store(static_cast<uint8_t>(
        g_ticks ? (100u * g_late) / g_ticks : 0u));
}

void ctrlTask(void*) {
    esp_task_wdt_add(nullptr);
    uint8_t vbat_dv = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t t0 = esp_timer_get_time();
        esp_task_wdt_reset();
        ++g_ticks;

        const float now_ms = static_cast<float>(t0) / 1000.0f;
        const bool benched = g_mode_request.load() == Mode::kBench;

        // -- bench handover ------------------------------------------------
        if (benched) {
            if (g_ctrl_owns_bus.load()) {
                releaseAll();               // never hand the CLI a live robot
                g_ctrl_owns_bus.store(false);
                g_primed = false;
                g_shaper.invalidate();
            }
        } else if (!g_ctrl_owns_bus.load()) {
            // The fitted bus set for this run, from the calibration as it
            // stands at the handover (the CLI writes it only while benched).
            g_nfit = fittedBusJoints(g_cal, g_fit_bus, g_fit_id);
            for (int k = 0; k < g_nfit; ++k) {
                g_fit_pol[k] = obs::busToPolicy(g_fit_bus[k]);
            }
            for (int b = 0; b < obs::kNumBusJoints; ++b) {
                g_hold_valid[b] = false;
            }
            g_ctrl_owns_bus.store(true);
            g_dog = linkproto::Watchdog();
            g_hist = obs::History();
            memset(g_prev_action, 0, sizeof g_prev_action);
            g_primed = false;
            g_shaper.invalidate();
            // Fresh run, fresh fall latch: `bench` + `run` is the tethered
            // clear (and a disarm + arm the wireless one), and the operator
            // doing it is looking at the robot.
            g_fallen = false;
            g_fall_streak = 0;
            // Fresh takeover ramp: re-arm it for this run's first acting
            // ticks. The hold pose is captured lazily at the first tick
            // that actually writes targets (the link may sit RELAX for a
            // while after `run`), from that tick's measured g_q.
            g_ramp_ticks = 0;
            g_ramp_held = false;
        }

        // -- drain the command mailbox (latest wins) -----------------------
        // In BOTH modes: while benched the watchdog is idle and the link
        // commands nothing, but the ArmLatch must still see every frame --
        // an ARM edge is how the wireless client arms the loop. The edge is
        // published, not acted on here; shared.h says why.
        CommandMsg msg;
        while (xQueueReceive(g_cmd_mailbox, &msg, 0) == pdTRUE) {
            bool want_armed = false;
            if (g_arm.update(msg.cmd.flags, want_armed)) {
                g_link_arm_level.store(want_armed);
                g_link_arm_edges.fetch_add(1);
            }
            // A home edge is published, never acted on here: the move is a
            // bench-mode CLI bus transaction and this task must not block on
            // one. Housekeeping benches the loop and then runs it -- which
            // is also why this needs no special case for the fall latch or
            // the E-stop. Both leave the frames flowing to exactly here.
            if (g_home.update(msg.cmd.flags)) {
                g_link_home_edges.fetch_add(1);
            }
            if (benched) continue;
            g_dog.accept(msg.cmd, now_ms);
        }

        if (benched) {
            // Keep the attitude filter running while benched. It is a
            // complementary filter with a ~2 s time constant, so it needs to
            // have been watching gravity for a while before its output means
            // anything -- and the instant `run` is typed, the very first tick
            // feeds `up` to the policy. Letting the estimate converge only
            // AFTER the robot is already walking is exactly backwards. The
            // IMU is on its own I2C bus, so this touches nothing the CLI owns.
            // The sampler task keeps the filter running; drain its gyro
            // accumulator so the first armed tick averages a fresh window.
            {
                imu::Sample warm{};
                takeImu(warm);
            }
            continue;
        }

        const linkproto::LinkState state = g_dog.state(now_ms);

        // -- sense ---------------------------------------------------------
        const int64_t t_read0 = esp_timer_get_time();
        const uint32_t faults = readJoints(obs::kControlDt);
        const int64_t t_read1 = esp_timer_get_time();
        // Publish the measured pose for the mirror-mode beacon (shared.h
        // g_joint_pose) -- every armed tick, LIVE or limp, because the pose
        // matters most when the link is RELAX/ESTOP and the robot is doing
        // its own thing. A faulted servo's entry is its last good angle
        // (readJoints holds it); `faults` says which, and rides the same
        // beacon. One 40-byte copy and a release store; lands in us_other.
        // Stamped with the middle of the bus read: the servos are polled one
        // after another, so the mean sample instant is the honest one. Every
        // bus joint, servo-ID order; an unfitted servo stays 0.
        g_joint_pose.publish(g_qbus, g_ticks, (t_read0 + t_read1) / 2);
        // read() == false means "no new report this tick"; the contract
        // (imu/imu.h) is that the caller REUSES the previous sample. Holding
        // it in a static and only overwriting on a successful read is that
        // reuse -- a fresh `Sample{}` per tick would feed the policy
        // up = (0,0,0), which reads as a torso in freefall on its side and is
        // an observation training never produced. Harmless under StubImu
        // (always true), live the moment a real driver lands.
        static imu::Sample s_held{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, 0,
                                  false};
        {
            imu::Sample fresh{};
            if (takeImu(fresh)) s_held = fresh;
        }
        const imu::Sample& s = s_held;
        const int64_t t_imu1 = esp_timer_get_time();

        // Pack voltage rides in on the servo feedback -- the board has no ADC
        // (main/board.h). It is already sitting in g_fb every tick, so the
        // guard is fed every tick and does its own debouncing in ticks; the
        // old 1 Hz decimation bought nothing and only slowed the trip.
        // 0 == "no servo answered", which the guard must not read as 0 volts.
        uint8_t fresh_dv = 0;
        for (int b = 0; b < obs::kNumBusJoints; ++b) {
            if (g_fb_ok[b]) { fresh_dv = g_fb[b].voltage_dv; break; }
        }
        if (fresh_dv != 0) vbat_dv = fresh_dv;
        g_batt.update(fresh_dv, now_ms);

        // -- fall detection ------------------------------------------------
        // The sim terminates an episode below fall_up_z (walker_env); the
        // hardware analogue of "episode over" is torque off. Past that line
        // the policy is outside anything training produced, and holding
        // torque only grinds servos against the floor. Debounced so a
        // footfall transient cannot trip it; the StubImu pins up_z to 1.0,
        // so an IMU-less bench board can never false-trigger.
        //
        // A fall ends the run OUTRIGHT: the latch trips, this tick still
        // reports FALLEN and goes limp below, and the mode request benches
        // the loop from the next tick. There is no auto-clear any more --
        // the old one (righted ~2 s + ENABLE off) re-engaged torque while
        // the robot was in the operator's hands on 2026-08-30 and twice on
        // 08-31. Re-arming is deliberate: a fresh wireless ARM edge or a
        // tethered `run`, both of which pass through the arm handover above,
        // which is also where the latch resets (user directive 2026-08-31).
        if (!g_fallen) {
            g_fall_streak =
                (s.up[2] < linkproto::kFallUpZ) ? g_fall_streak + 1 : 0;
            if (g_fall_streak >= kFallDebounceTicks) {
                g_fallen = true;
                g_arm_result.store(static_cast<uint8_t>(
                    linkproto::ArmResult::kDisarmedFall));
                g_mode_request.store(Mode::kBench);
            }
        }

        // -- safety --------------------------------------------------------
        // The pack guard outranks the link: an operator holding a live command
        // cannot keep the robot walking on a flat pack, and once the guard has
        // released torque nothing here re-engages it (the state is latched
        // inside the guard, so the `if (!g_torque_on) engageAll()` below is
        // unreachable while it holds). The fall latch ranks between the two:
        // above every link state (a downed robot must say so, not "LIVE"),
        // below the pack (a flat pack is the rarer, costlier message).
        const bool batt_limp = g_batt.torqueMustRelease();
        const linkproto::LinkState rep_state =
            g_batt.latched()
                ? (batt_limp ? linkproto::LinkState::kLowBattSafe
                             : linkproto::LinkState::kLowBattLand)
                : (g_fallen ? linkproto::LinkState::kFallen : state);
        const bool limp = (batt_limp || g_fallen ||
                           state == linkproto::LinkState::kRelax ||
                           state == linkproto::LinkState::kEstop);
        if (limp) {
            if (g_torque_on) releaseAll();
            // Torque is off, so the joints go wherever gravity and the
            // operator put them; the shaper must reseed from MEASURED q on
            // re-engage or the first shaped target would be a jump.
            g_shaper.invalidate();
            publish(rep_state, faults, s.up, vbat_dv,
                    static_cast<uint32_t>(esp_timer_get_time() - t0));
            continue;
        }
        if (!g_torque_on) engageAll();

        // -- observe -------------------------------------------------------
        // Full 7-wide ext_cmd vector off the link (vx, vy, wz, crouch,
        // lift, foot_dx, foot_dz); classic 14 B frames decode with the
        // extras at trained defaults, so nothing changes for old senders.
        float cmd7[7];
        g_dog.commandExt(now_ms, cmd7);
        const float vx = g_batt.latched() ? 0.0f : cmd7[0];
        const float vy = g_batt.latched() ? 0.0f : cmd7[1];
        const float wz = g_batt.latched() ? 0.0f : cmd7[2];
        const float lift = g_batt.latched() ? 0.0f : cmd7[4];
        // clock_stand_freeze runs generated per-policy: a policy trained
        // with the frozen-at-stand clock must see it here too, or the
        // stand obs oscillate in a way training never produced. Training's
        // gate (env_mjx plain_stand): no locomotion command AND no lift --
        // marches/balances keep their clock.
        if (!obs::kClockStandFreeze
            || fabsf(vx) > 0.05f || fabsf(vy) > 0.05f || fabsf(wz) > 0.05f
            || fabsf(lift) >= 0.5f) {
            // speed clock (kSpeedClock, 2026-09-08): cadence follows the
            // commanded planar speed exactly as in training; the base
            // frequency alone ran the robot 7-25 % slow on every drive
            // since loco_v32. Mirror of sil_lib.cpp.
            g_clock.advanceSpeedClock(obs::kControlDt, sqrtf(vx * vx + vy * vy));
        }

        obs::Inputs in{};
        memcpy(in.q, g_q, sizeof g_q);
        memcpy(in.dq, g_dq, sizeof g_dq);
        memcpy(in.up, s.up, sizeof in.up);
        memcpy(in.gyro, s.gyro, sizeof in.gyro);
        {
            const uint32_t fz = g_obs_freeze.load();
            if (fz & kObsFreezeUp) { in.up[0] = 0.0f; in.up[1] = 0.0f; in.up[2] = 1.0f; }
            if (fz & kObsFreezeGyro) { in.gyro[0] = in.gyro[1] = in.gyro[2] = 0.0f; }
            const float gg = g_obs_gyro_gain.load();
            if (gg != 1.0f) { for (int i = 0; i < 3; ++i) in.gyro[i] *= gg; }
            if (fz & kObsFreezeDq) { for (int i = 0; i < obs::kNumJoints; ++i) in.dq[i] = 0.0f; }
        }
        memcpy(in.prev_action, g_prev_action, sizeof g_prev_action);
        in.phase = g_clock.phase();
        // ext_cmd channel layout (walker_env.set_command): vx, vy, wz, crouch,
        // lift, foot_dx, foot_dz -- defaults (0,0,0,1,0,0,0).
        in.cmd[0] = vx;
        if (obs::kNumCmd > 1) in.cmd[1] = vy;
        in.cmd[2] = wz;
        // Crouch height: the LOWER of the link's command and the pack
        // guard's ramp -- the guard trips toward walker_env's
        // crouch_range[0], the lowest stance the policy trained to hold,
        // and a link asking for a deeper crouch than the guard allows is
        // still inside the trained band (both clamp at kCrouchMin).
        if (obs::kNumCmd > 3) {
            const float bg = g_batt.crouch();
            in.cmd[3] = cmd7[3] < bg ? cmd7[3] : bg;
        }
        if (obs::kNumCmd > 4) in.cmd[4] = lift;
        if (obs::kNumCmd > 5) in.cmd[5] = g_batt.latched() ? 0.0f : cmd7[5];
        if (obs::kNumCmd > 6) in.cmd[6] = g_batt.latched() ? 0.0f : cmd7[6];
        const int64_t t_obs0 = esp_timer_get_time();
        obs::assembleFrame(in, g_frame);
        if (!g_primed) {
            g_hist.fill(g_frame);
            g_primed = true;
        }
        g_hist.build(g_frame, g_obs);

        // -- act -----------------------------------------------------------
        const int64_t t_obs1 = esp_timer_get_time();
        policy::forward(g_obs, g_action);
        const int64_t t_net1 = esp_timer_get_time();
        g_hist.push(g_frame);
        memcpy(g_prev_action, g_action, sizeof g_prev_action);

        float angle[obs::kNumJoints];
        obs::actionToAngles(g_action, angle);

        // -- shape (see g_shaper above; docs/firmware-design.md) -------------
        const float shape_hz = g_shape_hz.load();
        if (shape_hz > 0.0f) {
            if (shape_hz != g_shaper.poleHz()) {
                g_shaper.setPole(shape_hz, obs::kControlDt);
            }
            if (!g_shaper.primed()) g_shaper.reset(g_q);   // measured, no jump
            g_shaper.update(angle, angle);
        } else {
            g_shaper.invalidate();      // re-enable reseeds from measured q
        }

        // -- takeover ramp (issue #29; rationale at kArmRampMs above) --------
        if (g_ramp_ticks < kArmRampTicks) {
            if (!g_ramp_held) {
                memcpy(g_ramp_hold, g_q, sizeof g_ramp_hold);
                g_ramp_held = true;
            }
            ++g_ramp_ticks;
            const float r = static_cast<float>(g_ramp_ticks) /
                            static_cast<float>(kArmRampTicks);
            for (int i = 0; i < obs::kNumJoints; ++i) {
                angle[i] = g_ramp_hold[i] + r * (angle[i] - g_ramp_hold[i]);
            }
        }

        for (int i = 0; i < obs::kNumJoints; ++i) {
            g_target_steps[i] = obs::angleToSteps(i, angle[i], g_cal);
            // g_fb[b].position holds the last reply from that servo; a joint
            // that has never answered reads 0, the error saturates and the
            // speed clamps to max -- exactly the legacy behaviour.
            g_target_speed[i] = obs::goalSpeedSteps(
                g_target_steps[i], g_fb[obs::policyToBus(i)].position);
        }
        // One SYNC WRITE over the fitted bus: the policy's targets, and a
        // hold at the takeover pose for every fitted servo it does not
        // drive. A held servo that has not answered since the arm is left
        // off the frame rather than sent a guess.
        int nwr = 0;
        for (int k = 0; k < g_nfit; ++k) {
            const int b = g_fit_bus[k];
            const int j = g_fit_pol[k];
            int32_t steps = 0;
            if (j >= 0) {
                steps = g_target_steps[j];
                g_wr_speed[nwr] = g_target_speed[j];
            } else {
                if (!g_hold_valid[b] && g_fb_ok[b]) {
                    g_hold_steps[b] = g_fb[b].position;
                    g_hold_valid[b] = true;
                }
                if (!g_hold_valid[b]) continue;
                steps = g_hold_steps[b];
                const int h = obs::heldIndexOfBus(b);
                if (h >= 0) {
                    // A servo the deployed run HOLDS at a trained target
                    // (obs_spec.h kHeldTarget: the robot's neck, its arms 15
                    // deg back): reached from the takeover pose along the
                    // same ramp as the policy's joints, then held there.
                    float tgt = obs::kHeldTarget[h];
                    if (tgt < kMechLo[b]) tgt = kMechLo[b];
                    if (tgt > kMechHi[b]) tgt = kMechHi[b];
                    const int32_t goal = obs::busAngleToStepsRaw(b, tgt, g_cal);
                    const float r =
                        g_ramp_ticks >= kArmRampTicks
                            ? 1.0f
                            : static_cast<float>(g_ramp_ticks) /
                                  static_cast<float>(kArmRampTicks);
                    steps = g_hold_steps[b] +
                            static_cast<int32_t>(lrintf(
                                r * static_cast<float>(goal - g_hold_steps[b])));
                }
                g_wr_speed[nwr] = obs::goalSpeedSteps(steps, g_fb[b].position);
            }
            g_wr_id[nwr] = g_fit_id[k];
            g_wr_steps[nwr] = steps;
            ++nwr;
        }
        const int64_t t_wr0 = esp_timer_get_time();
        if (g_bus && nwr > 0) {
            if (shape_hz > 0.0f) {
                g_bus->syncWritePositions(g_wr_id, g_wr_steps, g_wr_speed,
                                          static_cast<size_t>(nwr));
            } else {
                g_bus->syncWritePositions(g_wr_id, g_wr_steps,
                                          static_cast<size_t>(nwr));
            }
        }
        const int64_t t_wr1 = esp_timer_get_time();

        // -- dump ----------------------------------------------------------
        // The observation instrument (obs_dump.h). Deliberately last, after
        // the servos already have this tick's targets: it is a diagnostic and
        // must never sit between sensing and acting. Off it is one relaxed
        // load; on it is ~350 bytes of copy, which lands in `us_other` rather
        // than in any of the budgeted phase lines above.
        if (g_obs_dump.enabled()) g_obs_dump.publish(in, g_frame, g_ticks);

        // -- account -------------------------------------------------------
        const uint32_t took =
            static_cast<uint32_t>(esp_timer_get_time() - t0);
        if (took > static_cast<uint32_t>(kTickUs)) ++g_late;
        if (took > kOverrunUs) {
            // Two periods in one tick means we are not in control any more.
            releaseAll();
        }
        const uint32_t us_read = static_cast<uint32_t>(t_read1 - t_read0);
        const uint32_t us_imu = static_cast<uint32_t>(t_imu1 - t_read1);
        const uint32_t us_obs = static_cast<uint32_t>(t_obs1 - t_obs0);
        const uint32_t us_net = static_cast<uint32_t>(t_net1 - t_obs1);
        const uint32_t us_write = static_cast<uint32_t>(t_wr1 - t_wr0);
        g_telemetry.us_read.store(us_read);
        g_telemetry.us_imu.store(us_imu);
        g_telemetry.us_obs.store(us_obs);
        g_telemetry.us_net.store(us_net);
        g_telemetry.us_write.store(us_write);
        g_telemetry.us_other.store(
            took > (us_read + us_imu + us_obs + us_net + us_write)
                ? took - (us_read + us_imu + us_obs + us_net + us_write)
                : 0u);
        publish(rep_state, faults, s.up, vbat_dv, took);
    }
}

}  // namespace

std::atomic<bool> g_cal_from_nvs{false};
obs::Calibration& calibration() { return g_cal; }

int fittedBusJoints(const obs::Calibration& cal, int* bus_out,
                    uint8_t* id_out) {
    int n = 0;
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (!cal.fitted[b]) continue;
        if (bus_out) bus_out[n] = b;
        if (id_out) id_out[n] = obs::kBusServoId[b];
        ++n;
    }
    return n;
}
battguard::Guard& battGuard() { return g_batt; }

void startCtrlTask(imu::Imu& imu) {
    g_imu = &imu;
    memset(g_prev_action, 0, sizeof g_prev_action);
    memset(g_q, 0, sizeof g_q);
    memset(g_dq, 0, sizeof g_dq);
    g_vel.reset(g_q);

    // Highest application priority, pinned to core 1. WiFi/LwIP stay on core 0
    // by IDF default, so nothing they do can preempt this.
    g_task = xTaskCreateStaticPinnedToCore(
        ctrlTask, "ctrl", kStackWords, nullptr, configMAX_PRIORITIES - 2,
        g_stack, &g_tcb, 1);

    const esp_timer_create_args_t args = {
        .callback = &onTick,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "tick",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &g_tick_timer);
    esp_timer_start_periodic(g_tick_timer, kTickUs);
}

}  // namespace robot
