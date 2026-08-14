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
#include "linkproto/watchdog.h"
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
scsbus::Feedback g_fb[obs::kNumJoints];
bool g_fb_ok[obs::kNumJoints];
bool g_torque_on = false;
bool g_primed = false;

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

// Read the bus, fill g_q / g_dq, and return the fault bitmask.
uint16_t readJoints(float dt) {
    uint16_t faults = 0;
    if (!g_bus) return 0xFFFF;
    g_bus->syncReadFeedback(servoIds(), obs::kNumJoints, g_fb, g_fb_ok);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        if (!g_fb_ok[i]) {
            faults |= static_cast<uint16_t>(1u << i);
            continue;                       // hold the last angle for this one
        }
        g_q[i] = obs::stepsToAngle(i, g_fb[i].position, g_cal);
        if (g_fb[i].status) faults |= static_cast<uint16_t>(1u << i);
    }
    // Velocity source: the servo's own register 58. The finite-difference
    // alternative is kept alive (and fed) so the sys-ID open question in
    // firmware-design section 8 can be settled by flipping one branch.
    for (int i = 0; i < obs::kNumJoints; ++i) {
        g_dq[i] = g_fb_ok[i]
                      ? obs::stepsPerSecToRadPerSec(i, g_fb[i].speed, g_cal)
                      : 0.0f;
    }
    float fd[obs::kNumJoints];
    g_vel.update(g_q, dt, fd);
    (void)fd;
    return faults;
}

void publish(linkproto::LinkState st, uint16_t faults, float up_z,
             uint8_t vbat_dv, uint32_t tick_us) {
    g_telemetry.state.store(static_cast<uint8_t>(st));
    g_telemetry.seq_echo.store(g_dog.lastSeq());
    g_telemetry.servo_err.store(faults);
    g_telemetry.up_z.store(up_z);
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

        // -- bench handover ------------------------------------------------
        if (g_mode_request.load() == Mode::kBench) {
            if (g_ctrl_owns_bus.load()) {
                releaseAll();               // never hand the CLI a live robot
                g_ctrl_owns_bus.store(false);
                g_primed = false;
                g_shaper.invalidate();
            }
            // Keep the attitude filter running while benched. It is a
            // complementary filter with a ~2 s time constant, so it needs to
            // have been watching gravity for a while before its output means
            // anything -- and the instant `run` is typed, the very first tick
            // feeds `up` to the policy. Letting the estimate converge only
            // AFTER the robot is already walking is exactly backwards. The
            // IMU is on its own I2C bus, so this touches nothing the CLI owns.
            if (g_imu) {
                imu::Sample warm{};
                g_imu->read(warm);
            }
            continue;
        }
        if (!g_ctrl_owns_bus.load()) {
            g_ctrl_owns_bus.store(true);
            g_dog = linkproto::Watchdog();
            g_hist = obs::History();
            memset(g_prev_action, 0, sizeof g_prev_action);
            g_primed = false;
            g_shaper.invalidate();
        }

        const float now_ms = static_cast<float>(t0) / 1000.0f;

        // -- drain the command mailbox (latest wins) -----------------------
        CommandMsg msg;
        while (xQueueReceive(g_cmd_mailbox, &msg, 0) == pdTRUE) {
            g_dog.accept(msg.cmd, now_ms);
        }
        const linkproto::LinkState state = g_dog.state(now_ms);

        // -- sense ---------------------------------------------------------
        const int64_t t_read0 = esp_timer_get_time();
        const uint16_t faults = readJoints(obs::kControlDt);
        const int64_t t_read1 = esp_timer_get_time();
        // read() == false means "no new report this tick"; the contract
        // (imu/imu.h) is that the caller REUSES the previous sample. Holding
        // it in a static and only overwriting on a successful read is that
        // reuse -- a fresh `Sample{}` per tick would feed the policy
        // up = (0,0,0), which reads as a torso in freefall on its side and is
        // an observation training never produced. Harmless under StubImu
        // (always true), live the moment a real driver lands.
        static imu::Sample s_held{{0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}, 0,
                                  false};
        if (g_imu) {
            imu::Sample fresh{};
            if (g_imu->read(fresh)) s_held = fresh;
        }
        const imu::Sample& s = s_held;
        const int64_t t_imu1 = esp_timer_get_time();

        // Pack voltage rides in on the servo feedback -- the board has no ADC
        // (main/board.h). It is already sitting in g_fb every tick, so the
        // guard is fed every tick and does its own debouncing in ticks; the
        // old 1 Hz decimation bought nothing and only slowed the trip.
        // 0 == "no servo answered", which the guard must not read as 0 volts.
        uint8_t fresh_dv = 0;
        for (int i = 0; i < obs::kNumJoints; ++i) {
            if (g_fb_ok[i]) { fresh_dv = g_fb[i].voltage_dv; break; }
        }
        if (fresh_dv != 0) vbat_dv = fresh_dv;
        g_batt.update(fresh_dv, now_ms);

        // -- safety --------------------------------------------------------
        // The pack guard outranks the link: an operator holding a live command
        // cannot keep the robot walking on a flat pack, and once the guard has
        // released torque nothing here re-engages it (the state is latched
        // inside the guard, so the `if (!g_torque_on) engageAll()` below is
        // unreachable while it holds).
        const bool batt_limp = g_batt.torqueMustRelease();
        const linkproto::LinkState rep_state =
            g_batt.latched() ? (batt_limp ? linkproto::LinkState::kLowBattSafe
                                          : linkproto::LinkState::kLowBattLand)
                             : state;
        const bool limp = (batt_limp ||
                           state == linkproto::LinkState::kRelax ||
                           state == linkproto::LinkState::kEstop);
        if (limp) {
            if (g_torque_on) releaseAll();
            // Torque is off, so the joints go wherever gravity and the
            // operator put them; the shaper must reseed from MEASURED q on
            // re-engage or the first shaped target would be a jump.
            g_shaper.invalidate();
            publish(rep_state, faults, s.up[2], vbat_dv,
                    static_cast<uint32_t>(esp_timer_get_time() - t0));
            continue;
        }
        if (!g_torque_on) engageAll();

        // -- observe -------------------------------------------------------
        float vx = 0.0f, wz = 0.0f;
        g_dog.command(now_ms, vx, wz);
        // Landing on a flat pack: stop travelling, whatever was commanded.
        if (g_batt.latched()) { vx = 0.0f; wz = 0.0f; }
        // clock_stand_freeze runs generated per-policy: a policy trained
        // with the frozen-at-stand clock must see it here too, or the
        // stand obs oscillate in a way training never produced. vy and
        // lift are not commanded on hardware yet, so |vx|,|wz| is the
        // whole plain-stand gate.
        if (!obs::kClockStandFreeze
            || fabsf(vx) > 0.05f || fabsf(wz) > 0.05f) {
            g_clock.advance(obs::kControlDt);
        }

        obs::Inputs in{};
        memcpy(in.q, g_q, sizeof g_q);
        memcpy(in.dq, g_dq, sizeof g_dq);
        memcpy(in.up, s.up, sizeof in.up);
        memcpy(in.gyro, s.gyro, sizeof in.gyro);
        memcpy(in.prev_action, g_prev_action, sizeof g_prev_action);
        in.phase = g_clock.phase();
        // ext_cmd channel layout (walker_env.set_command): vx, vy, wz, crouch,
        // lift, foot_dx, foot_dz -- defaults (0,0,0,1,0,0,0).
        in.cmd[0] = vx;
        in.cmd[2] = wz;
        // Crouch height. 1.0 (full height) until the pack guard trips, then a
        // ramp down to walker_env's crouch_range[0] -- the lowest stance the
        // policy was actually trained to hold, so the landing stays inside the
        // training distribution instead of inventing a pose on a sagging rail.
        if (obs::kNumCmd > 3) in.cmd[3] = g_batt.crouch();
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

        for (int i = 0; i < obs::kNumJoints; ++i) {
            g_target_steps[i] = obs::angleToSteps(i, angle[i], g_cal);
            // g_fb[i].position holds the last servo that answered; a joint
            // that has never answered reads 0, the error saturates and the
            // speed clamps to max -- exactly the legacy behaviour.
            g_target_speed[i] =
                obs::goalSpeedSteps(g_target_steps[i], g_fb[i].position);
        }
        const int64_t t_wr0 = esp_timer_get_time();
        if (g_bus) {
            if (shape_hz > 0.0f) {
                g_bus->syncWritePositions(servoIds(), g_target_steps,
                                          g_target_speed, obs::kNumJoints);
            } else {
                g_bus->syncWritePositions(servoIds(), g_target_steps,
                                          obs::kNumJoints);
            }
        }
        const int64_t t_wr1 = esp_timer_get_time();

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
        publish(rep_state, faults, s.up[2], vbat_dv, took);
    }
}

}  // namespace

bool g_cal_from_nvs = false;
obs::Calibration& calibration() { return g_cal; }
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
