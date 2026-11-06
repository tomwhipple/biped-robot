// The console core as a C library, for the Qt console (link/console/,
// Python, ctypes).
//
// Nothing here decides anything. Every rule -- arming, the E-stop latch, the
// home frames, the handover, the short-vs-long frame -- is link/client.h's,
// called as is, so the Qt console and bimo_gui put the same bytes on the wire
// for the same intent (client.h's opening argument). This file only flattens
// the C++ objects into calls a ctypes binding can make: an opaque handle, ints
// and floats, and one fixed-layout status struct.
//
// The status struct is mirrored field for field in link/console/core.py,
// which refuses to load a library whose bimo_status_size() or
// bimo_api_version() disagrees with it. Append fields; never reorder.
#ifndef BIMO_LINK_CONSOLE_API_H_
#define BIMO_LINK_CONSOLE_API_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BIMO_API_VERSION 1
#define BIMO_NUM_JOINTS 17
#define BIMO_NUM_AXES 6

typedef struct BimoConsole BimoConsole;

typedef struct {
    // the link
    int32_t readonly;       // no transmit socket (Link::readonly)
    int32_t has_dest;       // a robot address to take control to
    int32_t watch_on;       // relaying beacons to a watcher
    int32_t have_tlm;       // any beacon decoded since (re)linking
    int32_t link_lost;      // no beacon for kTlmLostMs
    uint32_t seq, sent, tlm_count, tlm_bad, watched, last_len;
    double tlm_age_ms;      // -1 before the first beacon
    double rate_hz;         // beacons in the last second
    int64_t lag;            // our seq minus the robot's echo
    // the latest beacon
    uint32_t seq_echo, servo_err;
    int32_t state;          // linkproto::LinkState
    int32_t diag;           // linkproto::telemetryDiag(), 0 unless BENCH
    float vbat_v, up_z, vx_est, wz_est, up_x, up_y;
    int32_t loop_late_pct, n_joints, have_att;
    uint64_t t_us;          // robot UTC microseconds, 0 = not synced
    float joints[BIMO_NUM_JOINTS];   // radians, bus order
    // the operator's intent
    int32_t armed, estop, want_pose, want_att, home_frames, moving;
    int32_t held[BIMO_NUM_AXES];
    float crouch, lift, foot_dx, foot_dz;
    float speed_vx, speed_vy, speed_wz;
    // what the last tick put on the wire
    float vx, vy, wz;
    float sent_crouch, sent_lift, sent_foot_dx, sent_foot_dz;
    uint32_t flags;
    // the console recording
    int32_t rec_active;
    uint32_t rec_cmd, rec_tlm, rec_bench;
    // twin-in-the-loop
    int32_t tee_on;
    uint32_t teed;
} BimoStatus;

int bimo_api_version(void);
size_t bimo_status_size(void);
double bimo_now_ms(void);

BimoConsole* bimo_new(void);
void bimo_free(BimoConsole* c);

// -- the link ------------------------------------------------------------
// 1 on success, 0 on failure (errno says why).
int bimo_open(BimoConsole* c, const char* host, int cmd_port, int tlm_port);
// The observer: binds the beacon port, opens NO transmit socket. `host` may
// be null; it is only where a later take-control commands.
int bimo_listen(BimoConsole* c, int tlm_port, const char* host, int cmd_port);
// bimo::relinkDisarming.
int bimo_relink(BimoConsole* c, const char* host, int cmd_port, int tlm_port,
                int observe);
// HOST[:PORT]; `default_port` when the spec has none. 0 on a bad spec.
int bimo_set_watch(BimoConsole* c, const char* spec, int default_port);
void bimo_clear_watch(BimoConsole* c);
// Twin-in-the-loop: a copy of every command frame to a sim (Link::tee_to).
// 0 on a non-dotted-quad host.
int bimo_set_tee(BimoConsole* c, const char* host, int port);
void bimo_clear_tee(BimoConsole* c);
void bimo_close(BimoConsole* c);

// One tick: drain the beacon, follow the robot (arm sync, home answer), send
// the intent, record. The caller calls it at the send rate, on the thread
// that draws, so a frozen window sends nothing.
void bimo_tick(BimoConsole* c, double now_ms);
void bimo_status(const BimoConsole* c, double now_ms, BimoStatus* out);
// Drain beacons only, sending nothing: for a console on its way out that is
// still waiting to hear a stop confirmed.
void bimo_poll(BimoConsole* c, double now_ms);

// bimo::toggleControl. 1 if it toggled, 0 if control could not be taken;
// `msg` says which.
int bimo_toggle_control(BimoConsole* c, const char* host, double now_ms,
                        char* msg, size_t cap);
// bimo::leaveDisarmed: blocks frames * gap_ms.
void bimo_leave_disarmed(BimoConsole* c, int frames, int gap_ms);

// -- the operator's intent -----------------------------------------------
// The console's last message (Intent::note); "" when none.
const char* bimo_note(const BimoConsole* c);
void bimo_say(BimoConsole* c, const char* msg);
// 1 if the press was taken, 0 if refused (the note says why).
int bimo_press(BimoConsole* c, int axis, double now_ms);
void bimo_release(BimoConsole* c, int axis);
void bimo_release_all(BimoConsole* c);
void bimo_toggle_arm(BimoConsole* c, double now_ms);
void bimo_stand(BimoConsole* c, const char* why);
// The classic, commander's E-stop latch (Intent::fireEstop). The out-of-band
// operator stop is not here: link/console/stops.py sends tools/estop.py's
// frames on a socket of its own.
void bimo_estop(BimoConsole* c);
void bimo_home(BimoConsole* c, double now_ms);
// Snapped into the trained draw (Ext::clamp).
void bimo_set_ext(BimoConsole* c, float crouch, float lift, float foot_dx,
                  float foot_dz);
void bimo_reset_ext(BimoConsole* c);
void bimo_set_speeds(BimoConsole* c, float vx, float vy, float wz);
// Telemetry requests (kFlagPose, kFlagAtt): ask, command nothing.
void bimo_request(BimoConsole* c, int pose, int att);
const char* bimo_motion_text(const BimoConsole* c);

// -- the console recording (link/recorder.h, the JSONL schema) ------------
int bimo_rec_start(BimoConsole* c, const char* dir, const char* host,
                   double now_ms, char* err, size_t cap);
void bimo_rec_stop(BimoConsole* c);
const char* bimo_rec_path(const BimoConsole* c);
void bimo_rec_cam(BimoConsole* c, double now_ms, int pane, const char* src,
                  const char* url);
void bimo_rec_bench(BimoConsole* c, double now_ms, const char* datagram,
                    size_t n);

// -- names ---------------------------------------------------------------
const char* bimo_state_name(int state);
const char* bimo_diag_reason(int diag);
int bimo_diag_arm_result(int diag);
const char* bimo_fault_ids(uint32_t servo_err, char* buf, size_t cap);
const char* bimo_axis_name(int axis);

// -- the keymap (~/.bimo/keymap.conf, link/keymap.h) ----------------------
int bimo_action_count(void);
int bimo_motion_actions(void);
const char* bimo_action_name(int action);
const char* bimo_action_label(int action);
int bimo_action_from_name(const char* name);
const char* bimo_key_name(int key);
int bimo_key_from_name(const char* name);
int bimo_keymap_key(const BimoConsole* c, int action);
int bimo_keymap_action_for(const BimoConsole* c, int key);
void bimo_keymap_bind(BimoConsole* c, int action, int key);
void bimo_keymap_reset(BimoConsole* c);
int bimo_keymap_load(BimoConsole* c, const char* path, char* err, size_t cap);
int bimo_keymap_save(const BimoConsole* c, const char* path, char* err,
                     size_t cap);
int bimo_default_keymap_path(char* out, size_t cap);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // BIMO_LINK_CONSOLE_API_H_
