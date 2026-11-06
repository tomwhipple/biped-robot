#include "console_api.h"

#include <stdio.h>

#include "client.h"
#include "keymap.h"
#include "recorder.h"

using bimo::Ext;
using bimo::Intent;
using bimo::Keymap;
using bimo::Link;
using bimo::Recorder;
using bimo::Speeds;
using linkproto::LinkState;

struct BimoConsole {
    Link link;
    Intent in;
    Speeds speeds;
    Keymap keys{};
    Recorder rec;
    // what the last tick put on the wire
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    Ext ext_sent;
    uint8_t flags = 0;
    uint32_t recorded = 0;    // beacons already written to the recording
};

namespace {

bool validAxis(int axis) { return axis >= 0 && axis < bimo::kAxCount; }

}  // namespace

extern "C" {

int bimo_api_version(void) { return BIMO_API_VERSION; }
size_t bimo_status_size(void) { return sizeof(BimoStatus); }
double bimo_now_ms(void) { return bimo::nowMs(); }

BimoConsole* bimo_new(void) { return new BimoConsole(); }

void bimo_free(BimoConsole* c) {
    if (c == nullptr) return;
    if (c->rec.active()) c->rec.stop();
    c->link.close();
    delete c;
}

int bimo_open(BimoConsole* c, const char* host, int cmd_port, int tlm_port) {
    c->in.readonly = false;
    return c->link.open(host, cmd_port, tlm_port) ? 1 : 0;
}

int bimo_listen(BimoConsole* c, int tlm_port, const char* host, int cmd_port) {
    const bool ok = c->link.listen(tlm_port, host, cmd_port);
    c->in.readonly = true;
    return ok ? 1 : 0;
}

int bimo_relink(BimoConsole* c, const char* host, int cmd_port, int tlm_port,
                int observe) {
    c->recorded = 0;
    return bimo::relinkDisarming(c->link, c->in, host, cmd_port, tlm_port,
                                 observe != 0)
               ? 1
               : 0;
}

int bimo_set_watch(BimoConsole* c, const char* spec, int default_port) {
    char host[64] = "";
    int port = default_port;
    if (!bimo::splitHostPort(spec, host, sizeof host, port)) return 0;
    return c->link.setWatch(host, port) ? 1 : 0;
}

void bimo_clear_watch(BimoConsole* c) { c->link.clearWatch(); }

int bimo_set_tee(BimoConsole* c, const char* host, int port) {
    return c->link.setTee(host, port) ? 1 : 0;
}

void bimo_clear_tee(BimoConsole* c) { c->link.clearTee(); }
void bimo_close(BimoConsole* c) { c->link.close(); }

void bimo_tick(BimoConsole* c, double now_ms) {
    c->link.poll(now_ms);
    c->in.syncToRobot(c->link, now_ms);
    c->in.checkHome(c->link, now_ms);
    bimo::sendIntent(c->link, c->in, c->speeds, c->vx, c->vy, c->wz,
                     c->ext_sent, c->flags);
    // An observer put nothing on the wire, so it records telemetry only (the
    // same rule as bimo_gui's tick).
    if (!c->link.readonly) {
        c->rec.command(now_ms, c->link.seq - 1, c->vx, c->vy, c->wz,
                       c->ext_sent, c->flags, c->link.last_len);
    }
    if (c->link.tlm_count != c->recorded) {
        c->recorded = c->link.tlm_count;
        c->rec.telemetry(now_ms, c->link.tlm);
    }
    c->rec.maybeFlush(now_ms);
}

void bimo_poll(BimoConsole* c, double now_ms) { c->link.poll(now_ms); }

void bimo_status(const BimoConsole* c, double now_ms, BimoStatus* o) {
    *o = BimoStatus{};
    const Link& l = c->link;
    o->readonly = l.readonly;
    o->has_dest = l.has_dest;
    o->watch_on = l.watch_on;
    o->have_tlm = l.have_tlm;
    o->link_lost = l.linkLost(now_ms);
    o->seq = l.seq;
    o->sent = l.sent;
    o->tlm_count = l.tlm_count;
    o->tlm_bad = l.tlm_bad;
    o->watched = l.watched;
    o->last_len = static_cast<uint32_t>(l.last_len);
    o->tlm_age_ms = l.tlmAgeMs(now_ms);
    o->rate_hz = l.rate_hz;
    o->lag = l.lag();
    const linkproto::Telemetry& t = l.tlm;
    o->seq_echo = t.seq_echo;
    o->servo_err = t.servo_err;
    o->state = static_cast<int32_t>(t.state);
    o->diag = linkproto::telemetryDiag(t);
    o->vbat_v = t.vbat_v;
    o->up_z = t.up_z;
    o->vx_est = t.vx_est;
    o->wz_est = t.wz_est;
    o->up_x = t.up_x;
    o->up_y = t.up_y;
    o->loop_late_pct = t.loop_late_pct;
    o->n_joints = t.n_joints;
    o->have_att = t.have_att;
    o->t_us = t.t_us;
    for (size_t j = 0; j < linkproto::kNumJoints && j < BIMO_NUM_JOINTS; ++j) {
        o->joints[j] = t.joints[j];
    }
    const Intent& in = c->in;
    o->armed = in.armed;
    o->estop = in.estop;
    o->want_pose = in.want_pose;
    o->want_att = in.want_att;
    o->home_frames = in.home_frames;
    o->moving = in.moving();
    for (int a = 0; a < BIMO_NUM_AXES; ++a) o->held[a] = in.held[a];
    o->crouch = in.ext.crouch;
    o->lift = in.ext.lift;
    o->foot_dx = in.ext.foot_dx;
    o->foot_dz = in.ext.foot_dz;
    o->speed_vx = c->speeds.vx;
    o->speed_vy = c->speeds.vy;
    o->speed_wz = c->speeds.wz;
    o->vx = c->vx;
    o->vy = c->vy;
    o->wz = c->wz;
    o->sent_crouch = c->ext_sent.crouch;
    o->sent_lift = c->ext_sent.lift;
    o->sent_foot_dx = c->ext_sent.foot_dx;
    o->sent_foot_dz = c->ext_sent.foot_dz;
    o->flags = c->flags;
    o->rec_active = c->rec.active();
    o->rec_cmd = c->rec.n_cmd;
    o->rec_tlm = c->rec.n_tlm;
    o->rec_bench = c->rec.n_bench;
    o->tee_on = l.tee_on;
    o->teed = l.teed;
}

int bimo_toggle_control(BimoConsole* c, const char* host, double now_ms,
                        char* msg, size_t cap) {
    return bimo::toggleControl(c->link, c->in, host, now_ms, msg, cap) ? 1 : 0;
}

void bimo_leave_disarmed(BimoConsole* c, int frames, int gap_ms) {
    bimo::leaveDisarmed(c->link, frames, gap_ms);
}

const char* bimo_note(const BimoConsole* c) { return c->in.note; }
void bimo_say(BimoConsole* c, const char* msg) { c->in.say(msg); }

int bimo_press(BimoConsole* c, int axis, double now_ms) {
    if (!validAxis(axis)) return 0;
    return c->in.press(axis, now_ms) ? 1 : 0;
}

void bimo_release(BimoConsole* c, int axis) {
    if (validAxis(axis)) c->in.release(axis);
}

void bimo_release_all(BimoConsole* c) { c->in.releaseAll(); }
void bimo_toggle_arm(BimoConsole* c, double now_ms) { c->in.toggleArm(now_ms); }
void bimo_stand(BimoConsole* c, const char* why) { c->in.stand(why); }
void bimo_estop(BimoConsole* c) { c->in.fireEstop(); }
void bimo_home(BimoConsole* c, double now_ms) { c->in.requestHome(now_ms); }

void bimo_set_ext(BimoConsole* c, float crouch, float lift, float foot_dx,
                  float foot_dz) {
    c->in.ext.crouch = crouch;
    c->in.ext.lift = lift;
    c->in.ext.foot_dx = foot_dx;
    c->in.ext.foot_dz = foot_dz;
    c->in.ext.clamp();
}

void bimo_reset_ext(BimoConsole* c) {
    c->in.ext.reset();
    c->in.say("extended channels back to defaults (14 B frames)");
}

void bimo_set_speeds(BimoConsole* c, float vx, float vy, float wz) {
    c->speeds.vx = vx;
    c->speeds.vy = vy;
    c->speeds.wz = wz;
}

void bimo_request(BimoConsole* c, int pose, int att) {
    c->in.want_pose = pose != 0;
    c->in.want_att = att != 0;
}

const char* bimo_motion_text(const BimoConsole* c) {
    return bimo::motionText(c->in);
}

int bimo_rec_start(BimoConsole* c, const char* dir, const char* host,
                   double now_ms, char* err, size_t cap) {
    return c->rec.start(dir, host, now_ms, err, cap) ? 1 : 0;
}

void bimo_rec_stop(BimoConsole* c) { c->rec.stop(); }
const char* bimo_rec_path(const BimoConsole* c) { return c->rec.path; }

void bimo_rec_cam(BimoConsole* c, double now_ms, int pane, const char* src,
                  const char* url) {
    c->rec.cam(now_ms, pane, src, url);
}

void bimo_rec_bench(BimoConsole* c, double now_ms, const char* datagram,
                    size_t n) {
    c->rec.bench(now_ms, datagram, n);
}

const char* bimo_state_name(int state) {
    if (state < 0 || state > static_cast<int>(LinkState::kMaxState)) return "?";
    return bimo::stateName(static_cast<LinkState>(state));
}

const char* bimo_diag_reason(int diag) {
    return linkproto::diagReason(static_cast<uint8_t>(diag & 0xFF));
}

int bimo_diag_arm_result(int diag) {
    return static_cast<int>(
        linkproto::diagArmResult(static_cast<uint8_t>(diag & 0xFF)));
}

const char* bimo_fault_ids(uint32_t servo_err, char* buf, size_t cap) {
    return bimo::faultIds(servo_err, buf, cap);
}

const char* bimo_axis_name(int axis) {
    return validAxis(axis) ? bimo::axisName(axis) : "?";
}

int bimo_action_count(void) { return bimo::kActCount; }
int bimo_motion_actions(void) { return bimo::kMotionActions; }
const char* bimo_action_name(int action) { return bimo::actionName(action); }
const char* bimo_action_label(int action) { return bimo::actionLabel(action); }
int bimo_action_from_name(const char* name) {
    return bimo::actionFromName(name);
}
const char* bimo_key_name(int key) { return bimo::keyName(key); }
int bimo_key_from_name(const char* name) { return bimo::keyFromName(name); }

int bimo_keymap_key(const BimoConsole* c, int action) {
    if (action < 0 || action >= bimo::kActCount) return bimo::kKeyNone;
    return c->keys.key[action];
}

int bimo_keymap_action_for(const BimoConsole* c, int key) {
    return c->keys.actionFor(key);
}

void bimo_keymap_bind(BimoConsole* c, int action, int key) {
    if (action >= 0 && action < bimo::kActCount) c->keys.bind(action, key);
}

void bimo_keymap_reset(BimoConsole* c) { c->keys.reset(); }

int bimo_keymap_load(BimoConsole* c, const char* path, char* err, size_t cap) {
    return c->keys.load(path, err, cap) ? 1 : 0;
}

int bimo_keymap_save(const BimoConsole* c, const char* path, char* err,
                     size_t cap) {
    return c->keys.save(path, err, cap) ? 1 : 0;
}

int bimo_default_keymap_path(char* out, size_t cap) {
    return bimo::defaultKeymapPath(out, cap) ? 1 : 0;
}

}  // extern "C"
