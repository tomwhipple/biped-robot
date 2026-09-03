// Session recording: every frame in both directions, as JSONL.
//
// A hardware take is only worth as much as what can be replayed against a sim
// eval afterwards, and "what did the operator actually command" is exactly the
// thing nobody writes down at the time. One record per command frame and one
// per telemetry beacon, with a monotonic timestamp, into hw_sessions/ --
// gitignored, and already where sim/sil_twin.py --record puts its video.
//
// TIME (2026-09-02). Three clocks appear in a record and they are kept apart
// on purpose:
//   t          seconds since this recording started, CLOCK_MONOTONIC. The
//              x axis for anything plotted from one file.
//   utc        host wall clock at the record, epoch seconds, CLOCK_REALTIME.
//              NTP-disciplined on mira (chrony), and the same time base
//              tools/cam_record.sh burns into every webcam frame -- so a
//              beacon and a video frame are joined by subtracting.
//   robot_utc  the robot's own stamp from the beacon (Telemetry.t_us), its
//              SNTP clock; null until it has synced, or from a robot flashed
//              before the field existed. On a pose frame it is the instant
//              the joints were read. utc - robot_utc is link latency plus
//              the two clocks' disagreement, and is what to look at when a
//              filmstrip and a joint trace do not agree.
// The header says whether the host clock was NTP-synchronised when the file
// was opened (the kernel's own flag, via ntp_gettime), because a `utc`
// column from an unsynced host is a number, not a time.
//
// Emitting rather than parsing, so firmware/host/sil_json.h (a reader, and a
// user of std::string) is no help here; this is thirty lines of snprintf.
#ifndef BIMO_LINK_RECORDER_H_
#define BIMO_LINK_RECORDER_H_

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "client.h"

namespace bimo {

struct Recorder {
    FILE* f = nullptr;
    char path[512] = "";
    uint32_t n_cmd = 0, n_tlm = 0;
    double t0_ms = 0.0;
    double last_flush_ms = 0.0;

    bool active() const { return f != nullptr; }

    // Opens <dir>/<UTC stamp>-<host>.jsonl, creating <dir>. Returns false and
    // fills `err` if it cannot.
    bool start(const char* dir, const char* host, double now_ms, char* err,
               size_t cap);
    void stop();

    void command(double now_ms, uint32_t seq, float vx, float vy, float wz,
                 const Ext& ext, uint8_t flags, size_t len);
    void telemetry(double now_ms, const Telemetry& t);

    // Host wall clock, epoch seconds (CLOCK_REALTIME). Public so a test can
    // bracket a record with it.
    static double wallNow();
    // "ntp" when the kernel says the clock is disciplined, "unsynced" when it
    // says it is not, "unknown" where there is no way to ask (macOS).
    static const char* hostClockState(long* maxerror_us);

    void maybeFlush(double now_ms);
};

}  // namespace bimo

#endif  // BIMO_LINK_RECORDER_H_
