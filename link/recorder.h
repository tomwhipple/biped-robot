// Session recording: every frame in both directions, as JSONL.
//
// A hardware take is only worth as much as what can be replayed against a sim
// eval afterwards, and "what did the operator actually command" is exactly the
// thing nobody writes down at the time. One record per command frame and one
// per telemetry beacon, with a monotonic timestamp, into hw_sessions/ --
// gitignored, and already where sim/sil_twin.py --record puts its video.
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

    void maybeFlush(double now_ms);
};

}  // namespace bimo

#endif  // BIMO_LINK_RECORDER_H_
