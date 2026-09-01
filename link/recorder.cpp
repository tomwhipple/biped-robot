#include "recorder.h"

#include <string.h>
#include <sys/stat.h>
#include <time.h>

namespace bimo {

bool Recorder::start(const char* dir, const char* host, double now_ms,
                     char* err, size_t cap) {
    if (active()) return true;
    mkdir(dir, 0755);
    time_t now = time(nullptr);
    tm utc;
    gmtime_r(&now, &utc);
    char stamp[32];
    strftime(stamp, sizeof stamp, "%Y-%m-%dT%H-%M-%SZ", &utc);
    snprintf(path, sizeof path, "%s/%s-%s.jsonl", dir, stamp, host);
    f = fopen(path, "w");
    if (f == nullptr) {
        snprintf(err, cap, "cannot write %s", path);
        path[0] = 0;
        return false;
    }
    n_cmd = 0;
    n_tlm = 0;
    t0_ms = now_ms;
    last_flush_ms = now_ms;
    // A header record, so a reader knows what it is holding without guessing
    // from the first data line.
    fprintf(f, "{\"rec\":\"header\",\"host\":\"%s\",\"utc\":\"%s\"}\n", host,
            stamp);
    return true;
}

void Recorder::stop() {
    if (f == nullptr) return;
    fclose(f);
    f = nullptr;
}

void Recorder::command(double now_ms, uint32_t seq, float vx, float vy,
                       float wz, const Ext& ext, uint8_t flags, size_t len) {
    if (f == nullptr) return;
    fprintf(f,
            "{\"t\":%.3f,\"dir\":\"cmd\",\"seq\":%u,\"len\":%zu,\"flags\":%u,"
            "\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f,\"crouch\":%.3f,"
            "\"lift\":%.3f,\"foot_dx\":%.3f,\"foot_dz\":%.3f}\n",
            (now_ms - t0_ms) / 1000.0, seq, len, flags,
            static_cast<double>(vx), static_cast<double>(vy),
            static_cast<double>(wz), static_cast<double>(ext.crouch),
            static_cast<double>(ext.lift), static_cast<double>(ext.foot_dx),
            static_cast<double>(ext.foot_dz));
    ++n_cmd;
}

void Recorder::telemetry(double now_ms, const Telemetry& t) {
    if (f == nullptr) return;
    fprintf(f,
            "{\"t\":%.3f,\"dir\":\"tlm\",\"state\":\"%s\",\"seq_echo\":%u,"
            "\"vbat\":%.3f,\"up_z\":%.3f,\"vx_est\":%.3f,\"wz_est\":%.3f,"
            "\"servo_err\":%u,\"late_pct\":%u}\n",
            (now_ms - t0_ms) / 1000.0, stateName(t.state), t.seq_echo,
            static_cast<double>(t.vbat_v), static_cast<double>(t.up_z),
            static_cast<double>(t.vx_est), static_cast<double>(t.wz_est),
            t.servo_err, t.loop_late_pct);
    ++n_tlm;
}

void Recorder::maybeFlush(double now_ms) {
    if (f == nullptr) return;
    if (now_ms - last_flush_ms < 1000.0) return;
    fflush(f);
    last_flush_ms = now_ms;
}

}  // namespace bimo
