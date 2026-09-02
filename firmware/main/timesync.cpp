#include "timesync.h"

#include <atomic>
#include <string.h>
#include <sys/time.h>

#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_timer.h"

namespace robot {
namespace {

// Written by the SNTP callback (lwIP's tcpip task), read by the beacon tasks
// and the CLI. Plain atomics: none of these is wide enough to tear, and the
// consumer that matters (the beacon) only needs "synced yet?" plus a
// gettimeofday it does itself.
std::atomic<bool> g_synced{false};
std::atomic<bool> g_started{false};
std::atomic<uint32_t> g_syncs{0};
std::atomic<int64_t> g_last_sync_boot_us{0};
std::atomic<uint64_t> g_last_sync_epoch_us{0};
std::atomic<int32_t> g_last_step_ms{0};

constexpr char kFallback[] = "pool.ntp.org";

// esp_sntp_time_cb_t: the reply has already been applied to the system clock
// when this runs (immediate mode). `tv` is the new time; what it replaced is
// not handed over, so the step is measured against where the free-running
// clock would have been: the previous reply's time plus the crystal-clocked
// microseconds since. That is the number an operator wants -- "the clock
// jumped 340 ms" (link or server trouble) versus "it moved 1 ms" (normal).
void onSync(timeval* tv) {
    const int64_t now_boot = esp_timer_get_time();
    const uint64_t now_epoch =
        static_cast<uint64_t>(tv->tv_sec) * 1000000ull +
        static_cast<uint64_t>(tv->tv_usec);
    if (g_synced.load()) {
        const uint64_t predicted =
            g_last_sync_epoch_us.load() +
            static_cast<uint64_t>(now_boot - g_last_sync_boot_us.load());
        const int64_t step_us = static_cast<int64_t>(now_epoch - predicted);
        g_last_step_ms.store(static_cast<int32_t>(step_us / 1000));
    }
    g_last_sync_epoch_us.store(now_epoch);
    g_last_sync_boot_us.store(now_boot);
    g_syncs.fetch_add(1);
    g_synced.store(true);
}

}  // namespace

void startTimeSync() {
    esp_sntp_config_t cfg =
        ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(1, ESP_SNTP_SERVER_LIST(kFallback));
    cfg.start = false;               // GOT_IP starts it (timeSyncStart)
    cfg.wait_for_sync = false;       // nobody blocks on it; the beacon says 0
    cfg.smooth_sync = false;         // step, and report the step honestly
    cfg.server_from_dhcp = true;     // option 42 from the router
    cfg.renew_servers_after_new_IP = true;
    cfg.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    cfg.index_of_first_server = 1;   // DHCP's server takes slot 0; the
                                     // fallback above is re-set into slot 1
    cfg.sync_cb = onSync;
    esp_netif_sntp_init(&cfg);
}

void timeSyncStart() {
    if (g_started.exchange(true)) return;
    esp_netif_sntp_start();
}

uint64_t timeNowUs() {
    if (!g_synced.load()) return 0;
    timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ull +
           static_cast<uint64_t>(tv.tv_usec);
}

uint64_t timeFromBootUs(int64_t boot_us) {
    const uint64_t now = timeNowUs();
    if (now == 0) return 0;
    // epoch(boot_us) = epoch(now) - (boot(now) - boot_us). Both differences
    // are microseconds on the same crystal, so this is exact up to the
    // gettimeofday/esp_timer read skew (sub-microsecond).
    const int64_t age = esp_timer_get_time() - boot_us;
    if (age < 0 || static_cast<uint64_t>(age) > now) return now;
    return now - static_cast<uint64_t>(age);
}

void timeGetStatus(TimeStatus& out) {
    out.synced = g_synced.load();
    out.syncs = g_syncs.load();
    out.now_us = timeNowUs();
    out.since_sync_us =
        out.synced ? esp_timer_get_time() - g_last_sync_boot_us.load() : 0;
    out.last_step_ms = g_last_step_ms.load();
    out.started = g_started.load();
    // A DHCP-provided server is stored as an address, not a name; show
    // whichever form the slot holds.
    for (uint8_t i = 0; i < 2; ++i) {
        char* dst = (i == 0) ? out.server0 : out.server1;
        const size_t cap = sizeof out.server0;
        const char* name = esp_sntp_getservername(i);
        if (name != nullptr) {
            strncpy(dst, name, cap - 1);
            dst[cap - 1] = 0;
            continue;
        }
        dst[0] = 0;
        const ip_addr_t* a = esp_sntp_getserver(i);
        if (a != nullptr && !ip_addr_isany(a)) ipaddr_ntoa_r(a, dst, static_cast<int>(cap));
    }
}

}  // namespace robot
