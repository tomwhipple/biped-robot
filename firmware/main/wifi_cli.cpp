// The authenticated Wi-Fi CLI, ESP32 glue. See wifi_cli.h for the contract,
// components/wcli for the protocol and docs/wifi-cli.md for the threat model.
#include "wifi_cli.h"

#include <stdio.h>
#include <string.h>

#include "cli.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "flight_rec.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "link_task.h"
#include "linkproto/framing.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "shared.h"
#include "wcli/wcli.h"
#include "wifi_link.h"

static_assert(wcli::kMaxLine == linkproto::kMaxCliLine,
              "a Wi-Fi CLI line and a tether line have the same limit");

namespace robot {
namespace {

constexpr const char* kNs = "bimo";
constexpr const char* kKeyName = "wcli_key";

constexpr int kStackBytes = 4096;   // ESP-IDF stack depth is in bytes
StackType_t g_stack[kStackBytes];
StaticTask_t g_tcb;

StaticSemaphore_t g_mu_buf;
SemaphoreHandle_t g_mu = nullptr;
wcli::Server g_srv;          // under g_mu, always
uint32_t g_sessions_seen = 0;   // housekeeping only

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

// No-ops before startWifiCli() (nothing else runs yet then).
void lock() { if (g_mu != nullptr) xSemaphoreTake(g_mu, portMAX_DELAY); }
void unlock() { if (g_mu != nullptr) xSemaphoreGive(g_mu); }

// The hardware RNG: true random while the radio is on, which it is whenever
// a datagram can arrive (esp_random.h).
uint64_t freshNonce() {
    uint64_t v = 0;
    esp_fill_random(&v, sizeof v);
    return v;
}

void fingerprint(const uint8_t key[wcli::kKeyLen], char out[9]) {
    uint8_t d[wcli::kShaLen];
    wcli::sha256(key, wcli::kKeyLen, d);
    snprintf(out, 9, "%02x%02x%02x%02x", d[0], d[1], d[2], d[3]);
    wcli::wipe(d, sizeof d);
}

bool keyLoad(uint8_t key[wcli::kKeyLen]) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return false;
    size_t n = wcli::kKeyLen;
    const bool ok = nvs_get_blob(h, kKeyName, key, &n) == ESP_OK &&
                    n == wcli::kKeyLen;
    nvs_close(h);
    return ok;
}

struct NetTx : wcli::Tx {
    int sock = -1;
    void send(const wcli::Peer& to, const uint8_t* buf, size_t n) override {
        if (sock < 0) return;
        sockaddr_in a = {};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = lwip_htonl(to.ip);
        a.sin_port = lwip_htons(to.port);
        // Best effort: a full TX queue drops the datagram, and the client's
        // POLL/RESEND recovers it from the ring.
        lwip_sendto(sock, buf, n, 0, reinterpret_cast<sockaddr*>(&a), sizeof a);
    }
    bool submit(uint32_t seq, const char* line) override {
        return submitWifiCliLine(line, seq);
    }
    virtual ~NetTx() = default;
};
NetTx g_tx;   // the wcli task's (sock is its socket)

void wcliTask(void*) {
    int sock = -1;
    for (;;) {
        if (!wifiConnected()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (sock < 0) {
            sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
            // 10 ms: how promptly closed output chunks go out.
            timeval tv = {0, 10 * 1000};
            lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
            sockaddr_in addr = {};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);
            addr.sin_port = lwip_htons(wcli::kPort);
            if (lwip_bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof addr) < 0) {
                lwip_close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }
        // One byte more than the largest legal datagram, so an oversize one
        // arrives truncated to a length the decoder refuses.
        uint8_t rx[wcli::kMaxDatagram + 1];
        sockaddr_in from = {};
        socklen_t from_len = sizeof from;
        const int n = lwip_recvfrom(sock, rx, sizeof rx, 0,
                                    reinterpret_cast<sockaddr*>(&from), &from_len);
        lock();
        g_tx.sock = sock;
        if (n > 0) {
            const wcli::Peer p{lwip_ntohl(from.sin_addr.s_addr),
                               lwip_ntohs(from.sin_port)};
            g_srv.onDatagram(rx, static_cast<size_t>(n), p, nowMs(), freshNonce(),
                             g_tx);
        }
        g_srv.pump(g_tx, 8);
        unlock();
    }
}

// The sink for a Wi-Fi command's output: the flight recorder, then the
// client. When the client's window is full it waits -- like the tether's UART
// write when its buffer is full -- but only as wcli::Server::shouldStopWaiting
// allows: not for a client gone silent, never longer than kStallMs.
void wifiSay(const char* s) {
    flight::outputText(s, flightrec::kChanWifi);
    size_t n = strlen(s);
    uint32_t stalled_since = 0;
    while (n > 0) {
        lock();
        const uint32_t now = nowMs();
        const size_t k = g_srv.output(s, n, now);
        if (k == 0 && stalled_since != 0 &&
            g_srv.shouldStopWaiting(now - stalled_since, now)) {
            g_srv.setLagging();
        }
        unlock();
        if (k > 0) {
            s += k;
            n -= k;
            stalled_since = 0;
            continue;
        }
        if (stalled_since == 0) stalled_since = nowMs();
        vTaskDelay(pdMS_TO_TICKS(5));   // the wcli task drains and reads ACKs
    }
}

void endCommand(uint32_t seq, uint16_t flags) {
    const uint32_t t0 = nowMs();
    for (;;) {
        lock();
        const bool ok = g_srv.endCommand(seq, flags);
        const uint32_t now = nowMs();
        if (!ok && g_srv.shouldStopWaiting(now - t0, now)) g_srv.setLagging();
        unlock();
        if (ok) return;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

}  // namespace

void startWifiCli() {
    g_mu = xSemaphoreCreateMutexStatic(&g_mu_buf);
    uint8_t key[wcli::kKeyLen];
    if (keyLoad(key)) g_srv.setKey(key);   // before the task: no lock needed
    wcli::wipe(key, sizeof key);
    // Priority 1: BELOW housekeeping (2), the flight sender (3), the IMU
    // sampler (4) and the link tasks (5). Every datagram costs an HMAC before
    // it can be dropped, so a flood on port 4213 must only ever be able to
    // starve this task -- never the bench balance loop or the IMU. It runs
    // whenever housekeeping sleeps, which is most of every 10 ms.
    xTaskCreateStaticPinnedToCore(wcliTask, "wcli", kStackBytes, nullptr, 1,
                                  g_stack, &g_tcb, 0);
}

void wcliExecute(const char* line, uint32_t seq) {
    lock();
    const bool go = g_srv.beginCommand(seq);
    unlock();
    if (!go) return;   // cancelled by the client, or its session is gone

    // Bench-only, decided HERE (the housekeeping task is the mode's writer,
    // so nothing can arm the loop between this check and execute()).
    const bool armed = g_mode_request.load() == Mode::kRun ||
                       g_ctrl_owns_bus.load();
    const wcli::Decision d = wcli::classify(line, armed);

    // Every Wi-Fi line, refused or not, with its channel and seq. Static
    // buffers: housekeeping only, and its stack is what cli::execute needs.
    static char logged[wcli::kMaxLine + 1];
    wcli::redactForLog(line, logged, sizeof logged);
    flight::event(flightrec::Ev::kCmd, logged, flightrec::kChanWifi,
                  static_cast<int32_t>(seq));

    if (d.verdict != wcli::Verdict::kAllow) {
        static char msg[200];
        wcli::formatRefusal(d, line, msg, sizeof msg);
        wifiSay(msg);
        endCommand(seq, wcli::kOutRefused);
        return;
    }
    cli::execute(line, &wifiSay, /*remote=*/true);
    endCommand(seq, 0);
}

void wcliAsyncOutput(const char* text) {
    if (g_mu == nullptr) return;
    lock();
    const uint32_t now = nowMs();
    if (g_srv.listening(now)) {
        const size_t n = strlen(text);
        size_t k = g_srv.output(text, n, now);
        if (k < n) {
            // Never wait here: this runs inside poseTick/balance. Overwrite.
            g_srv.setLagging();
            k += g_srv.output(text + k, n - k, now);
        }
    }
    unlock();
}

void wcliHouseTick() {
    if (g_mu == nullptr) return;
    lock();
    if (!g_srv.flush()) {   // never waits: this runs between balance ticks
        g_srv.setLagging();
        g_srv.flush();
    }
    const uint32_t sessions = g_srv.counters().sessions;
    const wcli::Peer p = g_srv.peer();
    unlock();
    if (sessions != g_sessions_seen) {
        g_sessions_seen = sessions;
        char msg[48];
        snprintf(msg, sizeof msg, "wcli session from %u.%u.%u.%u:%u",
                 static_cast<unsigned>(p.ip >> 24), static_cast<unsigned>((p.ip >> 16) & 0xFF),
                 static_cast<unsigned>((p.ip >> 8) & 0xFF), static_cast<unsigned>(p.ip & 0xFF),
                 static_cast<unsigned>(p.port));
        flight::event(flightrec::Ev::kWcli, msg, static_cast<int32_t>(p.ip),
                      static_cast<int32_t>(p.port));
    }
}

void wcliGetStatus(WcliStatus& out) {
    out = WcliStatus{};
    uint8_t key[wcli::kKeyLen];
    out.have_key = keyLoad(key);
    if (out.have_key) fingerprint(key, out.fingerprint);
    wcli::wipe(key, sizeof key);
    if (g_mu == nullptr) return;
    lock();
    const uint32_t now = nowMs();
    out.session = g_srv.established();
    out.peer_ip = g_srv.peer().ip;
    out.peer_port = g_srv.peer().port;
    out.last_seq = g_srv.lastSeq();
    out.state = static_cast<uint8_t>(g_srv.state());
    out.out_next = g_srv.outNext();
    out.oldest = g_srv.oldestHeld();
    out.heard_ms_ago = g_srv.heardMsAgo(now);
    const wcli::Counters& c = g_srv.counters();
    out.rx_ok = c.rx_ok;
    out.rx_bad = c.rx_bad;
    out.rx_nokey = c.rx_nokey;
    out.rx_stale = c.rx_stale;
    out.rx_wrong_peer = c.rx_wrong_peer;
    out.replays = c.replays;
    out.sessions = c.sessions;
    out.cmds = c.cmds;
    out.busy = c.busy;
    out.lost_chunks = c.lost_chunks;
    unlock();
}

bool wcliKeySet(const char* hex, char fingerprint_out[9]) {
    if (strlen(hex) != 2 * wcli::kKeyLen) return false;
    uint8_t key[wcli::kKeyLen];
    for (size_t i = 0; i < wcli::kKeyLen; ++i) {
        uint8_t v = 0;
        for (int j = 0; j < 2; ++j) {
            const char c = hex[2 * i + static_cast<size_t>(j)];
            uint8_t nib;
            if (c >= '0' && c <= '9') nib = static_cast<uint8_t>(c - '0');
            else if (c >= 'a' && c <= 'f') nib = static_cast<uint8_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') nib = static_cast<uint8_t>(c - 'A' + 10);
            else { wcli::wipe(key, sizeof key); return false; }
            v = static_cast<uint8_t>((v << 4) | nib);
        }
        key[i] = v;
    }
    nvs_handle_t h;
    bool ok = nvs_open(kNs, NVS_READWRITE, &h) == ESP_OK;
    if (ok) {
        ok = nvs_set_blob(h, kKeyName, key, sizeof key) == ESP_OK &&
             nvs_commit(h) == ESP_OK;
        nvs_close(h);
    }
    if (ok) {
        fingerprint(key, fingerprint_out);
        lock();
        g_srv.setKey(key);
        unlock();
    }
    wcli::wipe(key, sizeof key);
    return ok;
}

bool wcliKeyClear() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    nvs_erase_key(h, kKeyName);   // absent is fine
    const bool ok = nvs_commit(h) == ESP_OK;
    nvs_close(h);
    lock();
    g_srv.clearKey();
    unlock();
    return ok;
}

void wcliDropSession() {
    lock();
    g_srv.dropSession();
    unlock();
}

}  // namespace robot
