// WiFi station + UDP command link. See wifi_link.h for the contract.
//
// Ownership rules (shared.h): this task is the second producer into
// g_cmd_mailbox -- explicitly blessed by the v2 seam note in link_task.cpp --
// and a second READER of g_telemetry. The peer address never leaves this
// task, so there is no cross-task state beyond the status atomics, each of
// which has this file as its only writer.
#include "wifi_link.h"

#include <string.h>

#include <atomic>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "linkproto/protocol.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "shared.h"

namespace robot {
namespace {

// Same namespace as the calibration blobs; string keys, because an SSID and a
// PSK are strings and a versioned blob buys nothing here.
constexpr const char* kNs = "bimo";
constexpr const char* kKeySsid = "wifi_ssid";
constexpr const char* kKeyPsk = "wifi_psk";

constexpr int kWifiStackWords = 4096;
StackType_t g_wifi_stack[kWifiStackWords];
StaticTask_t g_wifi_tcb;

// Status shared with the CLI. Single writer each (this file); the CLI read is
// a human-facing report, so torn reads follow the TelemetrySnapshot rule.
std::atomic<bool> g_driver_up{false};
std::atomic<bool> g_connected{false};
std::atomic<uint32_t> g_ip{0};
std::atomic<uint32_t> g_peer_ip{0};
std::atomic<uint32_t> g_rx_frames{0};
std::atomic<uint32_t> g_rx_bad{0};
std::atomic<uint32_t> g_tx_tlm{0};
std::atomic<uint32_t> g_disconnects{0};
char g_ssid[33] = {0};   // written by startWifiLink/wifiApply (CLI task) only

void onWifiEvent(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_connected.store(false);
        g_ip.store(0);
        g_disconnects.fetch_add(1);
        // Retry forever, immediately: the radio's own scan/assoc timing paces
        // this, and a robot mid-run must fight for its link, not back off.
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t* e = static_cast<ip_event_got_ip_t*>(data);
        g_ip.store(lwip_ntohl(e->ip_info.ip.addr));
        g_connected.store(true);
    }
}

// The UDP loop: receive command frames, feed the mailbox, beacon telemetry.
// Runs on core 0 at the same priority as the UART link task -- WiFi/LwIP's
// own tasks live on core 0 too (ctrl_task.cpp pins control to core 1 for
// exactly that reason).
void wifiLinkTask(void*) {
    int sock = -1;
    sockaddr_in peer = {};          // last commander; task-local on purpose
    bool have_peer = false;
    int64_t next_tlm_us = 0;

    for (;;) {
        if (!g_connected.load()) {
            // Not associated. A socket bound to INADDR_ANY survives the
            // outage, so just idle until the event handler reconnects us.
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (sock < 0) {
            sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
            timeval tv = {0, 50 * 1000};   // 50 ms: telemetry cadence tick
            lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
            sockaddr_in addr = {};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = lwip_htonl(INADDR_ANY);
            addr.sin_port = lwip_htons(linkproto::kCmdPort);
            if (lwip_bind(sock, reinterpret_cast<sockaddr*>(&addr),
                          sizeof addr) < 0) {
                lwip_close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        uint8_t rx[64];
        sockaddr_in from = {};
        socklen_t from_len = sizeof from;
        const int n = lwip_recvfrom(sock, rx, sizeof rx, 0,
                                    reinterpret_cast<sockaddr*>(&from),
                                    &from_len);
        if (n > 0) {
            // One datagram = one frame; UDP keeps the boundary, so no Demux.
            linkproto::Command pkt{};
            if (n == static_cast<int>(linkproto::kCmdLen) &&
                linkproto::decodeCommand(rx, linkproto::kCmdLen, pkt) ==
                    linkproto::Err::kOk) {
                CommandMsg msg;
                msg.cmd = pkt;
                msg.rx_ms = static_cast<float>(esp_timer_get_time()) / 1000.0f;
                if (g_cmd_mailbox) xQueueOverwrite(g_cmd_mailbox, &msg);
                g_rx_frames.fetch_add(1);
                // Latest commander wins, same rule as latest command.
                peer = from;
                have_peer = true;
                g_peer_ip.store(lwip_ntohl(from.sin_addr.s_addr));
            } else {
                // Stray traffic on an open port -- exactly what the CRC is
                // for (control-channel.md). Count it, drop it.
                g_rx_bad.fetch_add(1);
            }
        }

        // 10 Hz telemetry to whoever commanded last. Unlike the UART path
        // this is NOT gated on kRun: UDP has no CLI text to interleave with,
        // and a beacon you can hear while benched is what makes the link
        // verifiable without arming the control loop.
        const int64_t now_us = esp_timer_get_time();
        if (have_peer && now_us >= next_tlm_us) {
            // Drift-free schedule: the task wakes every <=50 ms (the recv
            // timeout), so `next += period` holds a true 10 Hz average, where
            // `next = now + period` would add the wakeup slack to every
            // beacon (~8 Hz, measured). Resync after an outage rather than
            // bursting to catch up.
            if (now_us - next_tlm_us > 500 * 1000) next_tlm_us = now_us;
            next_tlm_us += 100 * 1000;
            linkproto::Telemetry t{};
            t.seq_echo = g_telemetry.seq_echo.load();
            t.state = static_cast<linkproto::LinkState>(g_telemetry.state.load());
            // While benched, ctrl neither runs the watchdog nor publishes, so
            // the snapshot is stale (boot default reads as LIVE). The CLI owns
            // the bus and the link commands nothing: say BENCH, so the client
            // knows an ARM edge (not a walk command) is what it needs to send.
            if (g_mode_request.load() == Mode::kBench) {
                t.state = linkproto::LinkState::kBench;
                // ... and say WHY it is still benched. seq_echo is "the last
                // command seq the loop applied"; benched it applied none, so
                // this is the one state where the field has nothing to lose,
                // which is exactly why the diagnostic rides there (protocol.h
                // kDiag*, docs/control-channel.md). A commander that predates
                // this still decodes the frame: no version, no length change.
                t.seq_echo = linkproto::packDiag(
                    false, g_cal_from_nvs.load(),
                    static_cast<linkproto::ArmResult>(g_arm_result.load()));
            }
            t.vbat_v = g_telemetry.vbat_mv.load() / 1000.0f;
            t.up_z = g_telemetry.up_z.load();
            t.vx_est = g_telemetry.vx_est.load();
            t.wz_est = g_telemetry.wz_est.load();
            // 10 joints folded into the 8-bit wire field, same as link_task.
            const uint16_t f = g_telemetry.servo_err.load();
            t.servo_err = static_cast<uint8_t>(f & 0xFF) |
                          static_cast<uint8_t>((f >> 8) & 0x03);
            t.loop_late_pct = g_telemetry.loop_late_pct.load();
            uint8_t wire[linkproto::kTlmLen];
            linkproto::encodeTelemetry(wire, t);
            sockaddr_in to = peer;
            to.sin_port = lwip_htons(linkproto::kTlmPort);
            if (lwip_sendto(sock, wire, linkproto::kTlmLen, 0,
                            reinterpret_cast<sockaddr*>(&to), sizeof to) ==
                static_cast<int>(linkproto::kTlmLen)) {
                g_tx_tlm.fetch_add(1);
            }
        }
    }
}

bool setStationConfig(const char* ssid, const char* psk) {
    wifi_config_t cfg = {};
    strncpy(reinterpret_cast<char*>(cfg.sta.ssid), ssid,
            sizeof cfg.sta.ssid - 1);
    strncpy(reinterpret_cast<char*>(cfg.sta.password), psk,
            sizeof cfg.sta.password - 1);
    // WPA2 minimum: an open or WEP "match" for our SSID is an impostor.
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    return esp_wifi_set_config(WIFI_IF_STA, &cfg) == ESP_OK;
}

}  // namespace

bool wifiCredsSave(const char* ssid, const char* psk) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_set_str(h, kKeySsid, ssid);
    if (err == ESP_OK) err = nvs_set_str(h, kKeyPsk, psk);
    const esp_err_t cerr = (err == ESP_OK) ? nvs_commit(h) : err;
    nvs_close(h);
    return cerr == ESP_OK;
}

bool wifiCredsLoad(char* ssid, size_t ssid_cap, char* psk, size_t psk_cap) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return false;
    size_t sl = ssid_cap, pl = psk_cap;
    const bool ok = nvs_get_str(h, kKeySsid, ssid, &sl) == ESP_OK &&
                    nvs_get_str(h, kKeyPsk, psk, &pl) == ESP_OK;
    nvs_close(h);
    return ok;
}

bool wifiCredsErase() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    nvs_erase_key(h, kKeySsid);   // absent key is fine; erasing is idempotent
    nvs_erase_key(h, kKeyPsk);
    const esp_err_t cerr = nvs_commit(h);
    nvs_close(h);
    return cerr == ESP_OK;
}

void startWifiLink() {
    if (esp_netif_init() != ESP_OK) return;
    if (esp_event_loop_create_default() != ESP_OK) return;
    esp_netif_t* netif = esp_netif_create_default_wifi_sta();
    if (!netif) return;
    esp_netif_set_hostname(netif, "bimo");

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&init) != ESP_OK) return;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &onWifiEvent, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &onWifiEvent, nullptr, nullptr);
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) return;
    g_driver_up.store(true);

    char ssid[33], psk[65];
    if (wifiCredsLoad(ssid, sizeof ssid, psk, sizeof psk)) {
        strncpy(g_ssid, ssid, sizeof g_ssid - 1);
        if (setStationConfig(ssid, psk)) {
            esp_wifi_start();   // STA_START -> connect, via the handler
            // Modem power save trades ~100 ms latency spikes for milliwatts.
            // On a 20 Hz link with a 250 ms stale window that trade is wrong,
            // and next to ten servos the milliwatts are noise.
            esp_wifi_set_ps(WIFI_PS_NONE);
        }
    }
    // No credentials: the driver idles. app_main logs which case this is.

    xTaskCreateStaticPinnedToCore(wifiLinkTask, "wifi_link", kWifiStackWords,
                                  nullptr, 5, g_wifi_stack, &g_wifi_tcb, 0);
}

bool wifiApply(const char* ssid, const char* psk) {
    if (!g_driver_up.load()) return false;
    strncpy(g_ssid, ssid, sizeof g_ssid - 1);
    g_ssid[sizeof g_ssid - 1] = 0;
    esp_wifi_disconnect();   // fails harmlessly when not yet started
    if (!setStationConfig(ssid, psk)) return false;
    if (esp_wifi_start() != ESP_OK) return false;   // no-op if already started
    esp_wifi_set_ps(WIFI_PS_NONE);
    // If this was the FIRST start, the STA_START event has already kicked off
    // a connect and this one returns ESP_ERR_WIFI_CONN -- not a failure.
    esp_wifi_connect();
    return true;
}

void wifiGetStatus(WifiStatus& out) {
    out.driver_up = g_driver_up.load();
    char ssid[33], psk[65];
    out.have_creds = wifiCredsLoad(ssid, sizeof ssid, psk, sizeof psk);
    out.connected = g_connected.load();
    strncpy(out.ssid, g_ssid, sizeof out.ssid - 1);
    out.ssid[sizeof out.ssid - 1] = 0;
    out.ip = g_ip.load();
    out.rssi = 0;
    if (out.connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) out.rssi = ap.rssi;
    }
    out.peer_ip = g_peer_ip.load();
    out.rx_frames = g_rx_frames.load();
    out.rx_bad = g_rx_bad.load();
    out.tx_tlm = g_tx_tlm.load();
    out.disconnects = g_disconnects.load();
}

}  // namespace robot
