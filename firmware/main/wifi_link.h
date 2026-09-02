// WiFi station + UDP command link -- the untethered half of the v2 seam.
//
// docs/control-channel.md specifies identical framing over UDP and UART0, so
// this is exactly what link_task.cpp's header promised: a SECOND producer into
// g_cmd_mailbox, not a second protocol. Commands arrive as one 14-byte
// datagram per frame on port 4210; telemetry goes back to the last commander
// on port 4211 at 10 Hz.
//
// Credentials live in NVS (never in git, never compiled in) and are
// provisioned once over the tether: `wifi <ssid> <psk>`. A board with no
// stored credentials boots with the radio idle and says so.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace robot {

struct WifiStatus {
    bool driver_up;        // esp_wifi initialised (radio may still be idle)
    bool have_creds;
    bool connected;        // associated AND holding a DHCP lease
    char ssid[33];
    uint32_t ip;           // host order; 0 until GOT_IP
    int rssi;              // dBm, only meaningful while connected
    uint32_t peer_ip;      // last commander's address; 0 = none yet
    uint32_t rx_frames;    // valid command frames handed to the mailbox
    uint32_t rx_bad;       // wrong length, bad magic/version/CRC
    uint32_t tx_tlm;       // telemetry datagrams sent (both lengths)
    uint32_t tx_tlm_ext;   // ... of which kTlmLenExt (pose) beacons
    bool pose_wanted;      // last valid command frame carried kFlagPose
    uint32_t disconnects;  // STA_DISCONNECTED events since boot
};

// NVS-stored station credentials (namespace "bimo"). Load returns false when
// nothing is stored -- the normal first-boot path, not an error.
bool wifiCredsSave(const char* ssid, const char* psk);
bool wifiCredsLoad(char* ssid, size_t ssid_cap, char* psk, size_t psk_cap);
bool wifiCredsErase();

// Bring up netif + the WiFi driver and start the UDP link task. Joins the
// stored network when credentials exist; otherwise the radio stays idle until
// `wifi <ssid> <psk>` calls wifiApply(). Called once from app_main, AFTER
// startLinkTask() (the mailbox must exist). All driver allocation happens
// here, at init, per firmware-design section 2.
void startWifiLink();

// Re-point the running driver at new credentials without a reboot (the CLI
// calls this right after wifiCredsSave). Returns false if the driver is not
// up.
bool wifiApply(const char* ssid, const char* psk);

void wifiGetStatus(WifiStatus& out);

}  // namespace robot
