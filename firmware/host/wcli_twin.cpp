// The Wi-Fi CLI's robot half on a host UDP socket, for the Python client's
// end-to-end tests (tests/test_wcli.py) -- and for trying tools/wcli.py and
// tools/wcli_bridge.py with no robot at all.
//
//   build/wcli_twin --key-hex <64 hex> [--port N] [--loss P] [--seed S]
//                   [--armed]
//
// The session logic is the REAL components/wcli (Server, classify), on the
// same two-thread shape as main/wifi_cli.cpp: a network thread (receive ->
// Server::onDatagram, then Server::pump) and a "housekeeping" thread that
// executes queued lines through the allowlist and a FAKE CLI, with the same
// bounded-wait / lagging output sink. --loss drops that fraction of the
// datagrams in BOTH directions (deterministic for a given --seed), which is
// how the tests prove at-most-once execution and output recovery.
//
// The fake CLI (the words are allowlisted ones, so classify() passes them):
//   ping <tag>      "pong <tag> #<n>" -- n counts executions of that tag
//   trace <n>       n numbered lines (a long reply)
//   scan <ms>       sleeps, then "0 servo(s)" (a slow command)
//   pose ...        "pose: streaming", then ~300 ms later the ASYNC line
//                   "pose: done" (what poseTick prints on the robot)
//   help            a few lines
//   anything else   "? (try `help`)", like the CLI
// Prints "listening <port>" on stdout once bound.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "wcli/wcli.h"

namespace {

std::mutex g_mu;                 // around every Server call, like the firmware
wcli::Server g_srv;
std::atomic<bool> g_armed{false};
int g_sock = -1;
double g_loss = 0.0;
uint64_t g_rng = 1;

struct Line {
    uint32_t seq;
    std::string text;
};
std::mutex g_qmu;
std::deque<Line> g_queue;        // the CLI queue (the firmware's is 4 deep)
constexpr size_t kQueueLen = 4;

uint32_t nowMs() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

uint64_t rnd() {   // xorshift64*: deterministic per --seed
    g_rng ^= g_rng >> 12;
    g_rng ^= g_rng << 25;
    g_rng ^= g_rng >> 27;
    return g_rng * 0x2545F4914F6CDD1Dull;
}
bool drop() {
    return g_loss > 0.0 &&
           static_cast<double>(rnd() >> 11) / 9007199254740992.0 < g_loss;
}

struct NetTx : wcli::Tx {
    void send(const wcli::Peer& to, const uint8_t* buf, size_t n) override {
        if (drop()) return;
        sockaddr_in a = {};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(to.ip);
        a.sin_port = htons(to.port);
        sendto(g_sock, buf, n, 0, reinterpret_cast<sockaddr*>(&a), sizeof a);
    }
    bool submit(uint32_t seq, const char* line) override {
        std::lock_guard<std::mutex> l(g_qmu);
        if (g_queue.size() >= kQueueLen) return false;
        g_queue.push_back(Line{seq, line});
        return true;
    }
    virtual ~NetTx() = default;
};
NetTx g_tx;

// The output sink: main/wifi_cli.cpp's wifiSay, on std::mutex.
void say(const char* s) {
    size_t n = strlen(s);
    uint32_t stalled_since = 0;
    while (n > 0) {
        size_t k;
        {
            std::lock_guard<std::mutex> l(g_mu);
            const uint32_t now = nowMs();
            k = g_srv.output(s, n, now);
            if (k == 0 && stalled_since != 0 &&
                g_srv.shouldStopWaiting(now - stalled_since, now)) {
                g_srv.setLagging();
            }
        }
        if (k > 0) {
            s += k;
            n -= k;
            stalled_since = 0;
            continue;
        }
        if (stalled_since == 0) stalled_since = nowMs();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// Async output: main/wifi_cli.cpp's wcliAsyncOutput -- never waits.
void sayAsync(const char* s) {
    std::lock_guard<std::mutex> l(g_mu);
    const uint32_t now = nowMs();
    if (!g_srv.listening(now)) return;
    const size_t n = strlen(s);
    const size_t k = g_srv.output(s, n, now);
    if (k < n) {
        g_srv.setLagging();
        g_srv.output(s + k, n - k, now);
    }
}

void retry(bool (*fn)(uint32_t), uint32_t arg) {
    const uint32_t t0 = nowMs();
    for (;;) {
        {
            std::lock_guard<std::mutex> l(g_mu);
            if (fn(arg)) return;
            const uint32_t now = nowMs();
            if (g_srv.shouldStopWaiting(now - t0, now)) g_srv.setLagging();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

std::map<std::string, int> g_pings;
uint32_t g_pose_done_at = 0;

void fakeCli(const std::string& line) {
    char buf[256];
    const std::string w = line.substr(0, line.find(' '));
    const std::string arg = line.find(' ') == std::string::npos
                                ? ""
                                : line.substr(line.find(' ') + 1);
    if (w == "ping") {
        const int n = ++g_pings[arg];
        snprintf(buf, sizeof buf, "pong %s #%d\r\n", arg.c_str(), n);
        say(buf);
    } else if (w == "trace") {
        const int n = atoi(arg.c_str());
        for (int i = 0; i < n; ++i) {
            snprintf(buf, sizeof buf, "trace %05d pos 2048 spd 0 load 0\r\n", i);
            say(buf);
        }
    } else if (w == "scan") {
        std::this_thread::sleep_for(std::chrono::milliseconds(atoi(arg.c_str())));
        say("scanning IDs 0-253 ...\r\n");
        say("0 servo(s)\r\n");
    } else if (w == "pose") {
        say("pose: streaming\r\n");
        g_pose_done_at = nowMs() + 300;
    } else if (w == "help" || w == "?") {
        say("\r\nbimo firmware v1 -- bench mode (nothing moves until `run`)\r\n");
        say("  ping [id]            no id = every bus servo against the fitted set\r\n");
    } else if (!w.empty()) {
        say("? (try `help`)\r\n");
    }
}

bool endOk(uint32_t seq) { return g_srv.endCommand(seq, 0); }
bool endRefused(uint32_t seq) { return g_srv.endCommand(seq, wcli::kOutRefused); }
bool flushOk(uint32_t) {
    if (!g_srv.flush()) {   // async: never waits (the robot's runs in poseTick)
        g_srv.setLagging();
        g_srv.flush();
    }
    return true;
}

// The housekeeping task's half (link_task.cpp houseTask + wifi_cli.cpp
// wcliExecute): one queued line at a time, then the async work, then flush.
void house() {
    for (;;) {
        Line ln{0, ""};
        bool have = false;
        {
            std::lock_guard<std::mutex> l(g_qmu);
            if (!g_queue.empty()) {
                ln = g_queue.front();
                g_queue.pop_front();
                have = true;
            }
        }
        if (have) {
            bool go;
            {
                std::lock_guard<std::mutex> l(g_mu);
                go = g_srv.beginCommand(ln.seq);
            }
            if (go) {
                const wcli::Decision d = wcli::classify(ln.text.c_str(), g_armed.load());
                fprintf(stderr, "twin: exec %u '%s' -> %d\n", ln.seq, ln.text.c_str(),
                        static_cast<int>(d.verdict));
                if (d.verdict == wcli::Verdict::kAllow) {
                    fakeCli(ln.text);
                    retry(endOk, ln.seq);
                } else {
                    char msg[200];
                    wcli::formatRefusal(d, ln.text.c_str(), msg, sizeof msg);
                    say(msg);
                    retry(endRefused, ln.seq);
                }
            }
        }
        if (g_pose_done_at != 0 && nowMs() >= g_pose_done_at) {
            g_pose_done_at = 0;
            sayAsync("pose: done\r\n");
        }
        retry(flushOk, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool parseHex(const char* h, uint8_t* out, size_t n) {
    if (strlen(h) != 2 * n) return false;
    for (size_t i = 0; i < n; ++i) {
        unsigned v = 0;
        if (sscanf(h + 2 * i, "%2x", &v) != 1) return false;
        out[i] = static_cast<uint8_t>(v);
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    uint8_t key[wcli::kKeyLen] = {};
    bool have_key = false;
    int port = 0;
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const char* v = i + 1 < argc ? argv[i + 1] : "";
        if (!strcmp(a, "--key-hex")) { have_key = parseHex(v, key, sizeof key); ++i; }
        else if (!strcmp(a, "--port")) { port = atoi(v); ++i; }
        else if (!strcmp(a, "--loss")) { g_loss = atof(v); ++i; }
        else if (!strcmp(a, "--seed")) { g_rng = strtoull(v, nullptr, 0) | 1u; ++i; }
        else if (!strcmp(a, "--armed")) { g_armed = true; }
        else { fprintf(stderr, "unknown argument %s\n", a); return 2; }
    }
    if (!have_key) {
        fprintf(stderr, "usage: wcli_twin --key-hex <64 hex> [--port N] [--loss P] "
                        "[--seed S] [--armed]\n");
        return 2;
    }
    g_srv.setKey(key);
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(g_sock, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0) {
        perror("bind");
        return 1;
    }
    socklen_t al = sizeof a;
    getsockname(g_sock, reinterpret_cast<sockaddr*>(&a), &al);
    timeval tv = {0, 10 * 1000};
    setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    printf("listening %d\n", ntohs(a.sin_port));
    fflush(stdout);

    std::thread(house).detach();
    uint64_t nonce_ctr = 0x7717000000000000ull ^ static_cast<uint64_t>(time(nullptr));
    for (;;) {
        uint8_t buf[2048];
        sockaddr_in from = {};
        socklen_t fl = sizeof from;
        const ssize_t n = recvfrom(g_sock, buf, sizeof buf, 0,
                                   reinterpret_cast<sockaddr*>(&from), &fl);
        std::lock_guard<std::mutex> l(g_mu);
        if (n > 0 && !drop()) {
            const wcli::Peer p{ntohl(from.sin_addr.s_addr), ntohs(from.sin_port)};
            // The robot uses esp_random(); a counter mixed through rnd() is
            // plenty for a test double (and it never repeats).
            g_srv.onDatagram(buf, static_cast<size_t>(n), p, nowMs(),
                             ++nonce_ctr * 0x9E3779B97F4A7C15ull, g_tx);
        }
        g_srv.pump(g_tx, 8);
    }
}
