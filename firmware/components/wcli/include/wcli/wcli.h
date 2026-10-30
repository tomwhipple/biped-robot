// The authenticated Wi-Fi CLI -- wire format, robot-side session logic and
// the Wi-Fi allowlist. docs/wifi-cli.md is the manual and the threat model.
//
// link/wcliproto.py is the REFERENCE for the wire format; this file conforms
// to it byte for byte, and tools/gen_wcli_vectors.py freezes the Python
// side's output into host/vectors/wcli_vectors.h so a divergence is a failing
// host test, not a mystery on the bench (the linkproto convention).
//
// Everything here is pure -- no sockets, no clock, no RNG, no FreeRTOS: the
// caller passes now_ms and a fresh nonce, and receives datagrams to send and
// lines to execute through the Tx interface. That is what lets the host
// tests drive the exact session logic the robot runs, under ASan/UBSan, and
// what lets firmware/host/wcli_twin.cpp put it on a real UDP socket for the
// Python client's end-to-end tests.
//
// THE PROTOCOL IN ONE PARAGRAPH. Every datagram is
//   'B' 'W' ver type | sid u64 | seq u32 | aux u32 | flags u16 | len u16 |
//   payload[len] | tag[16]
// little-endian, tag = HMAC-SHA256(direction key, everything before it)
// truncated to 16 bytes. The direction keys are derived from the 32-byte
// shared key (provisioned over the tether, never sent over the air), so a
// robot->client message can never be reflected back as a command. The
// client says HELLO (sid = its own nonce); the robot answers CHALLENGE with a
// fresh random session id; the first authenticated message carrying that
// session id makes it THE session (one paired client; the latest wins), so
// every command of an earlier session -- or of an earlier boot -- is dead on
// arrival. Within a session, commands carry a strictly increasing seq: a seq
// already accepted is answered, never executed again (retransmits are safe),
// an older one is ignored. Output comes back as sequenced chunks the client
// acknowledges and can ask to have re-sent.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "wcli/sha256.h"

namespace wcli {

// -- wire ------------------------------------------------------------------
constexpr uint8_t kMagic0 = 'B';
constexpr uint8_t kMagic1 = 'W';
constexpr uint8_t kVersion = 1;
// The robot listens here; it answers to the request's source address/port,
// so the client may bind any port. 4210 commands, 4211 telemetry, 4212 sim
// pose (commander.py --pose-port), 4220 the flight collector.
constexpr uint16_t kPort = 4213;

constexpr size_t kKeyLen = 32;
constexpr size_t kTagLen = 16;
constexpr size_t kHeaderLen = 24;
constexpr size_t kOverhead = kHeaderLen + kTagLen;
// The longest CLI line the tether accepts (linkproto::kMaxCliLine; a
// static_assert in the firmware keeps the two equal).
constexpr size_t kMaxLine = 192;
// Output text per chunk. 440 B datagrams: well under any MTU, no IP
// fragmentation on the LAN.
constexpr size_t kMaxChunk = 400;
constexpr size_t kMaxDatagram = kOverhead + kMaxChunk;

enum class Type : uint8_t {
    // client -> robot
    kHello = 0x01,      // sid = client nonce. Answered with kChallenge.
    kCmd = 0x02,        // seq = command seq, payload = the CLI line
    kCancel = 0x03,     // seq = a command seq the client gave up on
    kPoll = 0x04,       // aux = next output seq wanted (ACKs all below);
                        // flags kPollResend = re-send from aux. Keepalive.
    // robot -> client
    kChallenge = 0x81,  // sid = the NEW session id, payload = client nonce (8)
    kOut = 0x82,        // seq = output seq, aux = command seq (0 = async),
                        // flags kOut*, payload = text, exactly as the tether
    kStatus = 0x83,     // seq = next output seq, aux = highest command seq consumed,
                        // flags = CmdState | kSt*, payload = u32 oldest
                        // output seq still held (anything older is gone)
    kStale = 0x84,      // sid = the session id the client used: not current
                        // (robot rebooted, or another client paired) -- HELLO
};

constexpr uint16_t kOutDone = 1u << 0;       // last chunk of command `aux`
constexpr uint16_t kOutAsync = 1u << 1;      // printed outside any Wi-Fi command
constexpr uint16_t kOutRefused = 1u << 2;    // the command was refused, not run
constexpr uint16_t kPollResend = 1u << 0;

enum class CmdState : uint8_t {
    kNone = 0,       // no command yet this session
    kQueued = 1,     // accepted, waiting for the housekeeping task
    kRunning = 2,
    kDone = 3,       // executed (or refused) -- its output is in the stream
    kCancelled = 4,  // the client cancelled it before it ran: never executed
};
constexpr uint16_t kStStateMask = 0x00FF;
constexpr uint16_t kStBusy = 1u << 8;     // the command/session was NOT accepted:
                                          // one is in flight, or the queue is full
constexpr uint16_t kStLagging = 1u << 9;  // output was overwritten unacknowledged

struct Msg {
    Type type = Type::kHello;
    uint64_t sid = 0;
    uint32_t seq = 0;
    uint32_t aux = 0;
    uint16_t flags = 0;
    uint16_t len = 0;
    const uint8_t* payload = nullptr;   // decode: points INTO the datagram
};

struct Keys {
    uint8_t c2r[kShaLen];   // client -> robot MAC key
    uint8_t r2c[kShaLen];   // robot -> client MAC key
};
// K_c2r = HMAC(K, "bimo-wcli c2r v1"), K_r2c = HMAC(K, "bimo-wcli r2c v1").
void deriveKeys(const uint8_t key[kKeyLen], Keys& out);

// Encode into out (cap bytes). Returns the datagram length, or 0 when it
// does not fit or len exceeds kMaxChunk.
size_t encode(uint8_t* out, size_t cap, const Msg& m, const uint8_t mac_key[kShaLen]);

enum class Err : uint8_t { kOk, kShort, kBadMagic, kBadVersion, kBadLength, kBadTag };
// Verify and parse. The tag is checked (constant time) BEFORE any field is
// believed; on kOk, out.payload points into buf.
Err decode(const uint8_t* buf, size_t n, const uint8_t mac_key[kShaLen], Msg& out);

// -- the Wi-Fi allowlist -----------------------------------------------------
//
// Default DENY: a command word not named here is refused over Wi-Fi, so a
// CLI command added later is tether-only until someone decides otherwise.
// The persistent writers are refused by name with the reason; the CLI also
// refuses them itself when the line came over Wi-Fi (cli.cpp tetherOnly()),
// so a slip in this table is not enough on its own.
enum class Verdict : uint8_t {
    kAllow,
    kRefuseArmed,     // the loop is armed: the Wi-Fi CLI is bench-only
    kRefuseTether,    // persistent state / safety latch / unsafe tool
    kRefuseUnknown,   // not on the allowlist
    kRefuseBadLine,   // control or non-ASCII bytes
};
struct Decision {
    Verdict verdict = Verdict::kAllow;
    const char* why = "";   // static text, for the refusal line
};
Decision classify(const char* line, bool armed);
// The refusal line the client sees (CRLF-terminated, like every CLI line).
// Returns its length; cap >= 160 always fits.
size_t formatRefusal(const Decision& d, const char* line, char* out, size_t cap);

// A CLI line as it may be LOGGED -- the flight recorder ships every line
// over UDP in clear: `wcli key <hex>` keeps only "wcli key <redacted>" and
// `wifi <ssid> <psk>` drops the PSK. Everything else is copied as is.
// Always NUL-terminates (cap > 0).
void redactForLog(const char* line, char* out, size_t cap);

// -- the robot side ----------------------------------------------------------
struct Peer {
    uint32_t ip = 0;     // host order
    uint16_t port = 0;
};
inline bool operator==(const Peer& a, const Peer& b) {
    return a.ip == b.ip && a.port == b.port;
}

// What the Server does to the world.
class Tx {
  public:
    virtual void send(const Peer& to, const uint8_t* buf, size_t n) = 0;
    // Queue a CLI line for the executor (the housekeeping task). false =
    // the queue is full; the seq is then NOT consumed and the client's
    // retransmit tries again.
    virtual bool submit(uint32_t seq, const char* line) = 0;

  protected:
    ~Tx() = default;
};

struct Counters {
    uint32_t rx_ok = 0;          // authenticated datagrams
    uint32_t rx_bad = 0;         // malformed, wrong tag, wrong direction: dropped, unanswered
    uint32_t rx_nokey = 0;       // anything at all while no key is provisioned
    uint32_t rx_stale = 0;       // authentic, but for a session that is not current
    uint32_t rx_wrong_peer = 0;  // authentic, current session, other address: dropped
    uint32_t replays = 0;        // command seq already accepted / older: not executed
    uint32_t sessions = 0;       // sessions established
    uint32_t cmds = 0;           // commands accepted for execution
    uint32_t busy = 0;           // commands/sessions refused as busy
    uint32_t lost_chunks = 0;    // output chunks overwritten unacknowledged
};

class Server {
  public:
    static constexpr uint32_t kRing = 16;               // output chunks held (6.4 KB)
    static constexpr uint32_t kChallengeMs = 10000;     // a CHALLENGE expires
    static constexpr uint32_t kIdleMs = 30000;          // no client traffic: stop
                                                        // buffering async output
    // The output sink's wait policy (shouldStopWaiting).
    static constexpr uint32_t kAckWaitMs = 1000;        // a client this silent is gone
    static constexpr uint32_t kMinWaitMs = 20;          // ...judged after this much wait
    static constexpr uint32_t kStallMs = 2000;          // never wait longer than this
    static constexpr int kPending = 2;                  // outstanding challenges
    static constexpr uint32_t kChunkMs = 100;           // a command's partial
                                                        // chunk older than this
                                                        // is closed on the next
                                                        // print (long commands
                                                        // stream, not burst)

    void setKey(const uint8_t key[kKeyLen]);   // drops any session
    void clearKey();
    bool hasKey() const { return have_key_; }
    void dropSession();

    // One received datagram. `nonce` must be fresh random bits on every call
    // (only a HELLO consumes it; 0 is replaced by 1).
    void onDatagram(const uint8_t* buf, size_t n, const Peer& from,
                    uint32_t now_ms, uint64_t nonce, Tx& tx);

    // -- executor side (the housekeeping task, under the caller's lock) ----
    // true: run command `seq` now (state -> RUNNING). false: it was
    // cancelled, or its session is gone -- do NOT execute it.
    bool beginCommand(uint32_t seq);
    // Append printed text: a command's output while one is RUNNING, else
    // async output (kept only while a client is listening). Returns how many
    // bytes were taken; fewer than n means the window is full of chunks the
    // client has not acknowledged yet. The caller then waits a little (the
    // client ACKs every ~100 ms) and calls again -- unless
    // shouldStopWaiting(), when it calls setLagging() and goes on.
    size_t output(const char* text, size_t n, uint32_t now_ms);
    // Close command `seq` with kOutDone (| extra flags). false = no room yet.
    bool endCommand(uint32_t seq, uint16_t extra_flags);
    // Close the open async chunk so it can be sent. false = no room yet.
    bool flush();
    // The wait policy, given how long this print has been stalled: stop when
    // the client has gone silent (kAckWaitMs, judged after kMinWaitMs), or
    // after kStallMs regardless -- a vanished laptop must not stall the
    // housekeeping task, and neither may a live one that never ACKs.
    bool shouldStopWaiting(uint32_t stalled_ms, uint32_t now_ms) const;
    // Stop waiting: from now on the oldest unacknowledged chunk is
    // overwritten (and counted lost). Cleared by the client's next POLL.
    void setLagging();
    bool lagging() const { return lagging_; }

    // -- sender side ---------------------------------------------------------
    // Send up to max_chunks closed, unsent chunks to the session's client.
    void pump(Tx& tx, int max_chunks);

    // -- status ---------------------------------------------------------------
    bool established() const { return established_; }
    Peer peer() const { return peer_; }
    uint32_t lastSeq() const { return last_seq_; }
    CmdState state() const { return state_; }
    uint32_t outNext() const { return next_seq_; }
    uint32_t oldestHeld() const;
    const Counters& counters() const { return ctr_; }
    // A client has been heard from within kIdleMs.
    bool listening(uint32_t now_ms) const;
    uint32_t heardMsAgo(uint32_t now_ms) const { return now_ms - last_rx_ms_; }

  private:
    struct Chunk {
        uint32_t seq = 0;
        uint32_t cmd = 0;
        uint16_t flags = 0;
        uint16_t len = 0;
        char text[kMaxChunk] = {};
    };
    struct Pending {
        bool used = false;
        uint64_t sid = 0;
        Peer peer;
        uint32_t t_ms = 0;
    };

    void reply(Tx& tx, const Peer& to, Type t, uint64_t sid, uint32_t seq,
               uint32_t aux, uint16_t flags, const uint8_t* payload,
               uint16_t len);
    void sendStatus(Tx& tx, uint16_t extra);
    void establish(uint64_t sid, const Peer& p);
    bool canClose() const;
    bool close(uint16_t extra_flags);
    bool inFlight() const {
        return state_ == CmdState::kQueued || state_ == CmdState::kRunning;
    }

    bool have_key_ = false;
    Keys keys_ = {};
    Pending pending_[kPending];
    int pending_next_ = 0;

    bool established_ = false;
    uint64_t sid_ = 0;
    Peer peer_;
    uint32_t last_rx_ms_ = 0;
    uint32_t last_seq_ = 0;   // the last command accepted for execution
    uint32_t burned_ = 0;     // the highest seq CANCELled before it was seen
    uint32_t consumed() const { return last_seq_ > burned_ ? last_seq_ : burned_; }
    CmdState state_ = CmdState::kNone;

    // Output. Chunks get their seq when CLOSED; seq s lives in ring_[s % kRing].
    Chunk ring_[kRing];
    Chunk open_;              // being filled; .cmd/.flags say whose
    bool open_used_ = false;
    uint32_t open_t_ms_ = 0;  // when open_ took its first byte
    uint32_t next_seq_ = 1;   // the seq the next closed chunk gets
    uint32_t acked_ = 1;      // the client has everything below this
    uint32_t next_send_ = 1;  // the first chunk not yet (re)sent
    bool lagging_ = false;

    Counters ctr_;
};

}  // namespace wcli
