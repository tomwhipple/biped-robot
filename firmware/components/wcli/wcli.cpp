// The Wi-Fi CLI's wire codec and robot-side session logic. See wcli/wcli.h
// for the contract and docs/wifi-cli.md for the threat model.
#include "wcli/wcli.h"

#include <string.h>

namespace wcli {
namespace {

void put16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}
void put32(uint8_t* p, uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
void put64(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
uint16_t get16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
uint32_t get32(const uint8_t* p) {
    uint32_t v = 0;
    for (int i = 3; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}
uint64_t get64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
    return v;
}

constexpr char kLabelC2R[] = "bimo-wcli c2r v1";
constexpr char kLabelR2C[] = "bimo-wcli r2c v1";

bool isClientType(uint8_t t) {
    return t == static_cast<uint8_t>(Type::kHello) ||
           t == static_cast<uint8_t>(Type::kCmd) ||
           t == static_cast<uint8_t>(Type::kCancel) ||
           t == static_cast<uint8_t>(Type::kPoll);
}

}  // namespace

void deriveKeys(const uint8_t key[kKeyLen], Keys& out) {
    hmacSha256(key, kKeyLen, reinterpret_cast<const uint8_t*>(kLabelC2R),
               sizeof kLabelC2R - 1, nullptr, 0, out.c2r);
    hmacSha256(key, kKeyLen, reinterpret_cast<const uint8_t*>(kLabelR2C),
               sizeof kLabelR2C - 1, nullptr, 0, out.r2c);
}

size_t encode(uint8_t* out, size_t cap, const Msg& m,
              const uint8_t mac_key[kShaLen]) {
    if (m.len > kMaxChunk) return 0;
    const size_t n = kOverhead + m.len;
    if (cap < n || (m.len > 0 && m.payload == nullptr)) return 0;
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = static_cast<uint8_t>(m.type);
    put64(out + 4, m.sid);
    put32(out + 12, m.seq);
    put32(out + 16, m.aux);
    put16(out + 20, m.flags);
    put16(out + 22, m.len);
    if (m.len > 0) memmove(out + kHeaderLen, m.payload, m.len);
    uint8_t mac[kShaLen];
    hmacSha256(mac_key, kShaLen, out, kHeaderLen + m.len, nullptr, 0, mac);
    memcpy(out + kHeaderLen + m.len, mac, kTagLen);
    return n;
}

Err decode(const uint8_t* buf, size_t n, const uint8_t mac_key[kShaLen],
           Msg& out) {
    if (n < kOverhead) return Err::kShort;
    if (buf[0] != kMagic0 || buf[1] != kMagic1) return Err::kBadMagic;
    if (buf[2] != kVersion) return Err::kBadVersion;
    const uint16_t len = get16(buf + 22);
    if (len > kMaxChunk || n != kOverhead + len) return Err::kBadLength;
    uint8_t mac[kShaLen];
    hmacSha256(mac_key, kShaLen, buf, kHeaderLen + len, nullptr, 0, mac);
    const bool ok = ctEqual(mac, buf + kHeaderLen + len, kTagLen);
    wipe(mac, sizeof mac);
    if (!ok) return Err::kBadTag;
    out.type = static_cast<Type>(buf[3]);
    out.sid = get64(buf + 4);
    out.seq = get32(buf + 12);
    out.aux = get32(buf + 16);
    out.flags = get16(buf + 20);
    out.len = len;
    out.payload = buf + kHeaderLen;
    return Err::kOk;
}

// -- Server ------------------------------------------------------------------

void Server::setKey(const uint8_t key[kKeyLen]) {
    deriveKeys(key, keys_);
    have_key_ = true;
    for (Pending& p : pending_) p = Pending();
    dropSession();
}

void Server::clearKey() {
    wipe(&keys_, sizeof keys_);
    have_key_ = false;
    for (Pending& p : pending_) p = Pending();
    dropSession();
}

void Server::dropSession() {
    established_ = false;
    sid_ = 0;
    peer_ = Peer();
    last_seq_ = 0;
    burned_ = 0;
    state_ = CmdState::kNone;
    open_used_ = false;
    next_seq_ = acked_ = next_send_ = 1;
    lagging_ = false;
}

void Server::establish(uint64_t sid, const Peer& p) {
    dropSession();
    established_ = true;
    sid_ = sid;
    peer_ = p;
    ++ctr_.sessions;
}

bool Server::listening(uint32_t now_ms) const {
    return established_ && (now_ms - last_rx_ms_) < kIdleMs;
}

uint32_t Server::oldestHeld() const {
    return next_seq_ > kRing ? next_seq_ - kRing : 1;
}

void Server::reply(Tx& tx, const Peer& to, Type t, uint64_t sid, uint32_t seq,
                   uint32_t aux, uint16_t flags, const uint8_t* payload,
                   uint16_t len) {
    Msg m;
    m.type = t;
    m.sid = sid;
    m.seq = seq;
    m.aux = aux;
    m.flags = flags;
    m.len = len;
    m.payload = payload;
    uint8_t buf[kMaxDatagram];
    const size_t n = encode(buf, sizeof buf, m, keys_.r2c);
    if (n > 0) tx.send(to, buf, n);
}

void Server::sendStatus(Tx& tx, uint16_t extra) {
    uint8_t pl[4];
    put32(pl, oldestHeld());
    // aux = the highest seq consumed, and the state is THAT seq's: a burned
    // (cancelled, never seen) seq above the last accepted one reads as
    // cancelled, while the accepted command may still be running.
    const CmdState st = burned_ > last_seq_ ? CmdState::kCancelled : state_;
    const uint16_t flags =
        static_cast<uint16_t>(static_cast<uint16_t>(st) | extra);
    reply(tx, peer_, Type::kStatus, sid_, next_seq_, consumed(), flags, pl,
          sizeof pl);
}

void Server::onDatagram(const uint8_t* buf, size_t n, const Peer& from,
                        uint32_t now_ms, uint64_t nonce, Tx& tx) {
    if (!have_key_) {
        ++ctr_.rx_nokey;
        return;
    }
    Msg m;
    // Nothing unauthenticated is ever answered: no oracle, no amplifier.
    if (decode(buf, n, keys_.c2r, m) != Err::kOk ||
        !isClientType(static_cast<uint8_t>(m.type)) ||
        (m.type == Type::kCmd && m.len > kMaxLine)) {
        ++ctr_.rx_bad;
        return;
    }
    ++ctr_.rx_ok;

    if (m.type == Type::kHello) {
        // Issue a challenge. It changes nothing yet: the current session (if
        // any) stays current until a message proves the challenger holds the
        // key AND saw this fresh id. A replayed HELLO therefore earns the
        // replayer an id it cannot use.
        if (nonce == 0) nonce = 1;
        Pending& p = pending_[pending_next_];
        pending_next_ = (pending_next_ + 1) % kPending;
        p.used = true;
        p.sid = nonce;
        p.peer = from;
        p.t_ms = now_ms;
        uint8_t pl[8];
        put64(pl, m.sid);
        reply(tx, from, Type::kChallenge, nonce, 0, 0, 0, pl, sizeof pl);
        return;
    }

    if (!(established_ && m.sid == sid_)) {
        int hit = -1;
        for (int i = 0; i < kPending; ++i) {
            const Pending& p = pending_[i];
            if (p.used && p.sid == m.sid && p.peer == from &&
                (now_ms - p.t_ms) < kChallengeMs) {
                hit = i;
            }
        }
        if (hit < 0) {
            // An old session, an earlier boot, or another client's: tell the
            // sender (authenticated, so it cannot be forged) to HELLO again.
            ++ctr_.rx_stale;
            reply(tx, from, Type::kStale, m.sid, 0, 0, 0, nullptr, 0);
            return;
        }
        if (inFlight()) {
            // Do not orphan a command that is queued or running: its output
            // would land in the new client's stream. Finish it first.
            ++ctr_.busy;
            reply(tx, from, Type::kStatus, m.sid, 0, 0, kStBusy, nullptr, 0);
            return;
        }
        pending_[hit] = Pending();
        establish(m.sid, from);
    } else if (!(from == peer_)) {
        // One paired client, at the address that proved itself. Anything
        // else with this session id is a copy (replayed or redirected).
        ++ctr_.rx_wrong_peer;
        return;
    }
    last_rx_ms_ = now_ms;

    switch (m.type) {
        case Type::kPoll: {
            if (m.aux > acked_ && m.aux <= next_seq_) acked_ = m.aux;
            const uint16_t lag = lagging_ ? kStLagging : 0;
            lagging_ = false;   // the client is alive: wait for it again
            if ((m.flags & kPollResend) && m.aux < next_send_) {
                const uint32_t lo = oldestHeld();
                next_send_ = m.aux < lo ? lo : m.aux;
            }
            sendStatus(tx, lag);
            return;
        }
        case Type::kCmd: {
            if (m.seq == 0 || m.seq <= consumed()) {
                // Retransmit (== last) or a delayed copy (< last): never
                // executed twice. Say where things stand.
                ++ctr_.replays;
                sendStatus(tx, 0);
                return;
            }
            if (inFlight()) {
                ++ctr_.busy;
                sendStatus(tx, kStBusy);
                return;
            }
            char line[kMaxLine + 1];   // m.len <= kMaxLine (checked above)
            memcpy(line, m.payload, m.len);
            line[m.len] = 0;
            if (!tx.submit(m.seq, line)) {
                ++ctr_.busy;
                sendStatus(tx, kStBusy);
                return;
            }
            last_seq_ = m.seq;
            state_ = CmdState::kQueued;
            ++ctr_.cmds;
            sendStatus(tx, 0);
            return;
        }
        case Type::kCancel: {
            if (m.seq > consumed()) {
                // Burn the seq: a delayed copy of that command can now never
                // run, which is what lets the client give up safely. The
                // command in flight (if any) is not disturbed.
                burned_ = m.seq;
            } else if (m.seq == last_seq_ && state_ == CmdState::kQueued) {
                state_ = CmdState::kCancelled;   // beginCommand() will refuse it
            }
            sendStatus(tx, 0);
            return;
        }
        default:
            return;
    }
}

bool Server::beginCommand(uint32_t seq) {
    if (!established_ || seq != last_seq_ || state_ != CmdState::kQueued) {
        return false;
    }
    state_ = CmdState::kRunning;
    return true;
}

bool Server::canClose() const {
    return (next_seq_ - acked_) < kRing || lagging_;
}

bool Server::shouldStopWaiting(uint32_t stalled_ms, uint32_t now_ms) const {
    if (stalled_ms >= kStallMs) return true;
    // A client that POLLs every 100 ms while a command runs and has not been
    // heard for kAckWaitMs is gone. Only judged after kMinWaitMs of waiting:
    // the network task may simply not have had the CPU to read its POLLs.
    return stalled_ms >= kMinWaitMs && (now_ms - last_rx_ms_) >= kAckWaitMs;
}

bool Server::close(uint16_t extra_flags) {
    if (!canClose()) return false;
    const uint32_t s = next_seq_;
    if (s - acked_ >= kRing) {
        // Lagging: this overwrites the oldest chunk the client never ACKed.
        ++ctr_.lost_chunks;
        acked_ = s - kRing + 1;
    }
    Chunk& c = ring_[s % kRing];
    c = open_;
    c.seq = s;
    c.flags = static_cast<uint16_t>(c.flags | extra_flags);
    ++next_seq_;
    if (next_send_ < oldestHeld()) next_send_ = oldestHeld();
    open_used_ = false;
    open_ = Chunk();
    return true;
}

size_t Server::output(const char* text, size_t n, uint32_t now_ms) {
    const bool cmd = state_ == CmdState::kRunning;
    if (!established_ || (!cmd && !listening(now_ms))) return n;   // nobody to tell
    const uint32_t who = cmd ? last_seq_ : 0;
    const uint16_t kind = cmd ? 0 : kOutAsync;
    size_t taken = 0;
    while (taken < n) {
        if (open_used_ && (open_.cmd != who || open_.flags != kind ||
                           open_.len == kMaxChunk ||
                           (now_ms - open_t_ms_) >= kChunkMs)) {
            if (!close(0)) return taken;
        }
        if (!open_used_) {
            open_used_ = true;
            open_t_ms_ = now_ms;
            open_.cmd = who;
            open_.flags = kind;
            open_.len = 0;
        }
        const size_t room = kMaxChunk - open_.len;
        const size_t k = (n - taken) < room ? (n - taken) : room;
        memcpy(open_.text + open_.len, text + taken, k);
        open_.len = static_cast<uint16_t>(open_.len + k);
        taken += k;
    }
    return taken;
}

bool Server::endCommand(uint32_t seq, uint16_t extra_flags) {
    if (!established_ || seq != last_seq_ || state_ != CmdState::kRunning) {
        return true;   // the session went away underneath: nothing to close
    }
    if (open_used_ && open_.cmd != seq) {
        if (!close(0)) return false;
    }
    if (!open_used_) {
        open_used_ = true;
        open_.cmd = seq;
        open_.flags = 0;
        open_.len = 0;
    }
    if (!close(static_cast<uint16_t>(kOutDone | extra_flags))) return false;
    state_ = CmdState::kDone;
    return true;
}

bool Server::flush() {
    if (!open_used_ || state_ == CmdState::kRunning) return true;
    return close(0);
}

void Server::setLagging() { lagging_ = true; }

void Server::pump(Tx& tx, int max_chunks) {
    if (!established_) return;
    if (next_send_ < oldestHeld()) next_send_ = oldestHeld();
    for (int i = 0; i < max_chunks && next_send_ < next_seq_; ++i) {
        const Chunk& c = ring_[next_send_ % kRing];
        reply(tx, peer_, Type::kOut, sid_, c.seq, c.cmd, c.flags,
              reinterpret_cast<const uint8_t*>(c.text), c.len);
        ++next_send_;
    }
}

}  // namespace wcli
