// The Wi-Fi CLI's pure half (components/wcli): SHA-256/HMAC against the
// published vectors, the wire codec against link/wcliproto.py (generated
// vectors), the robot-side session logic -- authentication, replay and
// cross-session replay, at-most-once execution, cancel, busy, the output
// window -- and the Wi-Fi allowlist. docs/wifi-cli.md is what is asserted.
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "test_util.h"
#include "vectors/wcli_vectors.h"
#include "wcli/sha256.h"
#include "wcli/wcli.h"

namespace {

std::string hex(const uint8_t* p, size_t n) {
    std::string s;
    char b[3];
    for (size_t i = 0; i < n; ++i) {
        snprintf(b, sizeof b, "%02x", p[i]);
        s += b;
    }
    return s;
}

std::vector<uint8_t> unhex(const char* h) {
    std::vector<uint8_t> v;
    for (size_t i = 0; h[i] && h[i + 1]; i += 2) {
        unsigned x = 0;
        sscanf(h + i, "%2x", &x);
        v.push_back(static_cast<uint8_t>(x));
    }
    return v;
}

// -- SHA-256 / HMAC ------------------------------------------------------------
void testSha256() {
    uint8_t d[32];
    wcli::sha256(reinterpret_cast<const uint8_t*>("abc"), 3, d);
    CHECK(hex(d, 32) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    wcli::sha256(nullptr, 0, d);
    CHECK(hex(d, 32) == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    const char* m448 = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    wcli::sha256(reinterpret_cast<const uint8_t*>(m448), strlen(m448), d);
    CHECK(hex(d, 32) == "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    // A million 'a', fed in uneven pieces: exercises the block carry.
    wcli::Sha256 s;
    std::vector<uint8_t> a(1000, 'a');
    for (int i = 0; i < 1000; ++i) s.update(a.data(), 1000);
    s.final(d);
    CHECK(hex(d, 32) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
    wcli::Sha256 s2;
    size_t fed = 0, step = 1;
    while (fed < 1000) {
        const size_t k = (fed + step <= 1000) ? step : 1000 - fed;
        s2.update(a.data() + fed, k);
        fed += k;
        step = step * 3 % 97 + 1;
    }
    uint8_t d2[32];
    s2.final(d2);
    uint8_t d3[32];
    wcli::sha256(a.data(), 1000, d3);
    CHECK(memcmp(d2, d3, 32) == 0);
}

void testHmacRfc4231() {
    struct V { const char* key; const char* data; const char* mac; };
    const V vs[] = {
        // case 1
        {"0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b", "4869205468657265",
         "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"},
        // case 2 ("Jefe", "what do ya want for nothing?")
        {"4a656665", "7768617420646f2079612077616e7420666f72206e6f7468696e673f",
         "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"},
        // case 3
        {"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
         "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
         "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe"},
        // case 4
        {"0102030405060708090a0b0c0d0e0f10111213141516171819",
         "cdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcdcd",
         "82558a389a443c0ea4cc819899f2083a85f0faa3e578f8077a2e3ff46729665b"},
        // case 6: a 131-byte key (longer than the block -> hashed first)
        {"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
         "54657374205573696e67204c6172676572205468616e20426c6f636b2d53697a65204b6579202d2048617368204b6579204669727374",
         "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54"},
    };
    for (const V& v : vs) {
        const std::vector<uint8_t> k = unhex(v.key), m = unhex(v.data);
        uint8_t mac[32];
        wcli::hmacSha256(k.data(), k.size(), m.data(), m.size(), nullptr, 0, mac);
        CHECK(hex(mac, 32) == v.mac);
        // Split input == joined input (the codec MACs header + payload).
        const size_t half = m.size() / 2;
        uint8_t mac2[32];
        wcli::hmacSha256(k.data(), k.size(), m.data(), half, m.data() + half,
                         m.size() - half, mac2);
        CHECK(memcmp(mac, mac2, 32) == 0);
    }
}

void testCtEqual() {
    const uint8_t a[4] = {1, 2, 3, 4}, b[4] = {1, 2, 3, 4}, c[4] = {1, 2, 3, 5},
                  e[4] = {0, 2, 3, 4};
    CHECK(wcli::ctEqual(a, b, 4));
    CHECK(!wcli::ctEqual(a, c, 4));
    CHECK(!wcli::ctEqual(a, e, 4));
    CHECK(wcli::ctEqual(a, c, 0));
}

// -- wire, against the Python reference --------------------------------------------
void testVectors() {
    using namespace wcli_vectors;
    wcli::Keys k;
    wcli::deriveKeys(kKey, k);
    CHECK_BYTES(k.c2r, kC2R, 32);
    CHECK_BYTES(k.r2c, kR2C, 32);
    for (const Case& c : kCases) {
        const uint8_t* mk = c.c2r ? k.c2r : k.r2c;
        const uint8_t* other = c.c2r ? k.r2c : k.c2r;
        wcli::Msg m;
        m.type = static_cast<wcli::Type>(c.type);
        m.sid = c.sid;
        m.seq = c.seq;
        m.aux = c.aux;
        m.flags = c.flags;
        m.len = c.len;
        m.payload = c.payload;
        uint8_t buf[wcli::kMaxDatagram];
        const size_t n = wcli::encode(buf, sizeof buf, m, mk);
        CHECK_EQ(n, c.wire_len);
        CHECK_BYTES(buf, c.wire, c.wire_len);

        wcli::Msg d;
        CHECK(wcli::decode(c.wire, c.wire_len, mk, d) == wcli::Err::kOk);
        CHECK_EQ(static_cast<int>(d.type), c.type);
        CHECK(d.sid == c.sid);
        CHECK_EQ(d.seq, c.seq);
        CHECK_EQ(d.aux, c.aux);
        CHECK_EQ(d.flags, c.flags);
        CHECK_EQ(d.len, c.len);
        CHECK(c.len == 0 || memcmp(d.payload, c.payload, c.len) == 0);
        // The other direction's key: a reflected datagram is not authentic.
        CHECK(wcli::decode(c.wire, c.wire_len, other, d) == wcli::Err::kBadTag);
        // Every single-bit flip anywhere is caught (header, payload or tag).
        uint8_t mut[wcli::kMaxDatagram + 1];
        int caught = 0, tried = 0;
        for (size_t i = 0; i < c.wire_len; ++i) {
            for (int bit = 0; bit < 8; ++bit) {
                memcpy(mut, c.wire, c.wire_len);
                mut[i] = static_cast<uint8_t>(mut[i] ^ (1u << bit));
                ++tried;
                if (wcli::decode(mut, c.wire_len, mk, d) != wcli::Err::kOk) ++caught;
            }
        }
        CHECK_EQ(caught, tried);
        // Truncated / padded.
        CHECK(wcli::decode(c.wire, c.wire_len - 1, mk, d) != wcli::Err::kOk);
        memcpy(mut, c.wire, c.wire_len);
        mut[c.wire_len] = 0;
        CHECK(wcli::decode(mut, c.wire_len + 1, mk, d) == wcli::Err::kBadLength);
    }
    wcli::Msg d;
    const uint8_t junk[8] = {'B', 'W', 1, 2, 0, 0, 0, 0};
    CHECK(wcli::decode(junk, sizeof junk, k.c2r, d) == wcli::Err::kShort);
    // An oversize payload refuses to encode.
    wcli::Msg big;
    uint8_t pl[wcli::kMaxChunk + 1] = {};
    big.len = wcli::kMaxChunk + 1;
    big.payload = pl;
    uint8_t buf[wcli::kMaxDatagram + 8];
    CHECK_EQ(wcli::encode(buf, sizeof buf, big, k.c2r), 0);
}

// -- the session -----------------------------------------------------------------
struct Sent {
    wcli::Peer to;
    std::vector<uint8_t> bytes;
};

struct FakeTx : wcli::Tx {
    std::vector<Sent> sent;
    std::vector<std::pair<uint32_t, std::string>> submitted;
    bool queue_full = false;
    void send(const wcli::Peer& to, const uint8_t* b, size_t n) override {
        sent.push_back(Sent{to, std::vector<uint8_t>(b, b + n)});
    }
    bool submit(uint32_t seq, const char* line) override {
        if (queue_full) return false;
        submitted.emplace_back(seq, line);
        return true;
    }
    virtual ~FakeTx() = default;
};

const wcli::Peer kClient{0xC0A80205u, 50000};   // 192.168.2.5:50000

struct Rig {
    wcli::Server srv;
    FakeTx tx;
    wcli::Keys keys{};
    uint32_t now = 1000;
    uint64_t next_nonce = 0x5E5510000000001ull;
    uint64_t sid = 0;

    Rig() {
        srv.setKey(wcli_vectors::kKey);
        wcli::deriveKeys(wcli_vectors::kKey, keys);
    }

    std::vector<uint8_t> make(wcli::Type t, uint64_t s, uint32_t seq = 0,
                              uint32_t aux = 0, uint16_t flags = 0,
                              const char* text = nullptr) {
        wcli::Msg m;
        m.type = t;
        m.sid = s;
        m.seq = seq;
        m.aux = aux;
        m.flags = flags;
        m.len = static_cast<uint16_t>(text ? strlen(text) : 0);
        m.payload = reinterpret_cast<const uint8_t*>(text);
        uint8_t buf[wcli::kMaxDatagram];
        const size_t n = wcli::encode(buf, sizeof buf, m, keys.c2r);
        return std::vector<uint8_t>(buf, buf + n);
    }
    void feed(const std::vector<uint8_t>& d, const wcli::Peer& from = kClient) {
        srv.onDatagram(d.data(), d.size(), from, now, next_nonce++, tx);
    }
    // The last robot datagram, decoded with the robot->client key.
    bool last(wcli::Msg& m) {
        if (tx.sent.empty()) return false;
        const Sent& s = tx.sent.back();
        return wcli::decode(s.bytes.data(), s.bytes.size(), keys.r2c, m) ==
               wcli::Err::kOk;
    }
    uint64_t handshake(const wcli::Peer& from = kClient) {
        const uint64_t cn = 0xC11E47000000000ull + now;
        feed(make(wcli::Type::kHello, cn), from);
        wcli::Msg m;
        if (!last(m) || m.type != wcli::Type::kChallenge || m.len != 8) return 0;
        uint64_t echoed = 0;
        memcpy(&echoed, m.payload, 8);   // host is little-endian, like the wire
        if (echoed != cn) return 0;
        const uint64_t s = m.sid;
        feed(make(wcli::Type::kPoll, s, 0, 1), from);
        sid = s;
        return s;
    }
    // Run the queued command the way the housekeeping task does.
    void execute(uint32_t seq, const char* text) {
        if (!srv.beginCommand(seq)) return;
        srv.output(text, strlen(text), now);
        srv.endCommand(seq, 0);
    }
    // Every OUT the robot sent since index `from`, in order.
    std::vector<wcli::Msg> outs(size_t from, std::vector<std::string>* texts) {
        std::vector<wcli::Msg> v;
        for (size_t i = from; i < tx.sent.size(); ++i) {
            wcli::Msg m;
            const Sent& s = tx.sent[i];
            if (wcli::decode(s.bytes.data(), s.bytes.size(), keys.r2c, m) !=
                    wcli::Err::kOk ||
                m.type != wcli::Type::kOut)
                continue;
            v.push_back(m);
            if (texts) texts->emplace_back(reinterpret_cast<const char*>(m.payload), m.len);
        }
        return v;
    }
};

void testNoKeyAndForgery() {
    wcli::Server srv;
    FakeTx tx;
    wcli::Keys k;
    wcli::deriveKeys(wcli_vectors::kKey, k);
    // No key provisioned: nothing is ever answered.
    srv.onDatagram(wcli_vectors::k_hello_wire, sizeof wcli_vectors::k_hello_wire,
                   kClient, 0, 1, tx);
    CHECK(tx.sent.empty());
    CHECK_EQ(srv.counters().rx_nokey, 1);

    Rig r;
    // Wrong key: silently dropped -- no reply, no oracle.
    uint8_t other_key[32];
    memset(other_key, 0x77, sizeof other_key);
    wcli::Keys ok;
    wcli::deriveKeys(other_key, ok);
    wcli::Msg m;
    m.type = wcli::Type::kHello;
    m.sid = 99;
    uint8_t buf[wcli::kMaxDatagram];
    size_t n = wcli::encode(buf, sizeof buf, m, ok.c2r);
    r.srv.onDatagram(buf, n, kClient, r.now, 5, r.tx);
    CHECK(r.tx.sent.empty());
    CHECK_EQ(r.srv.counters().rx_bad, 1);
    // A robot->client message reflected back at the robot (right key
    // material, wrong direction): dropped.
    n = wcli::encode(buf, sizeof buf, m, r.keys.r2c);
    r.srv.onDatagram(buf, n, kClient, r.now, 5, r.tx);
    CHECK(r.tx.sent.empty());
    // A robot-type message under the CLIENT key is refused by type.
    m.type = wcli::Type::kOut;
    n = wcli::encode(buf, sizeof buf, m, r.keys.c2r);
    r.srv.onDatagram(buf, n, kClient, r.now, 5, r.tx);
    CHECK(r.tx.sent.empty());
    CHECK_EQ(r.srv.counters().rx_bad, 3);
    // Garbage of every length up to a datagram: never answered, never crashes.
    for (size_t len = 0; len < 64; ++len) {
        uint8_t g[64];
        for (size_t i = 0; i < len; ++i) g[i] = static_cast<uint8_t>(i * 37 + len);
        if (len > 1) { g[0] = 'B'; g[1] = 'W'; }
        r.srv.onDatagram(g, len, kClient, r.now, 5, r.tx);
    }
    CHECK(r.tx.sent.empty());
    CHECK(!r.srv.established());
}

void testHandshakeAndStale() {
    Rig r;
    const uint64_t s = r.handshake();
    CHECK(s != 0);
    CHECK(r.srv.established());
    CHECK(r.srv.peer() == kClient);
    wcli::Msg m;
    CHECK(r.last(m) && m.type == wcli::Type::kStatus && m.sid == s);
    CHECK_EQ(m.seq, 1);    // next output seq
    CHECK_EQ(m.aux, 0);    // no command yet
    // A sid the robot never issued (or an earlier boot's): STALE, authentic.
    r.feed(r.make(wcli::Type::kPoll, 0xDEAD, 0, 1));
    CHECK(r.last(m) && m.type == wcli::Type::kStale && m.sid == 0xDEAD);
    CHECK_EQ(r.srv.counters().rx_stale, 1);
    // The real session is unaffected.
    CHECK(r.srv.established());
    // A challenge expires.
    r.feed(r.make(wcli::Type::kHello, 1234));
    CHECK(r.last(m) && m.type == wcli::Type::kChallenge);
    const uint64_t s2 = m.sid;
    r.now += wcli::Server::kChallengeMs + 1;
    r.feed(r.make(wcli::Type::kPoll, s2, 0, 1));
    CHECK(r.last(m) && m.type == wcli::Type::kStale);
    CHECK(r.srv.established());
    CHECK(r.srv.peer() == kClient);
}

void testAtMostOnce() {
    Rig r;
    const uint64_t s = r.handshake();
    const auto cmd1 = r.make(wcli::Type::kCmd, s, 1, 0, 0, "ping");
    r.feed(cmd1);
    CHECK_EQ(r.tx.submitted.size(), 1);
    CHECK(r.tx.submitted[0].first == 1 && r.tx.submitted[0].second == "ping");
    wcli::Msg m;
    CHECK(r.last(m) && m.type == wcli::Type::kStatus && m.aux == 1 &&
          (m.flags & wcli::kStStateMask) == static_cast<int>(wcli::CmdState::kQueued));
    // Retransmit while queued, then after it ran: never a second submit.
    r.feed(cmd1);
    CHECK_EQ(r.tx.submitted.size(), 1);
    r.execute(1, "id 1: present\r\n");
    r.feed(cmd1);
    r.feed(cmd1);
    CHECK_EQ(r.tx.submitted.size(), 1);
    CHECK(r.last(m) && m.type == wcli::Type::kStatus && m.aux == 1 &&
          (m.flags & wcli::kStStateMask) == static_cast<int>(wcli::CmdState::kDone));
    CHECK_EQ(r.srv.counters().replays, 3);
    // seq 0 is never a command.
    r.feed(r.make(wcli::Type::kCmd, s, 0, 0, 0, "ping"));
    CHECK_EQ(r.tx.submitted.size(), 1);
    // A line longer than the CLI takes is malformed: dropped, unanswered.
    const std::string too_long(wcli::kMaxLine + 1, 'a');
    const size_t before = r.tx.sent.size();
    r.feed(r.make(wcli::Type::kCmd, s, 9, 0, 0, too_long.c_str()));
    CHECK_EQ(r.tx.submitted.size(), 1);
    CHECK_EQ(r.tx.sent.size(), before);
    CHECK_EQ(r.srv.lastSeq(), 1);
    // seq 3, then a delayed seq 2: the late one never runs.
    r.feed(r.make(wcli::Type::kCmd, s, 3, 0, 0, "pos 3"));
    CHECK_EQ(r.tx.submitted.size(), 2);
    r.execute(3, "");
    r.feed(r.make(wcli::Type::kCmd, s, 2, 0, 0, "pos 2"));
    CHECK_EQ(r.tx.submitted.size(), 2);
    CHECK_EQ(r.srv.lastSeq(), 3);
}

void testCrossSessionReplay() {
    Rig r;
    const uint64_t a = r.handshake();
    const auto old_cmd = r.make(wcli::Type::kCmd, a, 1, 0, 0, "home");
    r.feed(old_cmd);
    r.execute(1, "homing\r\n");
    CHECK_EQ(r.tx.submitted.size(), 1);
    // The client (or an attacker replaying its HELLO) makes a new session.
    r.now += 100;
    const uint64_t b = r.handshake();
    CHECK(b != 0 && b != a);
    CHECK_EQ(r.srv.lastSeq(), 0);   // seqs restart with the session...
    // ...and session A's captured `home` (seq 1 > 0!) is dead on arrival.
    r.feed(old_cmd);
    CHECK_EQ(r.tx.submitted.size(), 1);
    wcli::Msg m;
    CHECK(r.last(m) && m.type == wcli::Type::kStale && m.sid == a);
    // A replayed HELLO only earns a challenge: the session stays current.
    r.feed(r.make(wcli::Type::kHello, 0xC11E47000000000ull));
    CHECK(r.last(m) && m.type == wcli::Type::kChallenge);
    r.feed(r.make(wcli::Type::kCmd, b, 1, 0, 0, "ping"));
    CHECK_EQ(r.tx.submitted.size(), 2);
    CHECK_EQ(r.srv.counters().sessions, 2);
}

void testWrongPeer() {
    Rig r;
    const uint64_t s = r.handshake();
    const wcli::Peer other{0xC0A80263u, 50000};   // .99, same port
    const size_t before = r.tx.sent.size();
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "release"), other);
    r.feed(r.make(wcli::Type::kPoll, s, 0, 1, wcli::kPollResend), other);
    CHECK_EQ(r.tx.submitted.size(), 0);
    CHECK_EQ(r.tx.sent.size(), before);            // not even answered
    CHECK_EQ(r.srv.counters().rx_wrong_peer, 2);
    const wcli::Peer port{kClient.ip, 50001};
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "release"), port);
    CHECK_EQ(r.tx.submitted.size(), 0);
    // A challenge is bound to the address that asked for it.
    r.feed(r.make(wcli::Type::kHello, 77), kClient);
    wcli::Msg m;
    CHECK(r.last(m) && m.type == wcli::Type::kChallenge);
    r.feed(r.make(wcli::Type::kPoll, m.sid, 0, 1), other);
    CHECK(r.last(m) && m.type == wcli::Type::kStale);
    CHECK(r.srv.peer() == kClient);
}

void testBusyAndQueueFull() {
    Rig r;
    const uint64_t s = r.handshake();
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "scan"));
    // A second command while the first is queued: refused, NOT consumed.
    r.feed(r.make(wcli::Type::kCmd, s, 2, 0, 0, "ping"));
    wcli::Msg m;
    CHECK(r.last(m) && (m.flags & wcli::kStBusy) && m.aux == 1);
    CHECK_EQ(r.tx.submitted.size(), 1);
    // A new session cannot take over while a command is in flight.
    r.feed(r.make(wcli::Type::kHello, 5), kClient);
    CHECK(r.last(m) && m.type == wcli::Type::kChallenge);
    const uint64_t s2 = m.sid;
    r.feed(r.make(wcli::Type::kPoll, s2, 0, 1));
    CHECK(r.last(m) && m.type == wcli::Type::kStatus && (m.flags & wcli::kStBusy) &&
          m.sid == s2);
    CHECK(r.srv.established());
    r.execute(1, "0 servo(s)\r\n");
    r.feed(r.make(wcli::Type::kCmd, s, 2, 0, 0, "ping"));
    CHECK_EQ(r.tx.submitted.size(), 2);
    r.execute(2, "");
    // Queue full: BUSY, seq not consumed, the retransmit later gets in.
    r.tx.queue_full = true;
    const auto c3 = r.make(wcli::Type::kCmd, s, 3, 0, 0, "volt");
    r.feed(c3);
    CHECK(r.last(m) && (m.flags & wcli::kStBusy));
    CHECK_EQ(r.srv.lastSeq(), 2);
    r.tx.queue_full = false;
    r.feed(c3);
    CHECK_EQ(r.tx.submitted.size(), 3);
    CHECK_EQ(r.srv.lastSeq(), 3);
    // The busy-session challenge still works once the command finished.
    r.execute(3, "");
    r.feed(r.make(wcli::Type::kPoll, s2, 0, 1));
    CHECK(r.last(m) && m.type == wcli::Type::kStatus && !(m.flags & wcli::kStBusy));
    CHECK(r.srv.established());
}

void testCancel() {
    Rig r;
    const uint64_t s = r.handshake();
    // Cancel a seq the robot never saw: it is burned, a late copy is dead.
    r.feed(r.make(wcli::Type::kCancel, s, 1));
    wcli::Msg m;
    CHECK(r.last(m) && m.aux == 1 &&
          (m.flags & wcli::kStStateMask) == static_cast<int>(wcli::CmdState::kCancelled));
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "home"));
    CHECK_EQ(r.tx.submitted.size(), 0);
    // Cancel a QUEUED one: the executor is told not to run it.
    r.feed(r.make(wcli::Type::kCmd, s, 2, 0, 0, "home"));
    CHECK_EQ(r.tx.submitted.size(), 1);
    r.feed(r.make(wcli::Type::kCancel, s, 2));
    CHECK(!r.srv.beginCommand(2));
    // Cancel a RUNNING one: too late, and the status says so.
    r.feed(r.make(wcli::Type::kCmd, s, 3, 0, 0, "scan"));
    CHECK(r.srv.beginCommand(3));
    r.feed(r.make(wcli::Type::kCancel, s, 3));
    CHECK(r.last(m) && m.aux == 3 &&
          (m.flags & wcli::kStStateMask) == static_cast<int>(wcli::CmdState::kRunning));
    CHECK(r.srv.endCommand(3, 0));
    // Give up on a NEW seq while another command runs (the client abandoned
    // seq 4, robot still busy with it): seq 5 is burned, seq 4 is untouched
    // and still ends with its DONE.
    r.feed(r.make(wcli::Type::kCmd, s, 4, 0, 0, "scan"));
    CHECK(r.srv.beginCommand(4));
    r.feed(r.make(wcli::Type::kCmd, s, 5, 0, 0, "home"));
    CHECK(r.last(m) && (m.flags & wcli::kStBusy));
    r.feed(r.make(wcli::Type::kCancel, s, 5));
    CHECK(r.last(m) && m.aux == 5 &&
          (m.flags & wcli::kStStateMask) == static_cast<int>(wcli::CmdState::kCancelled));
    CHECK_EQ(r.srv.state(), static_cast<int>(wcli::CmdState::kRunning));
    CHECK(r.srv.endCommand(4, 0));
    const size_t n_sub = r.tx.submitted.size();
    r.feed(r.make(wcli::Type::kCmd, s, 5, 0, 0, "home"));   // the late copy
    CHECK_EQ(r.tx.submitted.size(), n_sub);
    r.feed(r.make(wcli::Type::kCmd, s, 6, 0, 0, "ping"));
    CHECK_EQ(r.tx.submitted.size(), n_sub + 1);
    CHECK(r.last(m) && m.aux == 6 &&
          (m.flags & wcli::kStStateMask) == static_cast<int>(wcli::CmdState::kQueued));
}

void testOutputStream() {
    Rig r;
    const uint64_t s = r.handshake();
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "trace"));
    CHECK(r.srv.beginCommand(1));
    // 1000 bytes of output -> 3 chunks (400, 400, 200 + DONE), text intact.
    std::string text;
    for (int i = 0; i < 50; ++i) {
        char line[40];
        snprintf(line, sizeof line, "line %03d xxxxxxxxx\r\n", i % 1000);   // 20 B
        text += line;
        CHECK_EQ(r.srv.output(line, strlen(line), r.now), strlen(line));
    }
    CHECK(r.srv.endCommand(1, 0));
    const size_t from = r.tx.sent.size();
    r.srv.pump(r.tx, 16);
    std::vector<std::string> texts;
    const auto outs = r.outs(from, &texts);
    CHECK_EQ(outs.size(), 3);
    std::string joined;
    for (const auto& t : texts) joined += t;
    CHECK(joined == text);
    for (size_t i = 0; i < outs.size(); ++i) {
        CHECK_EQ(outs[i].seq, i + 1);
        CHECK_EQ(outs[i].aux, 1);
        CHECK_EQ(outs[i].flags, i + 1 == outs.size() ? wcli::kOutDone : 0);
    }
    // Pumping again sends nothing new.
    const size_t n1 = r.tx.sent.size();
    r.srv.pump(r.tx, 16);
    CHECK_EQ(r.tx.sent.size(), n1);
    // A lost chunk: POLL with RESEND from 2 re-sends 2 and 3, only.
    r.feed(r.make(wcli::Type::kPoll, s, 0, 2, wcli::kPollResend));
    const size_t from2 = r.tx.sent.size();
    r.srv.pump(r.tx, 16);
    const auto again = r.outs(from2, nullptr);
    CHECK_EQ(again.size(), 2);
    CHECK(again.size() == 2 && again[0].seq == 2 && again[1].seq == 3);
    // A command with no output still ends with a (empty) DONE chunk; a
    // refused one carries kOutRefused.
    r.feed(r.make(wcli::Type::kCmd, s, 2, 0, 0, "id 1 2"));
    CHECK(r.srv.beginCommand(2));
    const char* why = "refused over Wi-Fi (tether only): x\r\n";
    r.srv.output(why, strlen(why), r.now);
    CHECK(r.srv.endCommand(2, wcli::kOutRefused));
    r.feed(r.make(wcli::Type::kCmd, s, 3, 0, 0, ""));
    CHECK(r.srv.beginCommand(3));
    CHECK(r.srv.endCommand(3, 0));
    const size_t from3 = r.tx.sent.size();
    r.srv.pump(r.tx, 16);
    const auto tail = r.outs(from3, nullptr);
    CHECK_EQ(tail.size(), 2);
    CHECK(tail.size() == 2 && tail[0].flags == (wcli::kOutDone | wcli::kOutRefused) &&
          tail[0].aux == 2);
    CHECK(tail.size() == 2 && tail[1].flags == wcli::kOutDone && tail[1].len == 0 &&
          tail[1].aux == 3);
}

void testAsyncAndChunkTimer() {
    Rig r;
    // No session: async output is dropped on the floor (and says it took it).
    CHECK_EQ(r.srv.output("x\r\n", 3, r.now), 3);
    CHECK(r.srv.flush());
    const uint64_t s = r.handshake();
    const char* bal = "balance: landed\r\n";
    CHECK_EQ(r.srv.output(bal, strlen(bal), r.now), strlen(bal));
    CHECK(r.srv.flush());
    size_t from = r.tx.sent.size();
    r.srv.pump(r.tx, 8);
    std::vector<std::string> texts;
    auto o = r.outs(from, &texts);
    CHECK(o.size() == 1 && o[0].flags == wcli::kOutAsync && o[0].aux == 0 &&
          texts[0] == bal);
    // The client went quiet for kIdleMs: async output is no longer kept.
    r.now += wcli::Server::kIdleMs + 1;
    r.srv.output(bal, strlen(bal), r.now);
    r.srv.flush();
    from = r.tx.sent.size();
    r.srv.pump(r.tx, 8);
    CHECK_EQ(r.outs(from, nullptr).size(), 0);
    // A long command streams: a partial chunk older than kChunkMs is closed
    // by the next print.
    r.feed(r.make(wcli::Type::kPoll, s, 0, r.srv.outNext()));
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "scan"));
    CHECK(r.srv.beginCommand(1));
    r.srv.output("a\r\n", 3, r.now);
    r.now += wcli::Server::kChunkMs;
    r.srv.output("b\r\n", 3, r.now);
    from = r.tx.sent.size();
    r.srv.pump(r.tx, 8);
    texts.clear();
    o = r.outs(from, &texts);
    CHECK(o.size() == 1 && texts[0] == "a\r\n" && o[0].flags == 0);
    CHECK(r.srv.endCommand(1, 0));
}

void testWindowAndLagging() {
    Rig r;
    const uint64_t s = r.handshake();
    r.feed(r.make(wcli::Type::kCmd, s, 1, 0, 0, "bno dump"));
    CHECK(r.srv.beginCommand(1));
    char blob[wcli::kMaxChunk];
    memset(blob, 'z', sizeof blob);
    // Fill the window: kRing closed chunks can be outstanding unacknowledged.
    size_t total = 0;
    for (uint32_t i = 0; i <= wcli::Server::kRing; ++i) {
        total += r.srv.output(blob, sizeof blob, r.now);
    }
    // kRing closed + the open one (full) = (kRing + 1) * 400 taken; then the
    // robot WAITS for a client it heard from just now.
    CHECK_EQ(total, (wcli::Server::kRing + 1) * wcli::kMaxChunk);
    CHECK_EQ(r.srv.output(blob, 10, r.now), 0);
    CHECK(!r.srv.endCommand(1, 0));
    CHECK_EQ(r.srv.counters().lost_chunks, 0);
    // The client ACKs 10 chunks: room for exactly 10 more.
    r.now += 50;
    r.feed(r.make(wcli::Type::kPoll, s, 0, 11));
    size_t more = 0;
    for (int i = 0; i < 12; ++i) more += r.srv.output(blob, sizeof blob, r.now);
    CHECK_EQ(more, 10 * wcli::kMaxChunk);
    // Still talking but not acknowledging: the caller's stall bound gives up
    // (setLagging); the oldest chunks are overwritten and counted.
    r.srv.setLagging();
    CHECK_EQ(r.srv.output(blob, sizeof blob, r.now), sizeof blob);
    CHECK(r.srv.counters().lost_chunks > 0);
    wcli::Msg m;
    r.feed(r.make(wcli::Type::kPoll, s, 0, 11));
    CHECK(r.last(m) && (m.flags & wcli::kStLagging));
    CHECK(!r.srv.lagging());                  // ...and that POLL cleared it
    uint32_t oldest = 0;
    memcpy(&oldest, m.payload, 4);
    CHECK_EQ(oldest, r.srv.oldestHeld());
    CHECK(oldest > 11);
    // A resend from below the oldest held starts AT the oldest.
    r.feed(r.make(wcli::Type::kPoll, s, 0, 11, wcli::kPollResend));
    size_t from = r.tx.sent.size();
    r.srv.pump(r.tx, 1);
    auto o = r.outs(from, nullptr);
    CHECK(o.size() == 1 && o[0].seq == oldest);
    // Window full again, client alive: keep waiting...
    while (r.srv.output(blob, sizeof blob, r.now) > 0) {}
    const uint32_t lost = r.srv.counters().lost_chunks;
    CHECK(!r.srv.shouldStopWaiting(500, r.now));
    CHECK(r.srv.shouldStopWaiting(wcli::Server::kStallMs, r.now));   // ...but not forever
    // A client silent for kAckWaitMs is gone -- judged only after kMinWaitMs
    // of waiting (the network task may not have read its POLLs yet).
    r.now += wcli::Server::kAckWaitMs;
    CHECK(!r.srv.shouldStopWaiting(wcli::Server::kMinWaitMs - 1, r.now));
    CHECK(r.srv.shouldStopWaiting(wcli::Server::kMinWaitMs, r.now));
    CHECK_EQ(r.srv.output(blob, sizeof blob, r.now), 0);   // the Server never decides alone
    r.srv.setLagging();
    CHECK_EQ(r.srv.output(blob, sizeof blob, r.now), sizeof blob);
    CHECK(r.srv.endCommand(1, 0));
    CHECK(r.srv.counters().lost_chunks > lost);
    // Everything closed is sendable, in order, from the oldest held.
    r.feed(r.make(wcli::Type::kPoll, s, 0, 1, wcli::kPollResend));
    from = r.tx.sent.size();
    r.srv.pump(r.tx, 100);
    o = r.outs(from, nullptr);
    CHECK_EQ(o.size(), wcli::Server::kRing);
    for (size_t i = 1; i < o.size(); ++i) CHECK_EQ(o[i].seq, o[i - 1].seq + 1);
    CHECK(!o.empty() && o.back().flags == wcli::kOutDone);
}

void testKeyChangeDropsSession() {
    Rig r;
    r.handshake();
    CHECK(r.srv.established());
    uint8_t k2[32];
    memset(k2, 0x11, sizeof k2);
    r.srv.setKey(k2);
    CHECK(!r.srv.established());
    r.srv.clearKey();
    CHECK(!r.srv.hasKey());
    const size_t n = r.tx.sent.size();
    r.feed(r.make(wcli::Type::kHello, 1));
    CHECK_EQ(r.tx.sent.size(), n);
}

// -- the allowlist ------------------------------------------------------------------
void testPolicy() {
    using wcli::Verdict;
    struct Row { const char* line; bool armed; Verdict want; };
    const Row rows[] = {
        // the bench work the Wi-Fi CLI exists for
        {"ping", false, Verdict::kAllow},
        {"ping 7", false, Verdict::kAllow},
        {"pos 7", false, Verdict::kAllow},
        {"pose 2048 2048 2048 2048 2048 2048 2048 2048 2048 2048 s1500", false, Verdict::kAllow},
        {"balance", false, Verdict::kAllow},
        {"balance set 1 2 3 4", false, Verdict::kAllow},
        {"home", false, Verdict::kAllow},
        {"release", false, Verdict::kAllow},
        {"torque 7", false, Verdict::kAllow},
        {"move 7 2100", false, Verdict::kAllow},
        {"sread", false, Verdict::kAllow},
        {"rec on", false, Verdict::kAllow},          // RAM-only over Wi-Fi (cli.cpp)
        {"imu", false, Verdict::kAllow},
        {"imu raw 5", false, Verdict::kAllow},
        {"imu gscale", false, Verdict::kAllow},
        {"imu mount", false, Verdict::kAllow},
        {"gains", false, Verdict::kAllow},
        {"gains 7", false, Verdict::kAllow},
        {"cal", false, Verdict::kAllow},
        {"cal show", false, Verdict::kAllow},
        {"wifi", false, Verdict::kAllow},
        {"wcli", false, Verdict::kAllow},
        {"batt", false, Verdict::kAllow},
        {"stream status", false, Verdict::kAllow},
        {"obsdump", false, Verdict::kAllow},
        {"  ping\t7 ", false, Verdict::kAllow},
        {"", false, Verdict::kAllow},
        // NVS / EEPROM writers, safety latches, unsafe tools: tether only
        {"id 1 2", false, Verdict::kRefuseTether},
        {"middle 7", false, Verdict::kRefuseTether},
        {"cal zero", false, Verdict::kRefuseTether},
        {"cal save", false, Verdict::kRefuseTether},
        {"cal set L_knee 2048 1", false, Verdict::kRefuseTether},
        {"cal load", false, Verdict::kRefuseTether},
        {"cal reset", false, Verdict::kRefuseTether},
        {"cal show extra", false, Verdict::kRefuseTether},
        {"gains 7 40 32", false, Verdict::kRefuseTether},
        {"imu bias", false, Verdict::kRefuseTether},
        {"imu mount 0.7071 0 0.7071 0", false, Verdict::kRefuseTether},
        {"imu gscale 1 1 1", false, Verdict::kRefuseTether},
        {"imu forget", false, Verdict::kRefuseTether},
        {"wifi myssid mypsk", false, Verdict::kRefuseTether},
        {"wifi clear", false, Verdict::kRefuseTether},
        {"wcli key 00", false, Verdict::kRefuseTether},
        {"wcli key clear", false, Verdict::kRefuseTether},
        {"batt reset", false, Verdict::kRefuseTether},
        {"run", false, Verdict::kRefuseTether},
        {"stream 7 2000 2100 1000 0", false, Verdict::kRefuseTether},
        {"obsdump on", false, Verdict::kRefuseTether},
        // default deny
        {"imu frobnicate", false, Verdict::kRefuseUnknown},
        {"frobnicate", false, Verdict::kRefuseUnknown},
        {"PING", false, Verdict::kRefuseUnknown},
        // armed: status reads only
        {"stat", true, Verdict::kAllow},
        {"help", true, Verdict::kAllow},
        {"wifi", true, Verdict::kAllow},
        {"rec status", true, Verdict::kAllow},
        {"cal show", true, Verdict::kAllow},
        {"ping", true, Verdict::kRefuseArmed},
        {"pose 2048", true, Verdict::kRefuseArmed},
        {"bench", true, Verdict::kRefuseArmed},
        {"release", true, Verdict::kRefuseArmed},
        {"rec on", true, Verdict::kRefuseArmed},
        {"shape 3", true, Verdict::kRefuseArmed},
        {"id 1 2", true, Verdict::kRefuseArmed},
        // bytes the tether's line discipline would never deliver
        {"ping\x01", false, Verdict::kRefuseBadLine},
        {"ping\x7f", false, Verdict::kRefuseBadLine},
        {"pin\xc3\xa9", false, Verdict::kRefuseBadLine},
        {"ping\r", false, Verdict::kRefuseBadLine},
    };
    for (const Row& row : rows) {
        const wcli::Decision d = wcli::classify(row.line, row.armed);
        if (d.verdict != row.want) {
            fprintf(stderr, "  classify(\"%s\", armed=%d) = %d, want %d\n", row.line,
                    row.armed, static_cast<int>(d.verdict), static_cast<int>(row.want));
        }
        CHECK(d.verdict == row.want);
        char msg[200];
        const size_t n = wcli::formatRefusal(d, row.line, msg, sizeof msg);
        if (row.want == Verdict::kAllow) {
            CHECK_EQ(n, 0);
        } else {
            CHECK(n > 10 && n < sizeof msg);
            CHECK(strncmp(msg, "refused over Wi-Fi", 18) == 0);
            CHECK(msg[n - 2] == '\r' && msg[n - 1] == '\n');
        }
    }
    // The refusal names the command word and never echoes the arguments.
    char msg[200];
    wcli::formatRefusal(wcli::classify("frobnicate secretarg", false),
                        "frobnicate secretarg", msg, sizeof msg);
    CHECK(strstr(msg, "`frobnicate`") != nullptr);
    CHECK(strstr(msg, "secretarg") == nullptr);
    wcli::formatRefusal(wcli::classify("wifi ssid hunter2", false),
                        "wifi ssid hunter2", msg, sizeof msg);
    CHECK(strstr(msg, "hunter2") == nullptr);
    // Truncation is safe.
    char tiny[8];
    const size_t n = wcli::formatRefusal(wcli::classify("id 1 2", false), "id 1 2",
                                         tiny, sizeof tiny);
    CHECK(n == 7 && tiny[7] == 0);
    // A line longer than the CLI takes.
    std::string longline(wcli::kMaxLine + 1, 'a');
    CHECK(wcli::classify(longline.c_str(), false).verdict == Verdict::kRefuseBadLine);
}

void testRedact() {
    char out[200];
    wcli::redactForLog("wcli key 00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff",
                       out, sizeof out);
    CHECK(strcmp(out, "wcli key <redacted>") == 0);
    wcli::redactForLog("  wcli   key  deadbeef", out, sizeof out);
    CHECK(strcmp(out, "wcli key <redacted>") == 0);
    wcli::redactForLog("wcli key clear", out, sizeof out);
    CHECK(strcmp(out, "wcli key clear") == 0);
    wcli::redactForLog("wifi homenet hunter2", out, sizeof out);
    CHECK(strcmp(out, "wifi homenet <redacted>") == 0);
    wcli::redactForLog("wifi clear", out, sizeof out);
    CHECK(strcmp(out, "wifi clear") == 0);
    wcli::redactForLog("pose 2048 s100", out, sizeof out);
    CHECK(strcmp(out, "pose 2048 s100") == 0);
    char tiny[6];
    wcli::redactForLog("wifi homenet hunter2", tiny, sizeof tiny);
    CHECK(strlen(tiny) == 5 && strstr(tiny, "hunt") == nullptr);
}

}  // namespace

int main() {
    testSha256();
    testHmacRfc4231();
    testCtEqual();
    testVectors();
    testNoKeyAndForgery();
    testHandshakeAndStale();
    testAtMostOnce();
    testCrossSessionReplay();
    testWrongPeer();
    testBusyAndQueueFull();
    testCancel();
    testOutputStream();
    testAsyncAndChunkTimer();
    testWindowAndLagging();
    testKeyChangeDropsSession();
    testPolicy();
    testRedact();
    return testutil::report("wcli");
}
