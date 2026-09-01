#include "mjpeg.h"

#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "client.h"   // nowMs()

namespace bimo {
namespace {

// A frame bigger than this is a stream we do not understand; refuse it rather
// than let a bad Content-Length allocate the machine to death.
constexpr size_t kMaxFrame = 8u * 1024u * 1024u;
// Likewise a "header block" that never ends.
constexpr size_t kMaxHeaders = 64u * 1024u;

const uint8_t* findi(const uint8_t* hay, size_t n, const char* needle) {
    const size_t m = strlen(needle);
    if (m == 0 || n < m) return nullptr;
    for (size_t i = 0; i + m <= n; ++i) {
        if (strncasecmp(reinterpret_cast<const char*>(hay + i), needle, m) == 0) {
            return hay + i;
        }
    }
    return nullptr;
}

const uint8_t* findb(const uint8_t* hay, size_t n, const char* needle) {
    const size_t m = strlen(needle);
    if (m == 0 || n < m) return nullptr;
    for (size_t i = 0; i + m <= n; ++i) {
        if (memcmp(hay + i, needle, m) == 0) return hay + i;
    }
    return nullptr;
}

bool growBuf(uint8_t** buf, size_t* cap, size_t need) {
    if (need <= *cap) return true;
    if (need > kMaxFrame) return false;
    size_t c = *cap ? *cap : 8192;
    while (c < need) c *= 2;
    if (c > kMaxFrame) c = kMaxFrame;
    uint8_t* p = static_cast<uint8_t*>(realloc(*buf, c));
    if (p == nullptr) return false;
    *buf = p;
    *cap = c;
    return true;
}

}  // namespace

// -- MjpegParser ------------------------------------------------------------
MjpegParser::MjpegParser() {}

MjpegParser::~MjpegParser() {
    free(acc_);
    free(out_);
}

void MjpegParser::reset() {
    state_ = kHeaders;
    acc_len_ = 0;
    out_len_ = 0;
    want_ = 0;
    ready_ = false;
    overflow_ = false;
}

// Consume as much of the accumulator as possible. Idempotent and safe to call
// with nothing new, which is how take() surfaces a second frame that already
// arrived in the same read.
void MjpegParser::advance() {
    while (!ready_ && !overflow_) {
        if (state_ == kHeaders) {
            // The HTTP response's own headers arrive first and carry no
            // Content-Length; that block is dropped and scanning continues,
            // which is also what happens to each part's boundary line.
            const uint8_t* end = findb(acc_, acc_len_, "\r\n\r\n");
            if (end == nullptr) {
                if (acc_len_ > kMaxHeaders) overflow_ = true;
                return;
            }
            const size_t block = static_cast<size_t>(end - acc_) + 4;
            const uint8_t* cl = findi(acc_, block, "content-length:");
            size_t body = 0;
            if (cl != nullptr) {
                body = strtoul(reinterpret_cast<const char*>(cl) + 15, nullptr,
                               10);
            }
            memmove(acc_, acc_ + block, acc_len_ - block);
            acc_len_ -= block;
            if (cl == nullptr || body == 0) continue;   // not a part
            if (body > kMaxFrame) {
                overflow_ = true;
                return;
            }
            want_ = body;
            state_ = kBody;
        } else {
            if (acc_len_ < want_) return;               // need more bytes
            if (!growBuf(&out_, &out_cap_, want_)) {
                overflow_ = true;
                return;
            }
            memcpy(out_, acc_, want_);
            out_len_ = want_;
            memmove(acc_, acc_ + want_, acc_len_ - want_);
            acc_len_ -= want_;
            want_ = 0;
            state_ = kHeaders;
            ready_ = true;
        }
    }
}

void MjpegParser::feed(const uint8_t* data, size_t n) {
    if (overflow_) return;
    if (data != nullptr && n > 0) {
        if (!growBuf(&acc_, &acc_cap_, acc_len_ + n)) {
            overflow_ = true;
            return;
        }
        memcpy(acc_ + acc_len_, data, n);
        acc_len_ += n;
    }
    advance();
}

bool MjpegParser::take(const uint8_t** out, size_t* len) {
    // Deliberately advance BEFORE handing the pointer out, never after: two
    // parts can arrive in one read, and advancing afterwards would overwrite
    // the frame the caller is still holding.
    if (!ready_) advance();
    if (!ready_) return false;
    *out = out_;
    *len = out_len_;
    ready_ = false;
    return true;
}

// -- MjpegStream ------------------------------------------------------------
MjpegStream::MjpegStream() {}

MjpegStream::~MjpegStream() {
    stop();
    free(frame_);
}

void MjpegStream::freeFrame(uint8_t* p) { free(p); }

bool MjpegStream::start(const char* u) {
    stop();
    snprintf(url, sizeof url, "%s", u);
    err_[0] = 0;
    frames_ = 0;
    last_frame_ms_ = 0.0;
    stop_ = false;
    running_ = true;
    if (pthread_create(&thread_, nullptr, &MjpegStream::trampoline, this) != 0) {
        running_ = false;
        snprintf(err_, sizeof err_, "cannot start the stream thread");
        return false;
    }
    started_ = true;
    return true;
}

void MjpegStream::stop() {
    if (!started_) return;
    stop_ = true;
    pthread_join(thread_, nullptr);
    started_ = false;
    running_ = false;
}

bool MjpegStream::latest(uint64_t* seq, uint8_t** out, size_t* len) {
    bool got = false;
    pthread_mutex_lock(&lock_);
    if (frame_ != nullptr && frame_seq_ > *seq) {
        uint8_t* copy = static_cast<uint8_t*>(malloc(frame_len_));
        if (copy != nullptr) {
            memcpy(copy, frame_, frame_len_);
            *out = copy;
            *len = frame_len_;
            *seq = frame_seq_;
            got = true;
        }
    }
    pthread_mutex_unlock(&lock_);
    return got;
}

void MjpegStream::setError(const char* fmt, const char* a) {
    pthread_mutex_lock(&lock_);
    snprintf(err_, sizeof err_, fmt, a);
    pthread_mutex_unlock(&lock_);
}

void MjpegStream::setErrorPair(const char* fmt, const char* a, const char* b) {
    pthread_mutex_lock(&lock_);
    snprintf(err_, sizeof err_, fmt, a, b);
    pthread_mutex_unlock(&lock_);
}

void MjpegStream::error(char* out, size_t cap) const {
    pthread_mutex_lock(&lock_);
    snprintf(out, cap, "%s", err_);
    pthread_mutex_unlock(&lock_);
}

void* MjpegStream::trampoline(void* self) {
    static_cast<MjpegStream*>(self)->run();
    return nullptr;
}

// One connection attempt, streaming until it fails. Returns false if the
// connect itself did not happen (so the caller can keep waiting for a server
// that has not started yet).
bool MjpegStream::attempt(const char* host, const char* port,
                          const char* path) {
    addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (getaddrinfo(host, port, &hints, &res) != 0 || res == nullptr) {
        setErrorPair("cannot resolve %s:%s", host, port);
        return false;
    }
    const int fd = socket(res->ai_family, res->ai_socktype, 0);
    if (fd < 0 || connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        setErrorPair("waiting for a stream at %s:%s", host, port);
        if (fd >= 0) ::close(fd);
        freeaddrinfo(res);
        return false;
    }
    freeaddrinfo(res);

    char req[sizeof path + sizeof host + 64];
    const int rn = snprintf(req, sizeof req,
                            "GET %s HTTP/1.0\r\nHost: %s\r\n"
                            "User-Agent: bimo_gui\r\n\r\n",
                            path, host);
    if (rn <= 0 || static_cast<size_t>(rn) >= sizeof req ||
        write(fd, req, static_cast<size_t>(rn)) != rn) {
        setError("cannot send the request to %s", host);
        ::close(fd);
        return false;
    }
    setError("%s", "");                  // connected: clear the waiting note

    // A short timeout so stop() is honoured promptly and a silent server is
    // noticed rather than waited on forever.
    timeval tv = {0, 200 * 1000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

    MjpegParser parser;
    uint8_t rx[16384];
    while (!stop_) {
        const ssize_t n = recv(fd, rx, sizeof rx, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            setError("stream read failed: %s", strerror(errno));
            break;
        }
        if (n == 0) {
            setError("%s", "the stream closed");
            break;
        }
        parser.feed(rx, static_cast<size_t>(n));
        if (parser.overflowed()) {
            setError("%s", "unparseable stream (not MJPEG?)");
            break;
        }
        const uint8_t* jpeg = nullptr;
        size_t len = 0;
        while (parser.take(&jpeg, &len)) {
            pthread_mutex_lock(&lock_);
            uint8_t* nf = static_cast<uint8_t*>(realloc(frame_, len));
            if (nf != nullptr) {
                frame_ = nf;
                memcpy(frame_, jpeg, len);
                frame_len_ = len;
                ++frame_seq_;
                ++frames_;
                last_frame_ms_ = nowMs();
            }
            pthread_mutex_unlock(&lock_);
        }
    }
    ::close(fd);
    return true;
}

void MjpegStream::run() {
    // http://host[:port][/path]
    // `path` is a SUFFIX of url, so it is sized to hold the whole of one --
    // gcc's -Wformat-truncation is right that a 256 B path could not, and
    // sizing the destination removes the possibility rather than silencing
    // the warning. Likewise `req` is sized to hold path + host + the fixed
    // text with room to spare, so the request can never be half-formed.
    char host[256] = "127.0.0.1", port[16] = "80";
    char path[sizeof url] = "/";
    const char* p = url;
    if (!strncmp(p, "http://", 7)) p += 7;
    size_t i = 0;
    while (*p && *p != ':' && *p != '/' && i + 1 < sizeof host) host[i++] = *p++;
    host[i] = 0;
    if (*p == ':') {
        ++p;
        i = 0;
        while (*p && *p != '/' && i + 1 < sizeof port) port[i++] = *p++;
        port[i] = 0;
    }
    if (*p == '/') snprintf(path, sizeof path, "%s", p);

    // Keep trying until stop(). The natural order of operations is to press
    // Start and then wait -- sil_twin spends ~25 s loading MuJoCo and the
    // policy before it listens at all -- so a single connect attempt would
    // fail every single time it matters. A dropped stream reconnects for the
    // same reason.
    while (!stop_) {
        attempt(host, port, path);
        for (int k = 0; k < 10 && !stop_; ++k) usleep(50 * 1000);
    }
    running_ = false;
}

}  // namespace bimo
