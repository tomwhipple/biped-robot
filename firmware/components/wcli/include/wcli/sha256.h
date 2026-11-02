// SHA-256 and HMAC-SHA256 (FIPS 180-4, RFC 2104), for the Wi-Fi CLI's
// message authentication (wcli.h, docs/wifi-cli.md).
//
// Portable and dependency-free on purpose: the SAME source runs on the robot
// and in the host tests under ASan/UBSan, where it is checked against the
// FIPS 180-2 and RFC 4231 test vectors (host/test_wcli.cpp) and against
// Python's hashlib/hmac through the generated wire vectors. mbedTLS would be
// on the ESP32 anyway, but then the host gate would be testing a different
// implementation from the one that is flashed.
//
// Nothing here is secret-dependent in its control flow or memory access
// pattern: SHA-256 is a fixed sequence of 32-bit adds, rotates and XORs.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace wcli {

constexpr size_t kShaLen = 32;
constexpr size_t kShaBlock = 64;

class Sha256 {
  public:
    Sha256() { reset(); }
    void reset();
    void update(const uint8_t* data, size_t n);
    void final(uint8_t out[kShaLen]);

  private:
    void block(const uint8_t* p);
    uint32_t h_[8] = {};
    uint8_t buf_[kShaBlock] = {};
    size_t fill_ = 0;
    uint64_t total_ = 0;   // bytes hashed so far
};

void sha256(const uint8_t* data, size_t n, uint8_t out[kShaLen]);

// HMAC-SHA256 over the concatenation of up to two buffers (header and
// payload are separate in the codec; no copy needed). b may be nullptr/0.
void hmacSha256(const uint8_t* key, size_t key_len, const uint8_t* a,
                size_t a_len, const uint8_t* b, size_t b_len,
                uint8_t out[kShaLen]);

// Compare n bytes in time independent of where they differ. The MAC check
// uses this, never memcmp: an early-exit compare leaks how many leading tag
// bytes a forgery got right.
bool ctEqual(const uint8_t* a, const uint8_t* b, size_t n);

// Overwrite a buffer in a way the optimiser may not elide (key material).
void wipe(void* p, size_t n);

}  // namespace wcli
