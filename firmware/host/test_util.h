// The world's smallest test harness.
//
// Deliberately dependency-free: the acceptance gate for this firmware is that
// a laptop with nothing but a C++ compiler can run the pure modules under
// -fsanitize=address,undefined. Vendoring gtest to get CHECK_NEAR would be a
// worse trade.
#pragma once
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace testutil {

inline int g_checks = 0;
inline int g_fails = 0;

inline void fail(const char* file, int line, const char* what) {
    ++g_fails;
    fprintf(stderr, "FAIL %s:%d: %s\n", file, line, what);
}

inline int report(const char* suite) {
    printf("%-16s %4d checks, %d failures\n", suite, g_checks, g_fails);
    return g_fails ? 1 : 0;
}

}  // namespace testutil

#define CHECK(cond)                                                       \
    do {                                                                  \
        ++testutil::g_checks;                                             \
        if (!(cond)) testutil::fail(__FILE__, __LINE__, #cond);           \
    } while (0)

#define CHECK_EQ(a, b)                                                    \
    do {                                                                  \
        ++testutil::g_checks;                                             \
        const long long a_ = (long long)(a), b_ = (long long)(b);         \
        if (a_ != b_) {                                                   \
            char msg_[256];                                               \
            snprintf(msg_, sizeof msg_, "%s == %s  (%lld vs %lld)", #a,   \
                     #b, a_, b_);                                         \
            testutil::fail(__FILE__, __LINE__, msg_);                     \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                             \
    do {                                                                  \
        ++testutil::g_checks;                                             \
        const double a_ = (double)(a), b_ = (double)(b);                  \
        if (!(fabs(a_ - b_) <= (tol))) {                                  \
            char msg_[256];                                               \
            snprintf(msg_, sizeof msg_, "%s ~= %s  (%.9g vs %.9g, d=%.3g)", \
                     #a, #b, a_, b_, a_ - b_);                            \
            testutil::fail(__FILE__, __LINE__, msg_);                     \
        }                                                                 \
    } while (0)

#define CHECK_BYTES(got, want, n)                                         \
    do {                                                                  \
        ++testutil::g_checks;                                             \
        if (memcmp((got), (want), (n)) != 0) {                            \
            fprintf(stderr, "FAIL %s:%d: bytes differ\n  got  ", __FILE__, \
                    __LINE__);                                            \
            for (size_t i_ = 0; i_ < (size_t)(n); ++i_)                   \
                fprintf(stderr, "%02X ", ((const uint8_t*)(got))[i_]);     \
            fprintf(stderr, "\n  want ");                                 \
            for (size_t i_ = 0; i_ < (size_t)(n); ++i_)                   \
                fprintf(stderr, "%02X ", ((const uint8_t*)(want))[i_]);    \
            fprintf(stderr, "\n");                                        \
            ++testutil::g_fails;                                          \
        }                                                                 \
    } while (0)
