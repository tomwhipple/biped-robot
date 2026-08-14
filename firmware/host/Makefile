# Host build of the firmware's pure modules -- the acceptance gate.
#
#   make -C firmware/host test
#
# No ESP-IDF, no hardware, no serial port. Everything compiled here is the
# same source the board runs; the hardware-touching files (main/, the IDF Port
# and I2C implementations) are excluded by construction, which is the point of
# keeping the seams where they are.
#
# There is a CMakeLists.txt beside this Makefile for CI/`ctest`; this Makefile
# exists because a stock macOS has clang++ but not cmake.

CXX      ?= c++
ROOT      = ..
COMPONENTS= $(ROOT)/components

CXXFLAGS  = -std=c++17 -O1 -g \
            -Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion \
            -fno-exceptions -fno-rtti \
            -I. -I$(COMPONENTS)/scsbus/include \
            -I$(COMPONENTS)/linkproto/include \
            -I$(COMPONENTS)/obs/include \
            -I$(COMPONENTS)/policy/include \
            -I$(COMPONENTS)/imu/include \
            -I$(COMPONENTS)/battguard/include

# Kept before the sanitizers are folded in: the SIL shared library cannot use
# them (see the `sil` target below).
CXXFLAGS_NOSAN := $(CXXFLAGS)

SAN      ?= -fsanitize=address,undefined -fno-omit-frame-pointer
CXXFLAGS += $(SAN)
LDFLAGS  += $(SAN)

BUILD = build

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  SIL_EXT    = dylib
  SIL_SHARED = -dynamiclib
else
  SIL_EXT    = so
  SIL_SHARED = -shared
endif

SRC_SCS   = $(COMPONENTS)/scsbus/packet.cpp $(COMPONENTS)/scsbus/bus.cpp
SRC_LINK  = $(COMPONENTS)/linkproto/protocol.cpp \
            $(COMPONENTS)/linkproto/watchdog.cpp \
            $(COMPONENTS)/linkproto/framing.cpp
SRC_OBS   = $(COMPONENTS)/obs/assembler.cpp $(COMPONENTS)/obs/actuation.cpp \
            $(COMPONENTS)/imu/imu.cpp
SRC_POL   = $(COMPONENTS)/policy/mlp.cpp
SRC_BATT  = $(COMPONENTS)/battguard/guard.cpp

SRC_FUSION= $(COMPONENTS)/imu/imu.cpp $(COMPONENTS)/imu/fusion.cpp

TESTS = test_protocol test_scsbus test_obs test_policy test_battguard \
        test_fusion test_sil test_sil_golden
BINS  = $(addprefix $(BUILD)/,$(TESTS))

# The software-in-the-loop library (docs/sil-harness.md): the same component
# sources the tests use, plus the C boundary python loads with ctypes.
SIL_LIB   = $(BUILD)/libctrl_sil.$(SIL_EXT)
SRC_SIL   = sil_lib.cpp sil_net.cpp
SRC_SILDEP= $(COMPONENTS)/obs/assembler.cpp $(COMPONENTS)/obs/actuation.cpp \
            $(COMPONENTS)/policy/mlp.cpp

.PHONY: all test clean vectors sil
all: $(BINS) $(SIL_LIB)

# The test sources use std::vector (FakePort's script), so exceptions are left
# on for the test translation units only -- the firmware sources themselves are
# still compiled -fno-exceptions above.
TESTFLAGS = $(filter-out -fno-exceptions,$(CXXFLAGS))

# Generated headers are prerequisites, not just includes: regenerating the
# golden vectors must relink the tests (this bit us once already).
SPEC_H    = $(COMPONENTS)/obs/include/obs/obs_spec.h
WEIGHTS_H = $(COMPONENTS)/policy/include/policy/weights.h
# ... and so is every component header (2026-08-02: editing a constant in
# obs/actuation.h left libctrl_sil.dylib stale and the gate green against the
# OLD value -- a false pass this line makes impossible).
COMP_HDRS = $(wildcard $(COMPONENTS)/*/include/*/*.h)

$(BUILD)/test_protocol: test_protocol.cpp $(SRC_LINK) \
                        vectors/protocol_vectors.h test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD)/test_scsbus: test_scsbus.cpp $(SRC_SCS) test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

# cal_store.cpp lives in main/ but its pack/unpack half is plain data and
# compiles off-target (the NVS half is behind #ifdef ESP_PLATFORM), so the
# calibration blob is covered by the hardware-free gate like everything else.
$(BUILD)/test_obs: test_obs.cpp $(SRC_OBS) ../main/cal_store.cpp \
                   ../main/cal_store.h ../main/asbuilt_cal.h $(SPEC_H) \
                   vectors/obs_vectors.h test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD)/test_policy: test_policy.cpp $(SRC_POL) $(SPEC_H) $(WEIGHTS_H) \
                      vectors/policy_vectors.h test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD)/test_battguard: test_battguard.cpp $(SRC_BATT) test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

# The fusion filter is the half of the IMU driver that has no hardware in it,
# which is exactly why it lives in components/imu/ next to the pure maths
# rather than beside the I2C code. Its attitude assertions are cross-checked
# against MuJoCo's framezaxis on the real plant -- see the header of
# test_fusion.cpp.
$(BUILD)/test_fusion: test_fusion.cpp $(SRC_FUSION) test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

# -- software in the loop ---------------------------------------------------
#
# The SIL sources are linked straight into the test binaries, so the ctests DO
# run under -fsanitize=address,undefined even though the shipped dylib below
# does not. That is the important half of the coverage: the JSON reader and the
# .silw parser are the only new code that touches untrusted bytes.
SIL_HDRS = sil.h sil_net.h sil_json.h

$(BUILD)/test_sil: test_sil.cpp $(SRC_SIL) $(SRC_SILDEP) $(SIL_HDRS) \
                   $(SPEC_H) $(WEIGHTS_H) test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD)/test_sil_golden: test_sil_golden.cpp sil_net.cpp $(SRC_SILDEP) \
                          $(SIL_HDRS) $(SPEC_H) $(WEIGHTS_H) test_util.h \
                          $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

# libctrl_sil is built WITHOUT the sanitizers, on purpose.
#
# ASan on macOS installs its interceptors from the main executable's runtime;
# a dylib built with -fsanitize=address that is dlopen()ed into a stock
# `python3` aborts with "interceptors not installed" unless the whole
# interpreter is launched under DYLD_INSERT_LIBRARIES=<asan runtime>, which is
# not a thing a pytest suite can reasonably require. UBSan alone would load,
# but splitting the two buys little. The sanitizer coverage is not lost: the
# same translation units are compiled with both in $(BUILD)/test_sil above,
# and that is where the parsers are exercised on adversarial input.
#
# -fno-exceptions is also dropped for the two SIL translation units: they use
# std::string/std::vector for file parsing and are host-only glue that is never
# flashed. The component sources they link stay exactly as the firmware builds
# them.
SILFLAGS = $(filter-out -fno-exceptions,$(CXXFLAGS_NOSAN)) -fPIC \
           -fvisibility=default

sil: $(SIL_LIB)

$(SIL_LIB): $(SRC_SIL) $(SRC_SILDEP) $(SIL_HDRS) $(SPEC_H) $(WEIGHTS_H) \
            $(COMP_HDRS) | $(BUILD)
	$(CXX) $(SILFLAGS) $(SIL_SHARED) $(filter %.cpp,$^) -o $@

$(BUILD):
	mkdir -p $(BUILD)

test: $(BINS) $(SIL_LIB)
	@rc=0; for t in $(BINS); do ./$$t || rc=1; done; \
	 if [ $$rc -eq 0 ]; then echo "ALL GREEN"; else echo "FAILURES"; fi; \
	 exit $$rc

# Regenerate every committed golden vector / generated header from the sim.
vectors:
	cd ../.. && .venv/bin/python tools/gen_protocol_vectors.py
	cd ../.. && .venv/bin/python tools/gen_obs_spec.py
	cd ../.. && .venv/bin/python tools/gen_policy_weights.py

clean:
	rm -rf $(BUILD)
