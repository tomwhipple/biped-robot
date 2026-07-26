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
            -I$(COMPONENTS)/imu/include

SAN      ?= -fsanitize=address,undefined -fno-omit-frame-pointer
CXXFLAGS += $(SAN)
LDFLAGS  += $(SAN)

BUILD = build

SRC_SCS   = $(COMPONENTS)/scsbus/packet.cpp $(COMPONENTS)/scsbus/bus.cpp
SRC_LINK  = $(COMPONENTS)/linkproto/protocol.cpp \
            $(COMPONENTS)/linkproto/watchdog.cpp \
            $(COMPONENTS)/linkproto/framing.cpp
SRC_OBS   = $(COMPONENTS)/obs/assembler.cpp $(COMPONENTS)/obs/actuation.cpp \
            $(COMPONENTS)/imu/imu.cpp
SRC_POL   = $(COMPONENTS)/policy/mlp.cpp

TESTS = test_protocol test_scsbus test_obs test_policy
BINS  = $(addprefix $(BUILD)/,$(TESTS))

.PHONY: all test clean vectors
all: $(BINS)

# The test sources use std::vector (FakePort's script), so exceptions are left
# on for the test translation units only -- the firmware sources themselves are
# still compiled -fno-exceptions above.
TESTFLAGS = $(filter-out -fno-exceptions,$(CXXFLAGS))

# Generated headers are prerequisites, not just includes: regenerating the
# golden vectors must relink the tests (this bit us once already).
SPEC_H    = $(COMPONENTS)/obs/include/obs/obs_spec.h
WEIGHTS_H = $(COMPONENTS)/policy/include/policy/weights.h

$(BUILD)/test_protocol: test_protocol.cpp $(SRC_LINK) \
                        vectors/protocol_vectors.h test_util.h | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD)/test_scsbus: test_scsbus.cpp $(SRC_SCS) test_util.h | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

# cal_store.cpp lives in main/ but its pack/unpack half is plain data and
# compiles off-target (the NVS half is behind #ifdef ESP_PLATFORM), so the
# calibration blob is covered by the hardware-free gate like everything else.
$(BUILD)/test_obs: test_obs.cpp $(SRC_OBS) ../main/cal_store.cpp $(SPEC_H) \
                   vectors/obs_vectors.h test_util.h | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD)/test_policy: test_policy.cpp $(SRC_POL) $(SPEC_H) $(WEIGHTS_H) \
                      vectors/policy_vectors.h test_util.h | $(BUILD)
	$(CXX) $(TESTFLAGS) $(filter %.cpp,$^) -o $@ $(LDFLAGS)

$(BUILD):
	mkdir -p $(BUILD)

test: $(BINS)
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
