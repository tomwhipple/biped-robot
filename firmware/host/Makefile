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
        test_fusion test_obsdump test_sil test_sil_golden test_client
BINS  = $(addprefix $(BUILD)/,$(TESTS))

# The software-in-the-loop library (docs/sil-harness.md): the same component
# sources the tests use, plus the C boundary python loads with ctypes.
SIL_LIB   = $(BUILD)/libctrl_sil.$(SIL_EXT)
SRC_SIL   = sil_lib.cpp sil_net.cpp
SRC_SILDEP= $(COMPONENTS)/obs/assembler.cpp $(COMPONENTS)/obs/actuation.cpp \
            $(COMPONENTS)/policy/mlp.cpp

.PHONY: all test clean vectors deploy-headers sil tui gui deps check check-gui
all: $(BINS) $(SIL_LIB)

# The two operator clients share one core (link/client.cpp): the sockets, the
# 7-wide intent, the arm-sync rule and the short-vs-long frame decision. Only
# the front end differs -- ncurses in tui.cpp, Dear ImGui in gui.cpp -- so the
# two consoles cannot drift about what a keypress puts on the wire.
SRC_CLIENT = ../../link/client.cpp
CLIENT_HDRS= ../../link/client.h
GUI_HDRS   = $(CLIENT_HDRS) ../../link/keymap.h ../../link/recorder.h \
             ../../link/mjpeg.h ../../link/simproc.h

# The operator's console (link/tui.cpp): the firmware's own linkproto
# sources plus ncurses, no sanitizers (it is a tool, not a test), exceptions
# off like the firmware. `make -C firmware/host tui`.
TUI = $(BUILD)/bimo_tui
tui: $(TUI)
$(TUI): ../../link/tui.cpp $(SRC_CLIENT) $(SRC_LINK) $(CLIENT_HDRS) \
        $(COMP_HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS_NOSAN) $(filter %.cpp,$^) -o $@ -lncurses

# -- the window (link/gui.cpp) ---------------------------------------------
# Dear ImGui is not vendored: `make deps` fetches a PINNED tarball into a
# gitignored third_party/, the way sim/runs and the .silw weights are fetched
# or regenerated rather than committed. The repo stays first-party source.
THIRD     = ../../third_party
IMGUI     = $(THIRD)/imgui
IMGUI_VER = v1.92.9b
IMGUI_SHA = 21d8a0a565e85dce943e375db00812c2f3f0ab21f3f0f7964e364a63422d7f99
# stb_image decodes the sim's MJPEG frames, stb_image_write writes the
# console's own --screenshot. Pinned by commit, not by branch.
STB_REV   = 2c980bb59875b0d32144a71867fbdebb2f77cd20
STB_SHA   = 594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3
STBW_SHA  = cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05

# A recipe that fails must not leave its target behind. Without this, a failed
# sha256 leaves the BAD download on disk as an up-to-date target, and every
# later `make deps`/`make gui` compiles it into the operator's console without
# re-checking. --fail likewise, so an HTTP error page is never mistaken for a
# library. (Percy, PR #53.)
.DELETE_ON_ERROR:

# ... and the digests are re-checked on EVERY `make deps`, not just on the
# fetch. .DELETE_ON_ERROR only fires when a recipe RUNS; a file already on
# disk is simply "up to date", so a tree poisoned by an earlier failure (or by
# hand) would never be looked at again. Found by trying it.
deps: $(IMGUI)/imgui.cpp $(THIRD)/stb_image.h $(THIRD)/stb_image_write.h
	@echo "$(STB_SHA)  $(THIRD)/stb_image.h" | shasum -a 256 -c - >/dev/null \
	  || { echo "third_party/stb_image.h FAILS its pin -- delete it and re-run"; false; }
	@echo "$(STBW_SHA)  $(THIRD)/stb_image_write.h" | shasum -a 256 -c - >/dev/null \
	  || { echo "third_party/stb_image_write.h FAILS its pin -- delete it and re-run"; false; }
	@echo "deps: imgui $(IMGUI_VER) + stb $(STB_REV) verified against their pins"

$(IMGUI)/imgui.cpp:
	@mkdir -p $(THIRD)
	curl -sSL --fail -o $(THIRD)/imgui.tgz \
	  https://github.com/ocornut/imgui/archive/refs/tags/$(IMGUI_VER).tar.gz
	@echo "$(IMGUI_SHA)  $(THIRD)/imgui.tgz" | shasum -a 256 -c - || \
	  { rm -f $(THIRD)/imgui.tgz; false; }
	tar xzf $(THIRD)/imgui.tgz -C $(THIRD)
	rm -rf $(IMGUI) && mv $(THIRD)/imgui-$(IMGUI_VER:v%=%) $(IMGUI)
	rm -f $(THIRD)/imgui.tgz

$(THIRD)/stb_image.h:
	@mkdir -p $(THIRD)
	curl -sSL --fail -o $@ \
	  https://raw.githubusercontent.com/nothings/stb/$(STB_REV)/stb_image.h
	@echo "$(STB_SHA)  $@" | shasum -a 256 -c -

$(THIRD)/stb_image_write.h:
	@mkdir -p $(THIRD)
	curl -sSL --fail -o $@ \
	  https://raw.githubusercontent.com/nothings/stb/$(STB_REV)/stb_image_write.h
	@echo "$(STBW_SHA)  $@" | shasum -a 256 -c -

# GLFW + OpenGL, split by platform. macOS still ships OpenGL.framework
# (deprecated since 10.14, hence the silence flag); ImGui's GL3 backend only
# needs 3.2 core. On Linux, GLFW is a system package (libglfw3-dev) and the
# OpenGL symbols come from imgui's own loader (imgui_impl_opengl3_loader.h),
# reached through the gl-shim that stands in for <OpenGL/gl3.h> -- the same
# shim clang-tidy uses, now compiled into the real binary.
ifeq ($(UNAME_S),Darwin)
  BREW      = $(shell brew --prefix 2>/dev/null || echo /opt/homebrew)
  GUI_LIBS  = -L$(BREW)/lib -lglfw -lpthread -framework OpenGL \
              -framework Cocoa -framework IOKit -framework CoreVideo
  GUI_INC   = -I$(BREW)/include -I$(IMGUI) -I$(IMGUI)/backends -I$(THIRD)
  GUI_DEFS  = -DGL_SILENCE_DEPRECATION
else
  GUI_LIBS  = -lglfw -lGL -lpthread
  GUI_INC   = -Igl-shim -I$(IMGUI) -I$(IMGUI)/backends -I$(THIRD)
  GUI_DEFS  =
endif

# Third-party sources cannot survive -Werror -Wconversion -Wshadow, so they
# are compiled to objects of their own with warnings off. Ours are not.
IMGUI_SRC = $(IMGUI)/imgui.cpp $(IMGUI)/imgui_draw.cpp \
            $(IMGUI)/imgui_tables.cpp $(IMGUI)/imgui_widgets.cpp \
            $(IMGUI)/backends/imgui_impl_glfw.cpp \
            $(IMGUI)/backends/imgui_impl_opengl3.cpp
IMGUI_OBJ = $(patsubst $(IMGUI)/%.cpp,$(BUILD)/imgui/%.o,$(IMGUI_SRC)) \
            $(BUILD)/imgui/stb_impl.o

$(BUILD)/imgui/%.o: $(IMGUI)/%.cpp | $(IMGUI)/imgui.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O1 -w -fno-exceptions -fno-rtti $(GUI_INC) $(GUI_DEFS) \
	       -c $< -o $@

# link/stb_impl.cpp is the single instantiation of stb_image; it is ours, but
# it #includes a vendored single-header library, so it is built like one.
$(BUILD)/imgui/stb_impl.o: ../../link/stb_impl.cpp ../../link/jpeg.h \
                           $(THIRD)/stb_image.h $(THIRD)/stb_image_write.h
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O1 -w -fno-exceptions -fno-rtti $(GUI_INC) $(GUI_DEFS) \
	       -c $< -o $@

GUI = $(BUILD)/bimo_gui
gui: $(GUI)
$(GUI): ../../link/gui.cpp $(SRC_CLIENT) ../../link/keymap.cpp \
        ../../link/recorder.cpp ../../link/mjpeg.cpp ../../link/simproc.cpp \
        $(SRC_LINK) $(GUI_HDRS) ../../link/jpeg.h $(COMP_HDRS) \
        $(IMGUI_OBJ) | $(BUILD)
	$(CXX) $(CXXFLAGS_NOSAN) $(GUI_INC) $(GUI_DEFS) \
	       $(filter %.cpp,$^) $(IMGUI_OBJ) -o $@ $(GUI_LIBS)

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

# The console core: link/client.cpp's intent and frame-length rule, the
# keymap file format, and the MJPEG stream parser -- the parts of an
# operator's console that have no screen in them. Sanitizers ON here, unlike
# the tools that link the same sources.
$(BUILD)/test_client: test_client.cpp $(SRC_CLIENT) ../../link/keymap.cpp \
                      ../../link/mjpeg.cpp ../../link/recorder.cpp \
                      $(SRC_LINK) $(GUI_HDRS) \
                      test_util.h $(COMP_HDRS) | $(BUILD)
	$(CXX) $(TESTFLAGS) -I../../link $(filter %.cpp,$^) -o $@ $(LDFLAGS)

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

# The observation dump (main/obs_dump.cpp). Like cal_store it lives in main/
# but touches no ESP-IDF, precisely so the double buffer and the CSV writer
# are covered by the hardware-free gate rather than by a flash-and-squint.
$(BUILD)/test_obsdump: test_obsdump.cpp ../main/obs_dump.cpp \
                       ../main/obs_dump.h ../main/joint_pose.h $(SPEC_H) \
                       test_util.h \
                       $(COMP_HDRS) | $(BUILD)
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

# -- the fast local gate (single source of truth with CI) -------------------
#
# `make check` is what the .githooks/pre-push hook runs before a push, and it
# is the SAME set of gates CI's host-tests / link-tests / console jobs run.
# Do NOT fork the command list between the hook and CI: a pass here must be a
# pass there and vice versa (the "no drift" rule).
#
#   test + tui  -- HARD, always. These are the mjpeg.cpp gate that works on
#                  both macOS and Linux today (ASan/UBSan host gate + the
#                  ncurses console compile).
#   deps + gui  -- the windowed console. Tolerant on Linux ONLY until the
#                  Linux GUI port (PR #61) lands on main: before that, a Linux
#                  dev without libglfw3-dev cannot link bimo_gui, and blocking
#                  their push would be the opposite of the point. On macOS the
#                  GUI builds today, so a gui failure there IS a hard block.
#
# The tolerance is expressed as a separate `check-gui` phony that `check`
# invokes with `-` (ignore its exit status) on Linux only. After the port
# merges, fold `deps gui` back into `check` as hard prerequisites and delete
# `check-gui` -- that is a follow-up, not this change.
check: test tui
ifeq ($(UNAME_S),Darwin)
	@$(MAKE) --no-print-directory deps gui
else
	@echo "check: gui/deps tolerant on Linux until the GUI port lands (PR #61)"
	@-$(MAKE) --no-print-directory check-gui
endif

check-gui: deps gui


# Regenerate the golden PROTOCOL vectors from the Python reference. This is
# the only generator that has one right answer independent of any policy, so
# it is the only one safe to run without saying anything.
vectors:
	cd ../.. && .venv/bin/python tools/gen_protocol_vectors.py

# The other two generators write the DEPLOYED policy: obs_spec.h is the
# observation layout the flashed weights were trained against, and
# policy_weights.h is the net itself. They may only change together, and only
# because somebody said which run to deploy -- an argument-less run of either
# silently re-points the firmware at a different policy. Hence:
#
#   make -C firmware/host deploy-headers RUN=loco_v27tilt_b_s128r24
#
deploy-headers:
	@test -n "$(RUN)" || { echo "name the run: make deploy-headers RUN=<run>"; exit 2; }
	cd ../.. && .venv/bin/python tools/gen_obs_spec.py --run $(RUN)
	cd ../.. && .venv/bin/python tools/gen_policy_weights.py --run $(RUN)

clean:
	rm -rf $(BUILD)
