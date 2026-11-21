##@ Firmware
.PHONY: host console qtconsole vectors policy envelope deploy firmware firmware-powerlite lint cmake-check tidy tidy-gui cppcheck

# -- the generated policy sets (tools/policy_profile.py) ----------------------
# Git tracks each set's INPUTS in firmware/deployed/ (policies.txt: set, run,
# net format; runs/<run>/{config.json,params.pkl}; q8/<set>.npz for an int8
# set; headers.sha256: the bytes the deploy generated). `make policy`
# generates every set into build/policy/<set>/: the obs spec, weights and
# command envelope headers, their test vectors, the .silw and the SIL golden
# files, and checks the headers against headers.sha256. Two stages per set,
# each with a stamp: the envelope needs only the run's config.json
# (link/protocol.py reads it); the net needs the plant too (gen_obs_spec
# builds the env, the exporter rolls it).
POLICY_MANIFEST := firmware/deployed/policies.txt
POLICY_PINS := firmware/deployed/headers.sha256
POLICY_SETS := $(shell awk 'NF && substr($$1, 1, 1) != "\043" {print $$1}' $(POLICY_MANIFEST))
policy_field = $(shell awk '$$1 == "$(1)" {print $$$(2)}' $(POLICY_MANIFEST))
POLICY_PY := tools/policy_profile.py
POLICY_PLANTS = $(V6_PLANT) $(V6_LEGS_PLANT) $(V6_MEAS_PLANTS) $(V6_SOLE_PLANTS) $(V6_HANG_PLANT)

define policy_set
build/policy/$(1)/.envelope: $(POLICY_MANIFEST) $(POLICY_PINS) firmware/deployed/runs/$(2)/config.json $$(PYDEPS_tools/gen_envelope.py) $$(PYDEPS_$(POLICY_PY))
	$$(PY) $(POLICY_PY) gen envelope $(1)
	@touch $$@
build/policy/$(1)/.net: $(POLICY_MANIFEST) $(POLICY_PINS) firmware/deployed/runs/$(2)/config.json firmware/deployed/runs/$(2)/params.pkl $(if $(filter int8,$(3)),firmware/deployed/q8/$(1).npz) $$(PYDEPS_tools/gen_obs_spec.py) $$(PYDEPS_tools/export_policy_weights.py) $$(PYDEPS_tools/gen_policy_weights.py) $$(PYDEPS_$(POLICY_PY)) $$(POLICY_PLANTS)
	$$(PY) $(POLICY_PY) gen net $(1)
	@touch $$@
endef
$(foreach s,$(POLICY_SETS),$(eval $(call policy_set,$(s),$(call policy_field,$(s),2),$(call policy_field,$(s),3))))

POLICY_ENVELOPES := $(POLICY_SETS:%=build/policy/%/.envelope)
POLICY_DATA := $(POLICY_ENVELOPES) $(POLICY_SETS:%=build/policy/%/.net)
# one set: the deployed one, or POLICY=<profile>'s
POLICY_SET_DATA = $(addprefix build/policy/$(or $(POLICY),deployed)/,.envelope .net)

policy: $(POLICY_DATA) ## the policy sets of firmware/deployed/policies.txt, generated into build/policy/<set>/ (headers, vectors, .silw, SIL golden files)
envelope: $(POLICY_ENVELOPES) ## just the command envelopes (link/protocol.py's), which need no plant

PROTOCOL_VECTORS := build/host/vectors/protocol_vectors.h
FLIGHTREC_VECTORS := build/host/vectors/flightrec_vectors.h
WCLI_VECTORS := build/host/vectors/wcli_vectors.h
HOST_VECTORS := $(PROTOCOL_VECTORS) $(FLIGHTREC_VECTORS) $(WCLI_VECTORS)

host: $(HOST_VECTORS) $(POLICY_DATA) ## the firmware's pure modules under ASan/UBSan, the SIL library and the consoles (firmware/host)
	$(MAKE) -C firmware/host -j1 check

# ARGS replaces the default --host: a console given only --bench-telemetry or
# --cam-relay starts as an observer, and a --host would make it a commander.
console: $(HOST_VECTORS) build/policy/deployed/.envelope build/policy/deployed/.net ## build and open the GUI control panel (bimo_gui) on the local sim; ARGS="--host <robot-ip>" for the robot
	$(MAKE) -C firmware/host deps
	$(MAKE) -C firmware/host gui
	firmware/host/build/bimo_gui $(or $(ARGS),--host 127.0.0.1)

qtconsole: ## build the core library and open the Qt operator console (link/bimo_console.py, docs/console.md) observing the local sim; ARGS="--host <robot-ip>" for the robot
	$(MAKE) -C firmware/host console
	$(PY) link/bimo_console.py $(or $(ARGS),--host 127.0.0.1)

vectors: $(HOST_VECTORS) ## the protocol, flight-recorder and Wi-Fi CLI test vectors (build/host/vectors), from link/protocol.py, tools/flightrec.py and link/wcliproto.py
# (its watchdog cases clamp to the deployed command envelope)
$(PROTOCOL_VECTORS): $(PYDEPS_tools/gen_protocol_vectors.py) build/policy/deployed/.envelope
	$(PY) tools/gen_protocol_vectors.py
$(FLIGHTREC_VECTORS): $(PYDEPS_tools/gen_flightrec_vectors.py)
	$(PY) tools/gen_flightrec_vectors.py
$(WCLI_VECTORS): $(PYDEPS_tools/gen_wcli_vectors.py)
	$(PY) tools/gen_wcli_vectors.py

# Only when somebody names the run: this re-points the firmware at a policy.
# It copies the run's inputs into firmware/deployed/ (config.json and
# params.pkl; for an int8 set, the kernels quantized on today's plant),
# rewrites the set's line of policies.txt, generates the set and pins its
# headers' bytes in headers.sha256; then `make policy` rebuilds every set from
# the committed inputs against those pins. Commit firmware/deployed/.
deploy: ## put RUN=name's policy into the firmware: its inputs into firmware/deployed/, then make policy (POLICY=<profile>: that profile's set; QUANT=int8: an int8 profile)
	@test -n "$(RUN)" || { echo "name the run: make deploy RUN=<run under sim/runs>" >&2; exit 2; }
	$(MAKE) plant-data
	$(PY) $(POLICY_PY) deploy --run $(RUN) $(if $(POLICY),--profile $(POLICY)) $(if $(QUANT),--quant $(QUANT))
	$(MAKE) policy

IDF_PATH ?= $(firstword $(wildcard $(HOME)/esp/esp-idf $(HOME)/.espressif/frameworks/esp-idf-v5.4))
# ESP-IDF 5.4 supports Python 3.9-3.13; export.sh derives its env from whatever
# python3 is first on PATH, which on a Homebrew Mac moves with every Python
# release (3.15 has no env). Use an existing supported env when one is there.
export IDF_PYTHON_ENV_PATH ?= $(firstword $(wildcard $(addprefix $(HOME)/.espressif/python_env/idf5.4_py,3.12_env 3.13_env 3.11_env 3.10_env)))
# POLICY=<profile> builds the image with build/policy/<profile>'s policy
# instead of the deployed one (tools/policy_profile.py), in its own build dir.
# POWER_LITE=1 builds it for a robot with the power-lite board (main/power_rail.h);
# without that board such an image can never arm. Also its own build dir. Empty, 0,
# OFF and off all mean the default image.
POWER_LITE_ON = $(filter-out 0 OFF off,$(POWER_LITE))
FW_BUILD = build/firmware$(if $(POLICY),-$(POLICY))$(if $(POWER_LITE_ON),-powerlite)
firmware: $(POLICY_SET_DATA) ## the ESP32 image (ESP-IDF), in build/firmware (POLICY=<profile>: build/firmware-<profile>; POWER_LITE=1: ...-powerlite)
	@if ! command -v idf.py > /dev/null; then \
	  test -f "$(IDF_PATH)/export.sh" || { echo "no ESP-IDF: put idf.py on PATH or set IDF_PATH" >&2; exit 1; }; \
	  . "$(IDF_PATH)/export.sh" > /dev/null; \
	fi; \
	idf.py -C firmware -B $(FW_BUILD) -DBIMO_POLICY=$(POLICY) -DBIMO_POWER_LITE=$(if $(POWER_LITE_ON),ON,OFF) build

firmware-powerlite: ## the power-lite image, so the build option cannot rot (a pre-push gate)
	$(MAKE) firmware POWER_LITE=1

# Static analysis. A missing tool is a visible SKIP, never a silent pass.
lint: ## cmake-check, tidy, tidy-gui and cppcheck
	$(MAKE) cmake-check
	$(MAKE) tidy
	$(MAKE) tidy-gui
	$(MAKE) cppcheck

cmake-check: $(HOST_VECTORS) $(POLICY_DATA) ## firmware/host's CMakeLists.txt builds and tests the same as its Makefile
	@if ! command -v cmake > /dev/null; then echo "SKIP: cmake not installed" >&2; exit 0; fi; \
	cmake -S firmware/host -B build/host-cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
	cmake --build build/host-cmake -j$(JOBS) && \
	ctest --test-dir build/host-cmake --output-on-failure

# Homebrew's llvm is keg-only: its clang-tidy is not on PATH. Appended (never
# prepended) so the system compiler stays the one cmake picks. llvm@18 comes
# first: .clang-tidy enables whole check families by wildcard, and a newer
# clang-tidy adds checks to them (LLVM 23 reports ~600 findings the 18 series
# does not); 18 is what Ubuntu 24.04's clang-tidy is. Override: LLVM_BIN=<dir>.
LLVM_BIN ?= $(firstword $(wildcard /opt/homebrew/opt/llvm@18/bin /usr/local/opt/llvm@18/bin /opt/homebrew/opt/llvm/bin /usr/local/opt/llvm/bin))
TIDY_PATH = $(PATH)$(if $(LLVM_BIN),:$(LLVM_BIN))
# A keg-only llvm older than the SDK's libc++ cannot parse it (LLVM 18 vs the
# macOS 26 headers: __builtin_clzg); its own libc++ headers are used instead.
TIDY_LIBCXX = $(firstword $(wildcard $(LLVM_BIN)/../include/c++/v1))
TIDY_EXTRA = $(if $(TIDY_LIBCXX),-extra-arg=-nostdinc++ -extra-arg=-isystem$(TIDY_LIBCXX))

tidy: $(HOST_VECTORS) $(POLICY_DATA) ## clang-tidy over the host firmware (.clang-tidy, WarningsAsErrors)
	@PATH="$(TIDY_PATH)"; export PATH; if ! command -v run-clang-tidy > /dev/null; then echo "SKIP: clang-tidy not installed" >&2; exit 0; fi; \
	if ! command -v cmake > /dev/null; then echo "SKIP: cmake not installed (needed for compile_commands.json)" >&2; exit 0; fi; \
	cmake -S firmware/host -B build/host-cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && \
	run-clang-tidy -p build/host-cmake -quiet $(TIDY_EXTRA) '^(?!.*/third_party/)'

# clang-tidy-gui.sh reads firmware/host/build/compile_commands.json; a build
# folder configured here only for it is removed again.
tidy-gui: $(HOST_VECTORS) $(POLICY_DATA) ## clang-tidy over the GUI and console sources
	@PATH="$(TIDY_PATH)"; export PATH; if ! command -v clang-tidy > /dev/null; then echo "SKIP: clang-tidy not installed" >&2; exit 0; fi; \
	created=; \
	if [ ! -f firmware/host/build/compile_commands.json ]; then \
	  if ! command -v cmake > /dev/null; then echo "SKIP: cmake not installed (needed for compile_commands.json)" >&2; exit 0; fi; \
	  cmake -S firmware/host -B firmware/host/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null || exit 1; \
	  created=1; \
	fi; \
	$(MAKE) -C firmware/host -j1 deps && CLANG_TIDY_EXTRA="$(TIDY_EXTRA)" firmware/host/clang-tidy-gui.sh; rc=$$?; \
	if [ -n "$$created" ]; then rm -rf firmware/host/build; fi; \
	exit $$rc

cppcheck: $(HOST_VECTORS) $(POLICY_DATA) ## whole-program cppcheck over the firmware and link/, failing on any finding
	@if ! command -v cppcheck > /dev/null; then echo "SKIP: cppcheck not installed" >&2; exit 0; fi; \
	firmware/host/cppcheck.sh
