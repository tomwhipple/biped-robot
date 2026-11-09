##@ Firmware
.PHONY: host console vectors deploy firmware lint cmake-check tidy tidy-gui cppcheck

PROTOCOL_VECTORS := firmware/host/vectors/protocol_vectors.h
FLIGHTREC_VECTORS := firmware/host/vectors/flightrec_vectors.h
WCLI_VECTORS := firmware/host/vectors/wcli_vectors.h

host: $(PROTOCOL_VECTORS) $(FLIGHTREC_VECTORS) $(WCLI_VECTORS) ## the firmware's pure modules under ASan/UBSan, the SIL library and the consoles (firmware/host)
	$(MAKE) -C firmware/host -j1 check

# ARGS replaces the default --host: a console given only --bench-telemetry or
# --cam-relay starts as an observer, and a --host would make it a commander.
console: ## build and open the GUI control panel (bimo_gui) on the local sim; ARGS="--host <robot-ip>" for the robot
	$(MAKE) -C firmware/host deps
	$(MAKE) -C firmware/host gui
	firmware/host/build/bimo_gui $(or $(ARGS),--host 127.0.0.1)

vectors: $(PROTOCOL_VECTORS) $(FLIGHTREC_VECTORS) $(WCLI_VECTORS) ## the protocol, flight-recorder and Wi-Fi CLI test vectors, from link/protocol.py, tools/flightrec.py and link/wcliproto.py
$(PROTOCOL_VECTORS): $(PYDEPS_tools/gen_protocol_vectors.py)
	$(PY) tools/gen_protocol_vectors.py
$(FLIGHTREC_VECTORS): $(PYDEPS_tools/gen_flightrec_vectors.py)
	$(PY) tools/gen_flightrec_vectors.py
$(WCLI_VECTORS): $(PYDEPS_tools/gen_wcli_vectors.py)
	$(PY) tools/gen_wcli_vectors.py

# Only when somebody names the run: the headers re-point the firmware at a policy.
# The obs spec goes first: the exporter checks the run's widths against it and
# writes the golden obs vectors from its layout; the weights header reads the
# exported .silw.
deploy: ## put RUN=name's policy into the firmware: weights, obs spec, vectors, SIL golden files (POLICY=<profile>: into that profile's set; QUANT=int8: an int8 profile)
	@test -n "$(RUN)" || { echo "name the run: make deploy RUN=<run under sim/runs>" >&2; exit 2; }
	$(PY) tools/gen_obs_spec.py --run $(RUN) $(if $(POLICY),--profile $(POLICY))
	$(PY) tools/export_policy_weights.py --run $(RUN) $(if $(POLICY),--profile $(POLICY))
	$(PY) tools/gen_policy_weights.py --run $(RUN) $(if $(POLICY),--profile $(POLICY)) $(if $(QUANT),--quant $(QUANT))

IDF_PATH ?= $(firstword $(wildcard $(HOME)/esp/esp-idf $(HOME)/.espressif/frameworks/esp-idf-v5.4))
# ESP-IDF 5.4 supports Python 3.9-3.13; export.sh derives its env from whatever
# python3 is first on PATH, which on a Homebrew Mac moves with every Python
# release (3.15 has no env). Use an existing supported env when one is there.
export IDF_PYTHON_ENV_PATH ?= $(firstword $(wildcard $(addprefix $(HOME)/.espressif/python_env/idf5.4_py,3.12_env 3.13_env 3.11_env 3.10_env)))
# POLICY=<profile> builds the image with firmware/policies/<profile>'s policy
# instead of the deployed one (tools/policy_profile.py), in its own build dir.
FW_BUILD = build/firmware$(if $(POLICY),-$(POLICY))
firmware: ## the ESP32 image (ESP-IDF), in build/firmware (POLICY=<profile>: build/firmware-<profile>)
	@if ! command -v idf.py > /dev/null; then \
	  test -f "$(IDF_PATH)/export.sh" || { echo "no ESP-IDF: put idf.py on PATH or set IDF_PATH" >&2; exit 1; }; \
	  . "$(IDF_PATH)/export.sh" > /dev/null; \
	fi; \
	idf.py -C firmware -B $(FW_BUILD) -DBIMO_POLICY=$(POLICY) build

# Static analysis. A missing tool is a visible SKIP, never a silent pass.
lint: ## cmake-check, tidy, tidy-gui and cppcheck
	$(MAKE) cmake-check
	$(MAKE) tidy
	$(MAKE) tidy-gui
	$(MAKE) cppcheck

cmake-check: ## firmware/host's CMakeLists.txt builds and tests the same as its Makefile
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

tidy: ## clang-tidy over the host firmware (.clang-tidy, WarningsAsErrors)
	@PATH="$(TIDY_PATH)"; export PATH; if ! command -v run-clang-tidy > /dev/null; then echo "SKIP: clang-tidy not installed" >&2; exit 0; fi; \
	if ! command -v cmake > /dev/null; then echo "SKIP: cmake not installed (needed for compile_commands.json)" >&2; exit 0; fi; \
	cmake -S firmware/host -B build/host-cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && \
	run-clang-tidy -p build/host-cmake -quiet $(TIDY_EXTRA) '^(?!.*/third_party/)'

# clang-tidy-gui.sh reads firmware/host/build/compile_commands.json; a build
# folder configured here only for it is removed again.
tidy-gui: ## clang-tidy over the GUI and console sources
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

cppcheck: ## whole-program cppcheck over the firmware and link/, failing on any finding
	@if ! command -v cppcheck > /dev/null; then echo "SKIP: cppcheck not installed" >&2; exit 0; fi; \
	firmware/host/cppcheck.sh
