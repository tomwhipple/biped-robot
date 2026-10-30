##@ Firmware
.PHONY: host vectors deploy firmware lint cmake-check tidy tidy-gui cppcheck

PROTOCOL_VECTORS := firmware/host/vectors/protocol_vectors.h
FLIGHTREC_VECTORS := firmware/host/vectors/flightrec_vectors.h

host: $(PROTOCOL_VECTORS) $(FLIGHTREC_VECTORS) ## the firmware's pure modules under ASan/UBSan, the SIL library and the consoles (firmware/host)
	$(MAKE) -C firmware/host -j1 check

vectors: $(PROTOCOL_VECTORS) $(FLIGHTREC_VECTORS) ## the protocol and flight-recorder test vectors, from link/protocol.py and tools/flightrec.py
$(PROTOCOL_VECTORS): $(PYDEPS_tools/gen_protocol_vectors.py)
	$(PY) tools/gen_protocol_vectors.py
$(FLIGHTREC_VECTORS): $(PYDEPS_tools/gen_flightrec_vectors.py)
	$(PY) tools/gen_flightrec_vectors.py

# Only when somebody names the run: the headers re-point the firmware at a policy.
deploy: ## put RUN=name's policy into the firmware: weights, obs spec, vectors, SIL golden files
	@test -n "$(RUN)" || { echo "name the run: make deploy RUN=<run under sim/runs>" >&2; exit 2; }
	$(PY) tools/export_policy_weights.py --run $(RUN)
	$(MAKE) -C firmware/host -j1 deploy-headers RUN=$(RUN)

IDF_PATH ?= $(firstword $(wildcard $(HOME)/esp/esp-idf $(HOME)/.espressif/frameworks/esp-idf-v5.4))
firmware: ## the ESP32 image (ESP-IDF), in build/firmware
	@if ! command -v idf.py > /dev/null; then \
	  test -f "$(IDF_PATH)/export.sh" || { echo "no ESP-IDF: put idf.py on PATH or set IDF_PATH" >&2; exit 1; }; \
	  . "$(IDF_PATH)/export.sh" > /dev/null; \
	fi; \
	idf.py -C firmware -B build/firmware build

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

tidy: ## clang-tidy over the host firmware (.clang-tidy, WarningsAsErrors)
	@if ! command -v run-clang-tidy > /dev/null; then echo "SKIP: clang-tidy not installed" >&2; exit 0; fi; \
	if ! command -v cmake > /dev/null; then echo "SKIP: cmake not installed (needed for compile_commands.json)" >&2; exit 0; fi; \
	cmake -S firmware/host -B build/host-cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && \
	run-clang-tidy -p build/host-cmake -quiet

# clang-tidy-gui.sh reads firmware/host/build/compile_commands.json; a build
# folder configured here only for it is removed again.
tidy-gui: ## clang-tidy over the GUI and console sources
	@if ! command -v clang-tidy > /dev/null; then echo "SKIP: clang-tidy not installed" >&2; exit 0; fi; \
	created=; \
	if [ ! -f firmware/host/build/compile_commands.json ]; then \
	  if ! command -v cmake > /dev/null; then echo "SKIP: cmake not installed (needed for compile_commands.json)" >&2; exit 0; fi; \
	  cmake -S firmware/host -B firmware/host/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null || exit 1; \
	  created=1; \
	fi; \
	$(MAKE) -C firmware/host -j1 deps && firmware/host/clang-tidy-gui.sh; rc=$$?; \
	if [ -n "$$created" ]; then rm -rf firmware/host/build; fi; \
	exit $$rc

cppcheck: ## whole-program cppcheck over the firmware and link/, failing on any finding
	@if ! command -v cppcheck > /dev/null; then echo "SKIP: cppcheck not installed" >&2; exit 0; fi; \
	firmware/host/cppcheck.sh
