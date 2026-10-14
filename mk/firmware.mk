##@ Firmware
.PHONY: host vectors deploy firmware lint

PROTOCOL_VECTORS := firmware/host/vectors/protocol_vectors.h

host: $(PROTOCOL_VECTORS) ## the firmware's pure modules under ASan/UBSan, the SIL library and the consoles (firmware/host)
	$(MAKE) -C firmware/host -j1 check

vectors: $(PROTOCOL_VECTORS) ## the protocol test vectors, from link/protocol.py
$(PROTOCOL_VECTORS): $(PYDEPS_tools/gen_protocol_vectors.py)
	$(PY) tools/gen_protocol_vectors.py

# Only when somebody names the run: the headers re-point the firmware at a policy.
deploy: ## put RUN=name's policy into the firmware: weights, obs spec, vectors, SIL golden files
	@test -n "$(RUN)" || { echo "name the run: make deploy RUN=<run under sim/runs>" >&2; exit 2; }
	$(PY) tools/export_policy_weights.py --run $(RUN)
	$(MAKE) -C firmware/host -j1 deploy-headers RUN=$(RUN)

IDF_PATH ?= $(HOME)/esp/esp-idf
firmware: ## the ESP32 image (ESP-IDF), in build/firmware
	command -v idf.py > /dev/null || . $(IDF_PATH)/export.sh > /dev/null; \
	  idf.py -C firmware -B build/firmware build

# The pre-push hook's static-analysis gates; a missing tool is a visible SKIP.
lint: ## cmake + ctest (CMakeLists agrees with the Makefile), clang-tidy, clang-tidy-gui, cppcheck
	@if command -v cmake > /dev/null; then \
	  cmake -S firmware/host -B build/host-cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && \
	  cmake --build build/host-cmake -j$(JOBS) && \
	  ctest --test-dir build/host-cmake --output-on-failure; \
	else echo "SKIP: cmake not installed" >&2; fi
	@if command -v run-clang-tidy > /dev/null && test -f build/host-cmake/compile_commands.json; then \
	  run-clang-tidy -p build/host-cmake -quiet; \
	else echo "SKIP: run-clang-tidy not installed, or no cmake build" >&2; fi
	@if command -v clang-tidy > /dev/null; then \
	  $(MAKE) -C firmware/host -j1 deps && \
	  cmake -S firmware/host -B firmware/host/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON > /dev/null && \
	  firmware/host/clang-tidy-gui.sh; \
	else echo "SKIP: clang-tidy not installed" >&2; fi
	@if command -v cppcheck > /dev/null; then firmware/host/cppcheck.sh; \
	else echo "SKIP: cppcheck not installed" >&2; fi
