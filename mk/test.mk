##@ Tests: the pre-push hook runs these, one gate per target
.PHONY: test test-py sil-lib pytest sil-suite prepush

test: ## every test: the host checks, then the pytest suites
	$(MAKE) host
	$(MAKE) pytest
	$(MAKE) sil-suite

test-py: ## the Python suites, with the SIL library they load
	$(MAKE) sil-lib
	$(MAKE) pytest
	$(MAKE) sil-suite

sil-lib: ## firmware/host's libctrl_sil, which the SIL suite loads
	$(MAKE) -C firmware/host -j1 sil

pytest: $(V6_ROLLUP) ## tests/: sim physics parity, the link protocol, ROM, the referee, the design gates
	$(PY) -m pytest tests/ -q

sil-suite: ## sim/sil: the firmware's control code against the CPU plant (after sil-lib)
	$(PY) -m pytest sim/sil -q

prepush: ## every gate the pre-push hook can run, in its order
	$(MAKE) test
	$(MAKE) lint
	$(MAKE) firmware
