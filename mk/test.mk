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

sil-lib: ## firmware/host's libctrl_sil, which the SIL suite loads (the deployed policy's, and each policy profile's)
	$(MAKE) -C firmware/host -j1 sil
	@for p in $(notdir $(patsubst %/obs/obs_spec.h,%,$(wildcard firmware/policies/*/obs/obs_spec.h))); do \
	  $(MAKE) -C firmware/host -j1 sil POLICY=$$p || exit 1; \
	done

# pytest-xdist workers: a capped count, never `-n auto` (the gates run niced
# beside other work). PYTEST_WORKERS=0 runs one process, as plain pytest does.
# --dist loadgroup keeps each @pytest.mark.xdist_group on one worker.
PYTEST_WORKERS ?= 8
PYTEST_PAR = $(if $(filter 0,$(PYTEST_WORKERS)),-p no:xdist,-n $(PYTEST_WORKERS) --dist loadgroup)

pytest: $(V6_ROLLUP) ## tests/ (sim physics parity, the link protocol, ROM, the referee, the design gates, the power-lite simulation); PYTEST_WORKERS=<n> sets the xdist workers (default 8, 0 = one process)
	$(PY) -m pytest tests/ -q $(PYTEST_PAR)

sil-suite: ## sim/sil: the firmware's control code against the CPU plant (after sil-lib)
	$(PY) -m pytest sim/sil -q

prepush: ## every gate the pre-push hook can run, in its order
	$(MAKE) test
	$(MAKE) lint
	$(MAKE) firmware
