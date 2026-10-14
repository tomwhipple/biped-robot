##@ Tests
.PHONY: test test-py prepush

test: ## every test: the host checks, then the pytest suites (tests/, the SIL suite)
	$(MAKE) host
	$(PY) -m pytest tests/ -q
	$(PY) -m pytest sim/sil -q

test-py: ## the Python suites only, with the SIL library they load
	$(MAKE) -C firmware/host -j1 sil
	$(PY) -m pytest tests/ -q
	$(PY) -m pytest sim/sil -q

prepush: ## every gate the pre-push hook can run, in its order
	$(MAKE) test
	$(MAKE) lint
	$(MAKE) firmware
