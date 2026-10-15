# The one entry point for everything generated in this repo: `make` lists the
# targets. As for compiled code, make decides what is stale from file times;
# a Python generator's prerequisites are the repo modules it imports
# (tools/mkdeps.py writes them to build/deps/).
#
# GNU Make 3.81 (macOS /usr/bin/make) or newer. Runs from the repo root.

SHELL := /bin/bash
.DEFAULT_GOAL := help
.DELETE_ON_ERROR:
.SUFFIXES:
# keep what a rule chain builds on the way (renders that figures copies, ...)
.SECONDARY:

##@ Variables
## V=v6        the design: v6 (the robot), v5 (the 10-joint prototype), bench-v2, bench-v3
## PART=name   one v6 part or slice job instead of all of them (cad, slice)
## RUN=name    a training run under sim/runs/ (sim, eval, deploy)
## ARGS="..."  more arguments for the script behind the target (sim, puppet, eval, train)
## JOBS=n      parallel jobs (default: every core)

V ?= v6
NCPU := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
# the slicer is multithreaded itself: half the cores, as cad/slice.py takes
ifneq ($(filter slice,$(MAKECMDGOALS)),)
JOBS ?= $(shell echo $$(( ($(NCPU) + 1) / 2 )))
endif
JOBS ?= $(NCPU)
MAKEFLAGS += -j$(JOBS)
ifneq ($(filter clean distclean,$(MAKECMDGOALS)),)
.NOTPARALLEL:
endif

# A worktree's .venv is a symlink to the main checkout's; without one, use the
# main checkout's directly.
PY ?= $(firstword $(abspath $(wildcard .venv/bin/python $(dir $(shell git rev-parse --path-format=absolute --git-common-dir 2>/dev/null)).venv/bin/python)) python3)

ifeq ($(shell uname -s),Darwin)
export MUJOCO_GL ?= cgl
else
export MUJOCO_GL ?= egl
endif

# Prerequisite lists, remade when any Python source changes. Not read for the
# targets that need none.
PY_SOURCES := $(wildcard $(shell git ls-files -co --exclude-standard -- '*.py'))
ifneq ($(filter-out help clean distclean,$(or $(MAKECMDGOALS),help)),)
-include build/deps/py.mk build/deps/v6.mk
endif

build/deps/py.mk: tools/mkdeps.py $(PY_SOURCES)
	@mkdir -p $(@D)
	$(PY) tools/mkdeps.py imports > $@

build/deps/v6.mk: tools/mkdeps.py $(filter cad/%,$(PY_SOURCES)) cad/v6/print_settings.json
	@mkdir -p $(@D)
	$(PY) tools/mkdeps.py v6 > $@

include mk/cad.mk mk/sim.mk mk/firmware.mk mk/test.mk mk/housekeeping.mk
