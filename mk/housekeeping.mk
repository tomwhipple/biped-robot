##@ Housekeeping
.PHONY: help all clean distclean

help: ## list the targets and variables
	@awk 'BEGIN {FS = ":.*## "} \
	  /^##@ / {printf "\n%s\n", substr($$0, 5); next} \
	  /^## / {printf "  %s\n", substr($$0, 4); next} \
	  /^[a-z][a-z0-9-]*:.*## / {printf "  make %-14s %s\n", $$1, $$2}' $(MAKEFILE_LIST)

all: cad plant host ## the v6 parts, the sim plant and the host build

# Never results: sim/runs, hw_sessions, captures, experiments' dated data.
# cad/gcode is the slicer's old output folder; harvest any project saved from
# the GUI there (python cad/slice.py --harvest) before deleting it by hand.
clean: ## delete everything generated that git does not track
	rm -rf build firmware/build firmware/host/build
	rm -rf cad/renders/_anim cad/renders/_dressed cad/renders/_jointrom cad/renders/_full
	rm -rf cad/step/poses experiments/plan-b-bench/_v3_step
	find . -name __pycache__ -type d -not -path './.venv/*' -not -path './.claude/*' -prune -exec rm -rf {} +

distclean: clean ## clean, plus the fetched C++ dependencies and the venv
	rm -rf third_party .venv
