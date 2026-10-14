##@ CAD: the printed parts (V=v6 | v5 | bench-v2 | bench-v3)
.PHONY: cad cad-v6 cad-v5 cad-bench-v2 cad-bench-v3 slice assembly figures movies check-cad

# The robot to print; a variant's own parts are built with the variant's setting.
V6_ENV := ARMS=1 HIP_YOKE_VARIANT=single SERVO_PLAN=hips
V6_ALL_PARTS := $(V6_PARTS) $(V6_VARIANT_PARTS)

# tools/mkdeps.py could not list the v6 parts (no build123d here?)
v6_parts_listed = $(if $(V6_DEPS_ERROR),$(error $(V6_DEPS_ERROR)))

cad: cad-$(V) ## export the parts, STL + STEP (PART= for one v6 part)

cad-v6: $(if $(PART),cad/v6/stl/$(PART).stl,$(V6_ALL_PARTS:%=cad/v6/stl/%.stl) docs/design-v6/parts_v6_rollup.txt)
	$(v6_parts_listed)

# parts_v6 prints a table row per part and still exits 0 for a part it could
# not build: pass this part's row through, and fail when it is missing or NOT BUILT.
part_row = awk -v p=$(1) '$$1 == p {found = 1; if (/NOT BUILT/) bad = 1; print} END {exit !(found && !bad)}'

# One rule per part, with its folder as the stem: a pattern rule is how make
# 3.81 knows one run writes both files, and naming the part keeps the rule off
# the assembly's STEP.
define v6_part
cad/%/stl/$(1).stl cad/%/step/$(1).step: $$(PARTDEPS_$(1))
	@set -o pipefail; $$(V6_ENV) $$(PART_ENV_$(1)) $$(PY) cad/v6/parts_v6.py --only $(1) | $$(call part_row,$(1))
endef
$(foreach p,$(V6_ALL_PARTS),$(eval $(call v6_part,$(p))))

docs/design-v6/parts_v6_rollup.txt: $(V6_ROLLUP_DEPS)
	$(V6_ENV) $(PY) cad/v6/parts_v6.py --no-export > /dev/null

# The prototype and the bench rigs: one run of each script writes all its files.
V5_PARTS := pelvis yaw_carrier yoke_roll yoke_pitch leg_link foot gopro_base leg_link_print yaw_carrier_print
V5_SCREWS := grip disc_y roll yaw_carrier deck gopro board tower head_stack foot
V5_SERVOS := yaw roll pitch ankle
BENCH := experiments/plan-b-bench
BENCH_V2 := stl/plan_b_bracket.stl stl/plan_b_lever_stiff.stl stl/plan_b_lever_hold.stl \
	stl/plan_b_lid.stl stl/plan_b_stop_pin.stl figs/plan_b_rig_render.png figs/plan_b_rig_side.svg
BENCH_V3 := stl/plan_b_v3_bracket.stl stl/plan_b_v3_fork.stl

cad-v5: $(V5_PARTS:%=cad/stl/%.stl) $(V5_SCREWS:%=cad/step/screws_%.step) $(V5_SERVOS:%=cad/step/servo_%.step)
cad-bench-v2: $(BENCH_V2:%=$(BENCH)/%)
cad-bench-v3: $(BENCH_V3:%=$(BENCH)/%)

$(foreach p,$(V5_PARTS),%/stl/$(p).stl %/step/$(p).step): $(PYDEPS_cad/parts.py)
	$(PY) cad/parts.py > /dev/null
$(V5_SCREWS:%=\%/step/screws_%.step): $(PYDEPS_cad/fasteners.py)
	$(PY) cad/fasteners.py > /dev/null
$(V5_SERVOS:%=\%/step/servo_%.step): $(PYDEPS_cad/servos.py)
	$(PY) cad/servos.py > /dev/null
$(BENCH_V2:%=experiments/\%/%): $(PYDEPS_$(BENCH)/plan_b_rig.py)
	$(PY) $(BENCH)/plan_b_rig.py > /dev/null
$(BENCH_V3:%=experiments/\%/%): $(PYDEPS_$(BENCH)/plan_b_rig_v3.py)
	$(PY) $(BENCH)/plan_b_rig_v3.py > /dev/null

# The slicer is multithreaded itself; the Makefile runs half as many jobs for it.
SLICE_OUT := build/v6/gcode
slice: $(if $(PART),$(SLICE_OUT)/$(PART).gcode,$(V6_SLICE_JOBS:%=$(SLICE_OUT)/%.gcode)) ## OrcaSlicer projects + gcode for the v6 parts, in build/v6/gcode (PART=)
	$(v6_parts_listed)

define slice_job
build/%/gcode/$(1).gcode build/%/gcode/$(1).3mf: $$(JOBDEPS_$(1))
	$$(PY) cad/slice.py $(1) $$(SLICE_FLAGS_$(1)) --outdir $(abspath $(SLICE_OUT)) --jobs 1
endef
$(foreach j,$(V6_SLICE_JOBS),$(eval $(call slice_job,$(j))))

check-cad: cad-v6 ## the CAD gate: swept-ROM interference, armless and with arms, and the printability reports
	PY=$(PY) sh cad/run_checks.sh

##@ Renders: the current design's doc images (committed) and movies (gitignored)
V6_STEP_FIGS := $(wildcard docs/assembly/v6_*.png)
V6_DOC_RENDERS := cad/v6/renders/assembly_v6_arms.png \
	cad/v6/renders/assembly_v6_flyin_arms_strip.png cad/v6/renders/assembly_v6_flyin_strip.png

assembly: cad/v6/step/assembly_v6_arms.step cad/v6/step/assembly_v6.step ## the articulated v6 assembly, STEP + render, with and without arms
figures: $(V6_STEP_FIGS) $(V6_DOC_RENDERS) ## re-render the assembly guide's figures and the v6 renders the docs show, in place
movies: cad/v6/renders/assembly_v6_flyin_arms.mp4 cad/v6/renders/assembly_v6_flyin.mp4 ## the fly-in assembly movies

cad/%/step/assembly_v6_arms.step cad/%/renders/assembly_v6_arms.png: $(PYDEPS_cad/v6/assembly_v6.py)
	$(V6_ENV) $(PY) cad/v6/assembly_v6.py
cad/%/step/assembly_v6.step cad/%/renders/assembly_v6.png: $(PYDEPS_cad/v6/assembly_v6.py)
	$(V6_ENV) ARMS=0 $(PY) cad/v6/assembly_v6.py
cad/%/renders/assembly_v6_flyin_arms.mp4 cad/%/renders/assembly_v6_flyin_arms_strip.png: $(PYDEPS_cad/v6/animate_v6.py)
	$(V6_ENV) $(PY) cad/v6/animate_v6.py
cad/%/renders/assembly_v6_flyin.mp4 cad/%/renders/assembly_v6_flyin_strip.png: $(PYDEPS_cad/v6/animate_v6.py)
	$(V6_ENV) ARMS=0 $(PY) cad/v6/animate_v6.py
ifneq ($(V6_STEP_FIGS),)
$(subst docs/assembly/,docs/%/,$(V6_STEP_FIGS)): $(PYDEPS_cad/v6/render_steps_v6.py)
	$(PY) cad/v6/render_steps_v6.py
endif
