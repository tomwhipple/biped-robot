##@ CAD: the printed parts (V=v6 | v5 | bench-v2 | bench-v3)
.PHONY: cad cad-v6 cad-v5 cad-bench-v2 cad-bench-v3 freecad slice check-cad

# The robot to print; a variant's own parts are built with the variant's setting.
V6_ENV := ARMS=1 HIP_YOKE_VARIANT=single SERVO_PLAN=hips
V6_ALL_PARTS := $(V6_PARTS) $(V6_VARIANT_PARTS)
V6_ROLLUP := build/v6/reports/parts_v6_rollup.txt

# tools/mkdeps.py could not list the v6 parts (no build123d here?)
v6_parts_listed = $(if $(V6_DEPS_ERROR),$(error $(V6_DEPS_ERROR)))

cad: cad-$(V) ## export the parts: STLs (committed for now) and STEPs in build/<V>/step (PART= for one v6 part)

cad-v6: $(if $(PART),cad/v6/stl/$(PART).stl build/v6/step/$(PART).step,\
	$(V6_ALL_PARTS:%=cad/v6/stl/%.stl) $(V6_ALL_PARTS:%=build/v6/step/%.step) $(V6_ROLLUP))
	$(v6_parts_listed)

# parts_v6 prints a table row per part and still exits 0 for a part it could
# not build: pass this part's row through, and fail when it is missing or NOT BUILT.
part_row = awk -v p=$(1) '$$1 == p {found = 1; if (/NOT BUILT/) bad = 1; print} END {exit !(found && !bad)}'
build_part = set -o pipefail; $(V6_ENV) $(PART_ENV_$(1)) $(PY) cad/v6/parts_v6.py --only $(1) | $(call part_row,$(1))

# One run writes a part's STL and STEP: the STL's rule rebuilds both when a
# source changes, and the STEP's own rule only runs when the STEP is missing.
define v6_part
cad/v6/stl/$(1).stl: $$(PARTDEPS_$(1))
	@$$(call build_part,$(1))
build/v6/step/$(1).step: | cad/v6/stl/$(1).stl
	@$$(call build_part,$(1))
endef
$(foreach p,$(V6_ALL_PARTS),$(eval $(call v6_part,$(p))))

$(V6_ROLLUP): $(V6_ROLLUP_DEPS)
	$(V6_ENV) $(PY) cad/v6/parts_v6.py --no-export > /dev/null

# The prototype and the bench rigs: one run of each script writes all its
# files, so each is a pattern rule with the version as its stem (how make 3.81
# knows that). The prototype's STLs (cad/stl, committed for now) come with its STEPs.
V5_PARTS := pelvis yaw_carrier yoke_roll yoke_pitch leg_link foot gopro_base leg_link_print \
	yaw_carrier_print tower imu_carrier
V5_SCREWS := grip disc_y roll yaw_carrier deck gopro board tower head_stack foot
V5_SERVOS := yaw roll pitch ankle
BENCH := experiments/plan-b-bench
BENCH_V2 := stl/plan_b_bracket.stl stl/plan_b_lever_stiff.stl stl/plan_b_lever_hold.stl \
	stl/plan_b_lid.stl stl/plan_b_stop_pin.stl renders/plan_b_rig_render.png renders/plan_b_rig_side.svg
BENCH_V3 := stl/plan_b_v3_bracket.stl stl/plan_b_v3_fork.stl step/Bracket_v3.step \
	step/Fork_level.step step/Fork_hanging.step step/Weights_envelope_level.step

V5_STEPS := $(V5_PARTS:%=build/v5/step/%.step) $(V5_SCREWS:%=build/v5/step/screws_%.step) $(V5_SERVOS:%=build/v5/step/servo_%.step)

cad-v5: $(V5_STEPS)
cad-bench-v2: $(BENCH_V2:%=build/bench-v2/%)
cad-bench-v3: $(BENCH_V3:%=build/bench-v3/%)

$(V5_PARTS:%=build/\%/step/%.step): $(PYDEPS_cad/parts.py)
	$(PY) cad/parts.py > /dev/null
$(V5_SCREWS:%=build/\%/step/screws_%.step): $(PYDEPS_cad/fasteners.py)
	$(PY) cad/fasteners.py > /dev/null
$(V5_SERVOS:%=build/\%/step/servo_%.step): $(PYDEPS_cad/servos.py)
	$(PY) cad/servos.py > /dev/null
$(BENCH_V2:%=build/\%/%): $(PYDEPS_$(BENCH)/plan_b_rig.py)
	$(PY) $(BENCH)/plan_b_rig.py > /dev/null
$(BENCH_V3:%=build/\%/%): $(PYDEPS_$(BENCH)/plan_b_rig_v3.py)
	$(PY) $(BENCH)/plan_b_rig_v3.py > /dev/null

FREECADCMD ?= /Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd
freecad: build/v5/step/bimo_v3yaw_articulated.FCStd ## the prototype's articulated FreeCAD model (needs FreeCAD)
build/v5/step/bimo_v3yaw_articulated.FCStd: $(PYDEPS_cad/freecad_articulate.py) $(V5_STEPS)
	$(FREECADCMD) cad/freecad_articulate.py

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
	@mkdir -p build/v6/reports
	set -o pipefail; PY=$(PY) sh cad/run_checks.sh | tee build/v6/reports/cad_rom_check.txt

##@ Renders: build/<V>/renders and movies; `make figures` copies the ones docs show
.PHONY: assembly figures movies

V6_STEP_FIGS := $(wildcard docs/assembly/v6_*.png)
V6_GLUED := hip_yoke_glued ankle_link_glued
V6_DOC_FIGS := docs/img/assembly_v6_arms.png $(V6_GLUED:%=docs/img/%.png) \
	docs/assembly/assembly_v6_flyin_arms_strip.png docs/assembly/assembly_v6_flyin_strip.png $(V6_STEP_FIGS)

assembly: build/v6/step/assembly_v6_arms.step build/v6/step/assembly_v6.step ## the articulated v6 assembly, STEP + render, with and without arms
figures: $(V6_DOC_FIGS) ## re-render the v6 images the docs show and copy them into the docs (committed)
movies: build/v6/movies/assembly_v6_flyin_arms.mp4 build/v6/movies/assembly_v6_flyin.mp4 ## the fly-in assembly movies

build/%/step/assembly_v6_arms.step build/%/renders/assembly_v6_arms.png: $(PYDEPS_cad/v6/assembly_v6.py)
	$(V6_ENV) $(PY) cad/v6/assembly_v6.py
build/%/step/assembly_v6.step build/%/renders/assembly_v6.png: $(PYDEPS_cad/v6/assembly_v6.py)
	$(V6_ENV) ARMS=0 $(PY) cad/v6/assembly_v6.py
build/%/movies/assembly_v6_flyin_arms.mp4 build/%/movies/assembly_v6_flyin_arms_strip.png: $(PYDEPS_cad/v6/animate_v6.py)
	$(V6_ENV) $(PY) cad/v6/animate_v6.py
build/%/movies/assembly_v6_flyin.mp4 build/%/movies/assembly_v6_flyin_strip.png: $(PYDEPS_cad/v6/animate_v6.py)
	$(V6_ENV) ARMS=0 $(PY) cad/v6/animate_v6.py
ifneq ($(V6_STEP_FIGS),)
$(patsubst docs/assembly/%,build/\%/renders/%,$(V6_STEP_FIGS)): $(PYDEPS_cad/v6/render_steps_v6.py)
	$(PY) cad/v6/render_steps_v6.py
endif

docs/img/assembly_v6_arms.png: build/v6/renders/assembly_v6_arms.png
	cp $< $@
# the glued test parts: each module writes its STLs, STEPs and its strip
build/%/renders/hip_yoke_glued.png: $(PYDEPS_cad/v6/hip_yoke_glued.py)
	$(PY) cad/v6/hip_yoke_glued.py > /dev/null
build/%/renders/ankle_link_glued.png: $(PYDEPS_cad/v6/ankle_link_glued.py)
	$(PY) cad/v6/ankle_link_glued.py > /dev/null
$(V6_GLUED:%=docs/img/%.png): docs/img/%.png: build/v6/renders/%.png
	cp $< $@
docs/assembly/assembly_v6_flyin_arms_strip.png: build/v6/movies/assembly_v6_flyin_arms_strip.png
	cp $< $@
docs/assembly/assembly_v6_flyin_strip.png: build/v6/movies/assembly_v6_flyin_strip.png
	cp $< $@
docs/assembly/v6_%.png: build/v6/renders/v6_%.png
	cp $< $@
