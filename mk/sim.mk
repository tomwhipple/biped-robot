##@ Sim: build its inputs, then run it
.PHONY: plant sim-data sim puppet eval train report

V6_PLANT := sim/bimo_biped_v6ar.xml
SERVO_MESHES := $(wildcard sim/meshes/v6/servo_*.stl)
GETUP_REFS := sim/getup_ref_v6ar.npz sim/getup_ref_v6ar_prone.npz sim/getup_ref_v6ar_roll.npz

plant: $(V6_PLANT) ## what the simulator loads: the dressed v6 plant and its CAD meshes
	$(v6_parts_listed)
sim-data: sim/terrain_mosaic.npz $(GETUP_REFS) sim/getup_catch_states.npz ## the terrain and get-up references the trainer loads (slow; changes what trains)

$(V6_PLANT): $(PYDEPS_sim/build_v6_inertia.py) $(V6_PARTS:%=cad/v6/stl/%.stl) $(SERVO_MESHES)
	set -o pipefail; $(PY) sim/build_v6_inertia.py --write | tail -1
ifneq ($(SERVO_MESHES),)
$(subst sim/meshes/v6/,sim/meshes/%/,$(SERVO_MESHES)): $(PYDEPS_sim/add_cad_meshes.py)
	$(PY) sim/add_cad_meshes.py --export-servos
endif

sim/terrain_mosaic.npz: $(PYDEPS_sim/mjx/gen_terrain_mosaic.py)
	cd sim/mjx && $(PY) gen_terrain_mosaic.py
sim/getup_ref_v6ar.npz: START = supine
sim/getup_ref_v6ar_prone.npz: START = prone
sim/getup_ref_v6ar_roll.npz: START = roll
$(GETUP_REFS): $(PYDEPS_sim/record_getup_ref.py) $(V6_PLANT)
	$(PY) sim/record_getup_ref.py $(abspath $@) $(START)
sim/getup_catch_states.npz: $(PYDEPS_sim/mjx/harvest_rise_states.py) sim/bimo_biped_v3yaw.xml $(wildcard cad/stl/*.stl)
	cd sim/mjx && $(PY) harvest_rise_states.py --out ../getup_catch_states.npz

sim: plant ## open the v6 plant in MuJoCo's viewer; RUN=name plays that policy through the firmware's control code
ifdef RUN
	$(PY) sim/sil_twin.py --viewer --run-name $(RUN) $(ARGS)
else
	$(PY) -m mujoco.viewer --mjcf=$(V6_PLANT) $(ARGS)
endif

puppet: plant ## pose the v6 plant by hand in the joint puppet (Qt)
	$(PY) sim/joint_puppet.py $(ARGS)

eval: ## the CPU referee's scorecard for RUN=name, written into sim/runs/RUN
	@test -n "$(RUN)" || { echo "name the run: make eval RUN=<run under sim/runs>" >&2; exit 2; }
	cd sim/mjx && JAX_PLATFORMS=cpu $(PY) eval_precision.py --run-name $(RUN) $(ARGS)

train: plant ## start a training run now: ARGS="<train_mjx.py arguments>"
	cd sim/mjx && $(PY) train_mjx.py $(ARGS)

report: build/v5/renders/assembly_mujoco.png ## refresh the results page, sim/runs/night_summary.html
	$(PY) sim/build_report.py

build/%/renders/assembly_mujoco.png: $(PYDEPS_cad/render_assembly.py) $(wildcard cad/stl/*.stl)
	$(PY) cad/render_assembly.py
