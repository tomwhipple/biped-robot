# A policy profile (BIMO_POLICY, firmware/CMakeLists.txt) puts its generated
# headers ahead of the deployed ones in include/.
set(_incs "include")
if(BIMO_POLICY)
    list(PREPEND _incs "${CMAKE_CURRENT_LIST_DIR}/../../policies/${BIMO_POLICY}")
endif()
# q8.cpp: the int8 forward pass of a quantized profile (policy/q8.h);
# q8_mac16.S: its MAC16 dot-product kernel (Xtensa only, #if-guarded).
idf_component_register(SRCS "mlp.cpp" "q8.cpp" "q8_mac16.S"
                       INCLUDE_DIRS ${_incs})
# One IEEE operation per float statement, as tools/policy_q8.py does them:
# GCC's default -ffp-contract=fast would fuse a*b+c into madd.s and round
# differently from the host build and the numpy mirror (q8.h).
set_source_files_properties(q8.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
