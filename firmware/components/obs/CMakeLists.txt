# A policy profile (BIMO_POLICY, firmware/CMakeLists.txt) puts its generated
# headers ahead of the deployed ones in include/.
set(_incs "include")
if(BIMO_POLICY)
    list(PREPEND _incs "${CMAKE_CURRENT_LIST_DIR}/../../policies/${BIMO_POLICY}")
endif()
idf_component_register(SRCS "assembler.cpp" "actuation.cpp"
                       INCLUDE_DIRS ${_incs})
