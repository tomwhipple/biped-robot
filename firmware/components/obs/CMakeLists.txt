# The generated headers are the selected policy set's (BIMO_POLICY,
# firmware/CMakeLists.txt): build/policy/<set>/, first on the include path.
if(BIMO_POLICY)
    set(_set "${BIMO_POLICY}")
else()
    set(_set "deployed")
endif()
set(_incs "${CMAKE_CURRENT_LIST_DIR}/../../../build/policy/${_set}" "include")
idf_component_register(SRCS "assembler.cpp" "actuation.cpp"
                       INCLUDE_DIRS ${_incs})
