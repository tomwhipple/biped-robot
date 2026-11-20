# Pure: no IDF dependency at all, which is what lets host/ compile it as-is.
# The generated linkproto/cmd_envelope.h is the selected policy set's
# (BIMO_POLICY, firmware/CMakeLists.txt): build/policy/<set>/.
if(BIMO_POLICY)
    set(_set "${BIMO_POLICY}")
else()
    set(_set "deployed")
endif()
set(_incs "${CMAKE_CURRENT_LIST_DIR}/../../../build/policy/${_set}" "include")
idf_component_register(SRCS "protocol.cpp" "watchdog.cpp" "framing.cpp"
                       INCLUDE_DIRS ${_incs})
