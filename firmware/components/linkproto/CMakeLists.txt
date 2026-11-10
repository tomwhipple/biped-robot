# Pure: no IDF dependency at all, which is what lets host/ compile it as-is.
# A policy profile (BIMO_POLICY, firmware/CMakeLists.txt) puts its generated
# linkproto/cmd_envelope.h ahead of the deployed one in include/.
set(_incs "include")
if(BIMO_POLICY)
    list(PREPEND _incs "${CMAKE_CURRENT_LIST_DIR}/../../policies/${BIMO_POLICY}")
endif()
idf_component_register(SRCS "protocol.cpp" "watchdog.cpp" "framing.cpp"
                       INCLUDE_DIRS ${_incs})
