# Pure: no IDF dependency at all, which is what lets host/ compile it as-is.
idf_component_register(SRCS "track.cpp" "lift_gate.cpp" INCLUDE_DIRS "include")
