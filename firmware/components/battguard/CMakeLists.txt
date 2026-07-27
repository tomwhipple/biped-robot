# Pure: no IDF dependency at all, which is what lets host/ compile it as-is.
idf_component_register(SRCS "guard.cpp" INCLUDE_DIRS "include")
