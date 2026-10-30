# Pure: no IDF dependency at all, which is what lets host/ compile it as-is.
idf_component_register(SRCS "sha256.cpp" "wcli.cpp" "policy.cpp"
                       INCLUDE_DIRS "include")
