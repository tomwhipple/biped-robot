# ESP-IDF project for the Waveshare "General Driver for Robots" board (ESP32).
#
#   . $IDF_PATH/export.sh
#   idf.py set-target esp32
#   idf.py build
#
# Verified against ESP-IDF v5.4 on macos-arm64, 2026-07-26. Flashing stays a
# deliberately user-gated step (the board is on the bench). The host test
# target (firmware/host/Makefile) remains the hardware-free gate.
cmake_minimum_required(VERSION 3.16)

# project.cmake FIRST -- it is what defines idf_build_set_property. Calling
# the property setters above this line is a hard configure error ("Unknown
# CMake command"); they must sit between the include and project().
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# House rules from docs/firmware-design.md section 2: everything is warnings-
# as-errors, and C++ runs without exceptions or RTTI so the control loop has no
# hidden allocation or unwinding paths.
idf_build_set_property(COMPILE_OPTIONS "-Wall;-Wextra;-Werror" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS
                       "-std=gnu++17;-fno-exceptions;-fno-rtti" APPEND)

# Which compiled-in policy the image carries (tools/policy_profile.py).
# Empty: the DEPLOYED policy, whose generated headers live in the obs and
# policy components. A profile name (`idf.py -B build/firmware-v6legs
# -DBIMO_POLICY=v6legs build`, or `make firmware POLICY=v6legs`) selects the
# complete second set in policies/<name>/: the obs and policy components put
# it first on their include paths, and these definitions arm the generated
# headers' #error guards, so a wrong include order cannot build the wrong
# policy. It is a CMake cache variable -- it sticks to its build directory,
# so give each profile its own (-B).
set(BIMO_POLICY "" CACHE STRING
    "policy profile under firmware/policies/ (empty: the deployed policy)")
if(BIMO_POLICY)
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/policies/${BIMO_POLICY}/obs/obs_spec.h")
        message(FATAL_ERROR "BIMO_POLICY=${BIMO_POLICY}: no "
                "firmware/policies/${BIMO_POLICY}/obs/obs_spec.h")
    endif()
    string(TOUPPER "${BIMO_POLICY}" _bimo_policy_uc)
    idf_build_set_property(COMPILE_DEFINITIONS
        "BIMO_POLICY_OVERLAY;BIMO_POLICY_PROFILE_${_bimo_policy_uc}" APPEND)
    message(STATUS "bimo: policy profile ${BIMO_POLICY}")
endif()

project(bimo_firmware)

# The partition table is partitions.csv (3 MB app, NVS at its stock offset).
# sdkconfig is gitignored and sdkconfig.defaults only seeds a NEW one, so a
# checkout configured before the table changed would silently keep the old
# 1 MB app partition and fail later on size. Refuse here, with the fix.
if(NOT CONFIG_PARTITION_TABLE_CUSTOM OR
   NOT CONFIG_PARTITION_TABLE_CUSTOM_FILENAME STREQUAL "partitions.csv")
    message(FATAL_ERROR
        "firmware/sdkconfig predates the custom partition table "
        "(firmware/partitions.csv). Delete firmware/sdkconfig and build "
        "again: it is regenerated from sdkconfig.defaults. NVS is not moved, "
        "so no calibration is lost.")
endif()
