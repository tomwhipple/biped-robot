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

project(bimo_firmware)
