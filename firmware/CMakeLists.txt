# ESP-IDF project for the Waveshare "Servo Driver with ESP32" board.
#
#   . $IDF_PATH/export.sh
#   idf.py set-target esp32
#   idf.py build
#
# NOTE: nothing here has been compiled -- ESP-IDF is not installed on the
# development laptop and flashing is a deliberately user-gated step (the board
# is on the bench running vendor firmware). The host test target
# (firmware/host/Makefile) is what is verified today.
cmake_minimum_required(VERSION 3.16)

# House rules from docs/firmware-design.md section 2: everything is warnings-
# as-errors, and C++ runs without exceptions or RTTI so the control loop has no
# hidden allocation or unwinding paths.
idf_build_set_property(COMPILE_OPTIONS "-Wall;-Wextra;-Werror" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS
                       "-std=gnu++17;-fno-exceptions;-fno-rtti" APPEND)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bimo_firmware)
