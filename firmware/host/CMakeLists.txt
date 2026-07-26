# Host build of the firmware's pure modules, for CI (`cmake -S . -B build &&
# cmake --build build && ctest --test-dir build --output-on-failure`).
#
# NOT VERIFIED on the development laptop: cmake is not installed there, so the
# Makefile beside this file is the target that is actually run and proven. Keep
# the two in step.
cmake_minimum_required(VERSION 3.16)
project(bimo_firmware_host CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMPONENTS ${CMAKE_CURRENT_SOURCE_DIR}/../components)

add_compile_options(-Wall -Wextra -Werror -Wshadow -Wconversion
                    -Wsign-conversion -g -O1
                    -fsanitize=address,undefined -fno-omit-frame-pointer)
add_link_options(-fsanitize=address,undefined)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${COMPONENTS}/scsbus/include
    ${COMPONENTS}/linkproto/include
    ${COMPONENTS}/obs/include
    ${COMPONENTS}/policy/include
    ${COMPONENTS}/imu/include)

enable_testing()

function(bimo_test name)
    add_executable(${name} ${ARGN})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

bimo_test(test_protocol test_protocol.cpp
          ${COMPONENTS}/linkproto/protocol.cpp
          ${COMPONENTS}/linkproto/watchdog.cpp
          ${COMPONENTS}/linkproto/framing.cpp)

bimo_test(test_scsbus test_scsbus.cpp
          ${COMPONENTS}/scsbus/packet.cpp
          ${COMPONENTS}/scsbus/bus.cpp)

bimo_test(test_obs test_obs.cpp
          ${COMPONENTS}/obs/assembler.cpp
          ${COMPONENTS}/obs/actuation.cpp
          ${COMPONENTS}/imu/imu.cpp)

bimo_test(test_policy test_policy.cpp ${COMPONENTS}/policy/mlp.cpp)
