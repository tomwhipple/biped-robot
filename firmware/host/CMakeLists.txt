# Host build of the firmware's pure modules, for CI (`cmake -S . -B build &&
# cmake --build build && ctest --test-dir build --output-on-failure`).
#
# The Makefile beside this file is still the target that is run day to day
# (a stock macOS has clang++ but not cmake); keep the two in step. Both were
# verified on 2026-07-30.
cmake_minimum_required(VERSION 3.16)
project(bimo_firmware_host CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMPONENTS ${CMAKE_CURRENT_SOURCE_DIR}/../components)

add_compile_options(-Wall -Wextra -Werror -Wshadow -Wconversion
                    -Wsign-conversion -g -O1)

# The sanitizers are attached per target rather than globally: the SIL shared
# library must NOT have them (see the ctrl_sil target at the bottom).
set(SAN_COMPILE -fsanitize=address,undefined -fno-omit-frame-pointer)
set(SAN_LINK -fsanitize=address,undefined)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${COMPONENTS}/scsbus/include
    ${COMPONENTS}/linkproto/include
    ${COMPONENTS}/obs/include
    ${COMPONENTS}/policy/include
    ${COMPONENTS}/imu/include
    ${COMPONENTS}/battguard/include)

enable_testing()

function(bimo_test name)
    add_executable(${name} ${ARGN})
    target_compile_options(${name} PRIVATE ${SAN_COMPILE})
    target_link_options(${name} PRIVATE ${SAN_LINK})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

bimo_test(test_protocol test_protocol.cpp
          ${COMPONENTS}/linkproto/protocol.cpp
          ${COMPONENTS}/linkproto/watchdog.cpp
          ${COMPONENTS}/linkproto/framing.cpp)

bimo_test(test_scsbus test_scsbus.cpp
          ${COMPONENTS}/scsbus/packet.cpp
          ${COMPONENTS}/scsbus/bus.cpp)

# cal_store.cpp lives in main/ but its pack/unpack half is plain data (the NVS
# half is behind #ifdef ESP_PLATFORM), and test_obs.cpp exercises it. The
# Makefile has always linked it; this file did not, so the cmake build did not
# link at all until 2026-07-30.
bimo_test(test_obs test_obs.cpp
          ${COMPONENTS}/obs/assembler.cpp
          ${COMPONENTS}/obs/actuation.cpp
          ${COMPONENTS}/imu/imu.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/../main/cal_store.cpp)

bimo_test(test_policy test_policy.cpp ${COMPONENTS}/policy/mlp.cpp)

bimo_test(test_battguard test_battguard.cpp ${COMPONENTS}/battguard/guard.cpp)

# -- software in the loop (docs/sil-harness.md) -----------------------------
set(SIL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/sil_lib.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/sil_net.cpp)
set(SIL_DEPS ${COMPONENTS}/obs/assembler.cpp
             ${COMPONENTS}/obs/actuation.cpp
             ${COMPONENTS}/policy/mlp.cpp)

bimo_test(test_sil test_sil.cpp ${SIL_SRC} ${SIL_DEPS})
bimo_test(test_sil_golden test_sil_golden.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/sil_net.cpp ${SIL_DEPS})

# ctest runs from the build directory; test_sil_golden walks upwards looking
# for sim/sil/golden, and test_sil writes its fixtures into the cwd.
set_tests_properties(test_sil test_sil_golden PROPERTIES
                     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# The dylib/so python loads with ctypes. NO SANITIZERS, deliberately: an
# ASan-instrumented shared library dlopen()ed into a stock python aborts with
# "interceptors not installed" unless the whole interpreter is launched under
# DYLD_INSERT_LIBRARIES, which a pytest suite cannot reasonably require. The
# same translation units are compiled WITH both sanitizers into test_sil
# above, so the parsers are still covered where it matters.
add_library(ctrl_sil SHARED ${SIL_SRC} ${SIL_DEPS})
set_target_properties(ctrl_sil PROPERTIES
                      POSITION_INDEPENDENT_CODE ON
                      OUTPUT_NAME ctrl_sil)
add_custom_target(sil DEPENDS ctrl_sil)

# The operator's console (link/tui.cpp) -- the firmware's own linkproto plus
# ncurses; a tool, so no sanitizers. Optional: skipped where curses is absent.
find_package(Curses QUIET)
if(CURSES_FOUND)
    add_executable(bimo_tui ${CMAKE_CURRENT_SOURCE_DIR}/../../link/tui.cpp
                   ${COMPONENTS}/linkproto/protocol.cpp
                   ${COMPONENTS}/linkproto/watchdog.cpp
                   ${COMPONENTS}/linkproto/framing.cpp)
    target_include_directories(bimo_tui PRIVATE ${CURSES_INCLUDE_DIRS})
    target_link_libraries(bimo_tui PRIVATE ${CURSES_LIBRARIES})
    add_custom_target(tui DEPENDS bimo_tui)
endif()
