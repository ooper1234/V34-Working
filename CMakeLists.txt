cmake_minimum_required(VERSION 3.10)
project(softmodem C)

set(CMAKE_C_STANDARD 11)

# Compiler options
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-O2)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/logging
    ${CMAKE_SOURCE_DIR}/src/audio
    ${CMAKE_SOURCE_DIR}/src/serial
    ${CMAKE_SOURCE_DIR}/src/ppp
    ${CMAKE_SOURCE_DIR}/src/dsp
    ${CMAKE_SOURCE_DIR}/src/dsp/carrier
    ${CMAKE_SOURCE_DIR}/src/dsp/detectors
    ${CMAKE_SOURCE_DIR}/src/modem
    ${CMAKE_SOURCE_DIR}/src/modem/v22bis
)

# Math library
set(MATH_LIB m)

# Core library: modem + infra
add_library(softmodem STATIC
    src/modem/v22bis/v22bis_tx.c
    src/modem/v22bis/v22bis_rx.c
    src/logging/sm_log.c
    src/serial/sm_async.c
    src/ast_socket/sm_ast_socket.c
    src/ppp/sm_pppd.c
    src/call/sm_call.c
)
target_link_libraries(softmodem ${MATH_LIB})

# The answerer daemon
add_executable(sm_daemon src/main/sm_daemon.c)
target_link_libraries(sm_daemon softmodem ${MATH_LIB})

# Test helper: our implementation adapter (no spandsp required)
add_library(v22bs_helpers STATIC
    tests/v22bs_impl.h
    tests/v22bs_side_ours.c
    tests/v22bs_channel.h
)
target_include_directories(v22bs_helpers PUBLIC ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(v22bs_helpers softmodem ${MATH_LIB})

# V.22bis loopback test (modem vs modem in-process)
add_executable(v22bis_loopback tests/v22bis_loopback.c)
target_link_libraries(v22bis_loopback softmodem ${MATH_LIB})

# PTY byte transport test
add_executable(v22bis_pty tests/v22bis_pty.c)
target_link_libraries(v22bis_pty v22bs_helpers softmodem ${MATH_LIB})

# PPP bridge test (two pppd instances joined by the modem pair)
add_executable(pppbr tests/pppbr.c)
target_link_libraries(pppbr v22bs_helpers softmodem ${MATH_LIB})

# AudioSocket pipe: Asterisk substitute driving the daemon with a software
# calling modem and a real pppd on the client side.
add_executable(as_client tests/as_client.c)
target_link_libraries(as_client v22bs_helpers softmodem ${MATH_LIB})

# Reference (spandsp) cross-interop test - optional, needs spandsp headers
find_path(SPANDSP_INCLUDE_DIR spandsp/v22bis.h
    PATHS /usr/include /usr/local/include /tmp/opencode/spandsp/src
)
if(SPANDSP_INCLUDE_DIR)
    message(STATUS "spandsp headers found at ${SPANDSP_INCLUDE_DIR}")
    add_library(v22bs_ref STATIC tests/v22bs_side_ref.c)
    target_include_directories(v22bs_ref PUBLIC ${SPANDSP_INCLUDE_DIR})
    target_link_libraries(v22bs_ref softmodem ${MATH_LIB})
    add_executable(v22bis_cross tests/v22bis_cross.c)
    target_link_libraries(v22bis_cross v22bs_helpers v22bs_ref softmodem ${MATH_LIB})
else()
    message(STATUS "spandsp headers not found - v22bis_cross skipped")
endif()
