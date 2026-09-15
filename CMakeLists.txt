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
    ${CMAKE_SOURCE_DIR}/src/sip
    ${CMAKE_SOURCE_DIR}/src/call
    ${CMAKE_SOURCE_DIR}/src/dsp
    ${CMAKE_SOURCE_DIR}/src/dsp/carrier
    ${CMAKE_SOURCE_DIR}/src/dsp/detectors
    ${CMAKE_SOURCE_DIR}/src/modem
    ${CMAKE_SOURCE_DIR}/src/modem/v22bis
)

# Math library
set(MATH_LIB m)

# Optional V.8 support (spanDSP-based). Requires the spanDSP source tree used
# to build src/modem/v8/v8build/libsmv8.a (see src/modem/v8/build_v8.sh)
# and the vendored headers in third_party/spandsp.
find_path(SPANDSP_V8_INC spandsp/v8.h
    PATHS ${CMAKE_SOURCE_DIR}/third_party/spandsp/src
)
find_library(SPANDSP_V8_LIB smv8
    PATHS ${CMAKE_SOURCE_DIR}/src/modem/v8/v8build
)
if(SPANDSP_V8_INC AND SPANDSP_V8_LIB)
    message(STATUS "V.8 support enabled (${SPANDSP_V8_LIB})")
    set(SM_V8_SRC src/modem/v8/sm_v8.c)
    set(SM_V8_DEFS SM_HAVE_V8)
    set(SM_V8_LIBS ${SPANDSP_V8_LIB})
    include_directories(${SPANDSP_V8_INC})
else()
    message(STATUS "V.8 support disabled (build libsmv8.a first)")
    set(SM_V8_SRC "")
    set(SM_V8_DEFS "")
    set(SM_V8_LIBS "")
endif()

# Core library: modem + infra
add_library(softmodem STATIC
    src/modem/v22bis/v22bis_tx.c
    src/modem/v22bis/v22bis_rx.c
    src/logging/sm_log.c
    src/serial/sm_async.c
    src/ast_socket/sm_ast_socket.c
    src/ppp/sm_pppd.c
    src/call/sm_call.c
    src/sip/sm_sip.c
    ${SM_V8_SRC}
)
target_compile_definitions(softmodem PUBLIC ${SM_V8_DEFS})
target_link_libraries(softmodem ${MATH_LIB} ${SM_V8_LIBS})

# The answerer daemon
add_executable(sm_daemon src/main/sm_daemon.c)
target_link_libraries(sm_daemon softmodem ${MATH_LIB})

# SIP/RTP bridge for the PAP2T ATA (replaces Asterisk when needed)
add_executable(sm_sip src/main/sm_sip_main.c)
target_link_libraries(sm_sip softmodem ${MATH_LIB})

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

# SIP/RTP pipe: PAP2T substitute driving sm_sip with a software calling modem
# and a real pppd on the client side.
add_executable(sip_client tests/sip_client.c)
target_link_libraries(sip_client v22bs_helpers softmodem ${MATH_LIB})

# Reference (spandsp) cross-interop test - optional. Needs spanDSP headers
# and the ref_-prefixed reference library built by tests/build_ref.sh into
# tests/refbuild/. Without it the target is skipped and every other target
# still builds.
find_path(SPANDSP_INCLUDE_DIR spandsp/v22bis.h
    PATHS ${CMAKE_SOURCE_DIR}/third_party/spandsp/src
          /usr/include /usr/local/include /tmp/opencode/spandsp/src
)
find_library(SPANDSP_REF_LIB refspandsp
    PATHS ${CMAKE_SOURCE_DIR}/tests/refbuild
)
if(SPANDSP_INCLUDE_DIR AND SPANDSP_REF_LIB)
    message(STATUS "spandsp headers found at ${SPANDSP_INCLUDE_DIR}")
    message(STATUS "spandsp reference library found at ${SPANDSP_REF_LIB}")
    add_library(v22bs_ref STATIC tests/v22bs_side_ref.c)
    target_include_directories(v22bs_ref PUBLIC ${SPANDSP_INCLUDE_DIR})
    target_link_libraries(v22bs_ref softmodem ${MATH_LIB})
    add_executable(v22bis_cross tests/v22bis_cross.c)
    target_link_libraries(v22bis_cross v22bs_helpers v22bs_ref softmodem
                          ${SPANDSP_REF_LIB} ${MATH_LIB})
else()
    message(STATUS "spandsp reference library not found - v22bis_cross skipped "
                   "(build it with tests/build_ref.sh)")
endif()
