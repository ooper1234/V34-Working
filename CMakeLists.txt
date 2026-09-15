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
    ${CMAKE_SOURCE_DIR}/src/dsp
    ${CMAKE_SOURCE_DIR}/src/dsp/carrier
    ${CMAKE_SOURCE_DIR}/src/dsp/detectors
    ${CMAKE_SOURCE_DIR}/src/modem
    ${CMAKE_SOURCE_DIR}/src/modem/v22bis
)

# Math library
set(MATH_LIB m)

# V.22bis library
add_library(v22bis STATIC
    src/modem/v22bis/v22bis_tx.c
    src/modem/v22bis/v22bis_rx.c
    src/logging/sm_log.c
)
target_link_libraries(v22bis ${MATH_LIB})

# V.22bis loopback test
add_executable(v22bis_loopback tests/v22bis_loopback.c)
target_link_libraries(v22bis_loopback v22bis ${MATH_LIB})

# V.22bis cross test helper implementations
add_library(v22bs_helpers STATIC
    tests/v22bs_impl.h
    tests/v22bs_side_ours.c
    tests/v22bs_side_ref.c
    tests/v22bs_channel.h
)
target_include_directories(v22bs_helpers PUBLIC ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(v22bs_helpers v22bis ${MATH_LIB})

# V.22bis cross-interop test
add_executable(v22bis_cross tests/v22bis_cross.c)
target_link_libraries(v22bis_cross v22bs_helpers v22bis ${MATH_LIB})

# PPP bridge test
add_executable(pppbr tests/pppbr.c)
target_link_libraries(pppbr v22bs_helpers v22bis ${MATH_LIB})