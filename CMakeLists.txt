cmake_minimum_required(VERSION 3.24)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MOTOR_REFERENCE_STANDALONE On)
endif()

option(CMAKE_COMPILE_WARNING_AS_ERROR "Enable warnings-as-error" On)
option(MOTOR_REFERENCE_INCLUDE_DEFAULT_INIT "Include default initialization code; turn off when providing custom initialization" On)
option(MOTOR_REFERENCE_BUILD_TESTS "Enable build of the examples" Off)

if (MOTOR_REFERENCE_BUILD_TESTS)
    set(EMIL_BUILD_TESTS On)
endif()

add_definitions(-DEMIL_ENABLE_TRACING=1)

project(motor_reference LANGUAGES C CXX ASM VERSION 0.0.1) # x-release-please-version

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set_directory_properties(PROPERTY USE_FOLDERS ON)

add_subdirectory(embedded-infra-lib)
add_subdirectory(dsp-control)
add_subdirectory(hal/st)
add_subdirectory(hal/ti)
add_subdirectory(application)

emil_clangformat_directories(motor_reference DIRECTORIES application)
emil_folderize_all_targets()
