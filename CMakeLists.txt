cmake_minimum_required(VERSION 3.24)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(MOTOR_REFERENCE_STANDALONE On)
endif()

option(CMAKE_COMPILE_WARNING_AS_ERROR "Enable warnings-as-error" On)
option(MOTOR_REFERENCE_INCLUDE_DEFAULT_INIT "Include default initialization code; turn off when providing custom initialization" On)
option(MOTOR_REFERENCE_BUILD_TESTS "Enable build of the examples" Off)

if (MOTOR_REFERENCE_BUILD_TESTS)
    set(BUILD_TESTING On)
endif()

add_definitions(-DEMIL_ENABLE_TRACING=1)

project(motor_reference LANGUAGES C CXX ASM VERSION 0.0.1) # x-release-please-version

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set_directory_properties(PROPERTY USE_FOLDERS ON)

set(MOTOR_REFERENCE_EXCLUDE_FROM_ALL "")

add_subdirectory(embedded-infra-lib)

if (MOTOR_REFERENCE_BUILD_TESTS)
    include(CTest)
    emil_enable_testing()
endif()

add_subdirectory(numerical-toolbox)
add_subdirectory(application)

if (MOTOR_REFERENCE_BUILD_FOR_EMBEDDED)
    if (TARGET_MCU_VENDOR STREQUAL "st")
        add_subdirectory(hal/st)
    elseif(TARGET_MCU_VENDOR STREQUAL "ti")
        add_subdirectory(hal/ti)
    else()
        message(ERROR "Vendor not supported!")
    endif()
endif()

emil_clangformat_directories(motor_reference DIRECTORIES application)
emil_folderize_all_targets()
