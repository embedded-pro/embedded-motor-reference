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

set(HALST_INCLUDE_DEFAULT_LINKER_SCRIPTS Off CACHE INTERNAL "")
set(HALST_BUILD_EXAMPLES Off CACHE INTERNAL "")

if(DEFINED EMIL_SEGGER_RTT_BUFFER_SIZE_UP)
    set(SEGGER_RTT_BUFFER_SIZE_UP ${EMIL_SEGGER_RTT_BUFFER_SIZE_UP} CACHE INTERNAL "")
endif()

add_definitions(-DEMIL_ENABLE_TRACING=1)

if(DEFINED MOTOR_REFERENCE_TARGET_BOARD)
    add_definitions(-DMOTOR_REFERENCE_TARGET_BOARD="${MOTOR_REFERENCE_TARGET_BOARD}")
else()
    add_definitions(-DMOTOR_REFERENCE_TARGET_BOARD="host")
endif()

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

if (DEFINED TARGET_MCU_VENDOR)
    add_subdirectory(hal/${TARGET_MCU_VENDOR})
endif()

add_subdirectory(numerical-toolbox)
add_subdirectory(application)

if (MOTOR_REFERENCE_BUILD_SIMULATOR)
    add_subdirectory(simulator)
endif()

emil_clangformat_directories(motor_ref DIRECTORIES application)
emil_folderize_all_targets()

include(scripts/development_environment.cmake)

if (DEFINED TARGET_MCU)
    generate_development_environment_script(
        TARGET_MCU "${TARGET_MCU}"
        GDB_PORT "2334"
    )
endif()

set(CPACK_GENERATOR "ZIP;TGZ")
set(CPACK_SOURCE_IGNORE_FILES ".vs/;.git/;build/")
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION "${PROJECT_DESCRIPTION}")
set(CPACK_DEBIAN_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
set(CPACK_RPM_PACKAGE_NAME "${CPACK_PACKAGE_NAME}")
set(CPACK_PACKAGE_HOMEPAGE_URL "${PROJECT_HOMEPAGE_URL}")
set(CPACK_PACKAGE_MAINTAINER "${CPACK_PACKAGE_VENDOR}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_VENDOR}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

include(CPack)
