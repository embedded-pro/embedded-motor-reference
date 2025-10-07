function(generate_development_environment_script)
    set(options "")
    set(oneValueArgs TARGET_MCU GDB_PORT)
    set(multiValueArgs "")

    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARGS_TARGET_MCU)
        message(FATAL_ERROR "generate_development_environment_script: TARGET_MCU is required")
    endif()

    if(NOT ARGS_GDB_PORT)
        message(FATAL_ERROR "generate_development_environment_script: GDB_PORT is required")
    endif()

    string(TOUPPER "${ARGS_TARGET_MCU}" TARGET_MCU_UPPER)

    set(GDB_PORT "${ARGS_GDB_PORT}")

    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/development")

    configure_file(
        "${CMAKE_SOURCE_DIR}/scripts/start-development-environment.ps1.in"
        "${CMAKE_BINARY_DIR}/development/start-development-environment.ps1"
        @ONLY
    )

    message(STATUS "Generated PowerShell development script: ${CMAKE_BINARY_DIR}/development/start-development-environment.ps1")
    message(STATUS "  - Target MCU: ${TARGET_MCU_UPPER}")
    message(STATUS "  - GDB Port: ${GDB_PORT}")
endfunction()
