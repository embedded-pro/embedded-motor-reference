add_executable(motor_ref.hardware_test.main)

target_include_directories(motor_ref.hardware_test.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.hardware_test.main PUBLIC
    motor_ref.hardware_impl
    motor_ref.hardware_test.instantiations
    numerical.controllers.implementations
)

target_sources(motor_ref.hardware_test.main PRIVATE
    Main.cpp
)

if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(motor_ref.hardware_test.main)
    halst_target_default_init(motor_ref.hardware_test.main)
elseif(TARGET_MCU_VENDOR STREQUAL "ti")
    hal_ti_target_default_linker_scripts(motor_ref.hardware_test.main)
    hal_ti_target_default_init(motor_ref.hardware_test.main)
endif()

if(DEFINED TARGET_MCU_VENDOR)
    emil_generate_artifacts(TARGET motor_ref.hardware_test.main LST MAP BIN HEX)
endif()
