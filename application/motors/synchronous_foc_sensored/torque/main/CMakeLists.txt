add_executable(motor_reference.synchronous_foc_sensored.torque.main)

target_include_directories(motor_reference.synchronous_foc_sensored.torque.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.synchronous_foc_sensored.torque.main PUBLIC
    motor_reference.hardware_impl
    motor_reference.synchronous_foc_sensored.torque.instantiations
    numerical.controllers
)

target_sources(motor_reference.synchronous_foc_sensored.torque.main PRIVATE
    Main.cpp
)

if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(motor_reference.synchronous_foc_sensored.torque.main)
    halst_target_default_init(motor_reference.synchronous_foc_sensored.torque.main)
elseif(TARGET_MCU_VENDOR STREQUAL "ti")
    hal_ti_target_default_linker_scripts(motor_reference.synchronous_foc_sensored.torque.main)
    hal_ti_target_default_init(motor_reference.synchronous_foc_sensored.torque.main)
endif()

if(DEFINED TARGET_MCU_VENDOR)
    emil_generate_artifacts(TARGET motor_reference.synchronous_foc_sensored.torque.main LST MAP BIN HEX)
endif()
