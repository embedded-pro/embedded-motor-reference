add_executable(motor_ref.dc.main)

target_include_directories(motor_ref.dc.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.dc.main PUBLIC
    motor_ref.hardware_impl
    motor_ref.dc.instantiations
    numerical.controllers
)

target_sources(motor_ref.dc.main PRIVATE
    Main.cpp
)

if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(motor_ref.dc.main)
    halst_target_default_init(motor_ref.dc.main)
elseif(TARGET_MCU_VENDOR STREQUAL "ti")
    hal_ti_target_default_linker_scripts(motor_ref.dc.main)
    hal_ti_target_default_init(motor_ref.dc.main)
endif()

if(DEFINED TARGET_MCU_VENDOR)
    emil_generate_artifacts(TARGET motor_ref.dc.main LST MAP BIN HEX)
endif()
