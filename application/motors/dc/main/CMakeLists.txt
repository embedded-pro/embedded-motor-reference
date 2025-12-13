add_executable(e_foc.dc.main)

target_include_directories(e_foc.dc.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_foc.dc.main PUBLIC
    e_foc.hardware_impl
    e_foc.dc.instantiations
    numerical.controllers.implementations
)

target_sources(e_foc.dc.main PRIVATE
    Main.cpp
)

if(TARGET_MCU_VENDOR STREQUAL "st")
    halst_target_default_linker_scripts(e_foc.dc.main)
    halst_target_default_init(e_foc.dc.main)
elseif(TARGET_MCU_VENDOR STREQUAL "ti")
    hal_ti_target_default_linker_scripts(e_foc.dc.main)
    hal_ti_target_default_init(e_foc.dc.main)
endif()

if(DEFINED TARGET_MCU_VENDOR)
    emil_generate_artifacts(TARGET e_foc.dc.main LST MAP BIN HEX)
endif()
