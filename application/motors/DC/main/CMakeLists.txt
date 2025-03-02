add_executable(motor_reference.dc.main)

target_include_directories(motor_reference.dc.main PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.dc.main PUBLIC
    motor_reference.hardware
    motor_reference.dc.instantiations
    numerical.controllers
)

target_sources(motor_reference.dc.main PRIVATE
    Main.cpp
)
