add_executable(motor_reference.dc)
emil_build_for(motor_reference.dc)

target_include_directories(motor_reference.dc PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.dc PUBLIC
    motor_reference.hardware
    motor_reference.logic
    numerical.controllers
)

target_sources(motor_reference.dc PRIVATE
    Main.cpp
)

add_subdirectory(logic)
