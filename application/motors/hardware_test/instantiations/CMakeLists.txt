add_library(motor_reference.hardware_test.instantiations ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(motor_reference.hardware_test.instantiations)

target_include_directories(motor_reference.hardware_test.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.hardware_test.instantiations PUBLIC
    motor_reference.hardware
    motor_reference.hardware_test.components
    motor_reference.foc.instantiations
    numerical.controllers
)

target_sources(motor_reference.hardware_test.instantiations PRIVATE
    Logic.cpp
    Logic.hpp
)
