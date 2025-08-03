add_library(motor_reference.synchronous_foc_sensored.torque.instantiations ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(motor_reference.synchronous_foc_sensored.torque.instantiations)

target_include_directories(motor_reference.synchronous_foc_sensored.torque.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.synchronous_foc_sensored.torque.instantiations PUBLIC
    motor_reference.hardware
    motor_reference.synchronous_foc_sensored.torque.components
    motor_reference.foc.instantiations
    numerical.controllers
)

target_sources(motor_reference.synchronous_foc_sensored.torque.instantiations PRIVATE
    Logic.cpp
    Logic.hpp
)
