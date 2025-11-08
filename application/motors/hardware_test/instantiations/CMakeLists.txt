add_library(motor_ref.hardware_test.instantiations ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(motor_ref.hardware_test.instantiations)

target_include_directories(motor_ref.hardware_test.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.hardware_test.instantiations PUBLIC
    motor_ref.hardware
    motor_ref.hardware_test.components
    motor_ref.foc.instantiations
    numerical.controllers.implementations
)

target_sources(motor_ref.hardware_test.instantiations PRIVATE
    Logic.cpp
    Logic.hpp
)
