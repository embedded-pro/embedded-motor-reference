add_library(motor_reference.hardware_test.components ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.hardware_test.components PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.hardware_test.components PUBLIC
    infra.util
    motor_reference.pid
    motor_reference.hardware
    motor_reference.foc
    services.util
    services.tracer
)

target_sources(motor_reference.hardware_test.components PRIVATE
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
