add_library(motor_ref.hardware_test.components ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.hardware_test.components PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.hardware_test.components PUBLIC
    infra.util
    motor_ref.pid
    motor_ref.hardware
    motor_ref.foc
    services.util
    services.tracer
    motor_ref.foc.instantiations
)

target_sources(motor_ref.hardware_test.components PRIVATE
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
