add_library(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM})

target_include_directories(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM} PUBLIC
    motor_reference.hardware
    infra.event
    infra.timer
    services.tracer
    hal_tiva.tiva
    hal_tiva.synchronous_tiva
)

target_sources(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM} PRIVATE
    HardwareFactoryImpl.cpp
    HardwareFactoryImpl.hpp
    Pins.hpp
)
