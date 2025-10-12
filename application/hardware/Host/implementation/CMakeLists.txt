add_library(motor_ref.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM})

target_include_directories(motor_ref.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM} PUBLIC
    motor_ref.hardware
    infra.event
    infra.timer
    services.tracer
)

target_sources(motor_ref.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM} PRIVATE
    HardwareFactoryImpl.cpp
    HardwareFactoryImpl.hpp
)
