add_library(motor_reference.hardware)
emil_build_for(motor_reference.hardware)

target_include_directories(motor_reference.hardware PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.hardware PUBLIC
    services.util
    hal.interfaces
    hal.synchronous_interfaces
    infra.util
    infra.event
    motor_reference.foc
    motor_reference.pid
)

target_sources(motor_reference.hardware PRIVATE
    HardwareFactory.hpp
    MotorFieldOrientedControllerAdapter.hpp
    MotorFieldOrientedControllerAdapter.cpp
    PidAdapter.hpp
    PidAdapter.cpp
)

add_subdirectory(${MOTOR_REFERENCE_TARGET_PLATFORM})

add_library(motor_reference.hardware_impl INTERFACE)

target_compile_definitions(motor_reference.hardware_impl INTERFACE
    HARDWARE_FACTORY_IMPL_HEADER="application/hardware/${MOTOR_REFERENCE_TARGET_PLATFORM}/implementation/HardwareFactoryImpl.hpp"
)

target_link_libraries(motor_reference.hardware_impl INTERFACE
    motor_reference.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM}
)
