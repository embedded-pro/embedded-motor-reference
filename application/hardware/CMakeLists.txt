add_library(motor_reference.hardware INTERFACE)
emil_build_for(motor_reference.hardware)

target_include_directories(motor_reference.hardware INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.hardware INTERFACE
    services.util
    hal.interfaces
    hal.synchronous_interfaces
    infra.util
    infra.event
)

target_sources(motor_reference.hardware PRIVATE
    HardwareFactory.hpp
)

add_subdirectory(${MOTOR_REFERENCE_TARGET_PLATFORM})

add_library(motor_reference.hardware_impl INTERFACE)

target_compile_definitions(motor_reference.hardware_impl INTERFACE
    HARDWARE_FACTORY_IMPL_HEADER="application/hardware/${MOTOR_REFERENCE_TARGET_PLATFORM}/HardwareFactoryImpl.hpp"
)

target_link_libraries(motor_reference.hardware_impl INTERFACE
    motor_reference.hardware_${MOTOR_REFERENCE_TARGET_PLATFORM}
)
