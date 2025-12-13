add_library(motor_ref.hardware)
emil_build_for(motor_ref.hardware)

target_include_directories(motor_ref.hardware PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.hardware PUBLIC
    services.util
    hal.interfaces
    hal.synchronous_interfaces
    infra.util
    infra.event
    motor_ref.foc.interfaces
)

target_sources(motor_ref.hardware PRIVATE
    HardwareFactory.hpp
    MotorFieldOrientedControllerAdapter.hpp
    MotorFieldOrientedControllerAdapter.cpp
    PidAdapter.hpp
    PidAdapter.cpp
)

add_subdirectory(${E_FOC_TARGET_PLATFORM})

add_library(motor_ref.hardware_impl INTERFACE)

target_compile_definitions(motor_ref.hardware_impl INTERFACE
    HARDWARE_FACTORY_IMPL_HEADER="application/hardware/${E_FOC_TARGET_PLATFORM}/implementation/HardwareFactoryImpl.hpp"
)

target_link_libraries(motor_ref.hardware_impl INTERFACE
    motor_ref.hardware_${E_FOC_TARGET_PLATFORM}
)
