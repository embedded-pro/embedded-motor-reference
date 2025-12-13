add_library(motor_ref.hardware_${E_FOC_TARGET_PLATFORM})

target_include_directories(motor_ref.hardware_${E_FOC_TARGET_PLATFORM} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.hardware_${E_FOC_TARGET_PLATFORM} PUBLIC
    motor_ref.hardware
    infra.event
    infra.timer
    services.tracer
    motor_ref.hardware_${E_FOC_TARGET_BOARD}
)

target_compile_definitions(motor_ref.hardware_${E_FOC_TARGET_PLATFORM} PUBLIC
    HARDWARE_PINS_AND_PERIPHERALS_HEADER="application/hardware/${E_FOC_TARGET_PLATFORM}/${E_FOC_TARGET_BOARD}/PinsAndPeripherals.hpp"
)

target_sources(motor_ref.hardware_${E_FOC_TARGET_PLATFORM} PRIVATE
    HardwareFactoryImpl.cpp
    HardwareFactoryImpl.hpp
)
