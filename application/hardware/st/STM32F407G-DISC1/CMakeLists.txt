add_library(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_BOARD} INTERFACE)

target_include_directories(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_BOARD} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../..>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_BOARD} INTERFACE
    hal_st.stm32fxxx
    hal_st.synchronous_stm32fxxx
)

target_sources(motor_reference.hardware_${MOTOR_REFERENCE_TARGET_BOARD} INTERFACE
    PinsAndPeripherals.hpp
)
