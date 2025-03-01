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

# if (USB_HOST_BUILD_FOR_EMBEDDED)
#     add_subdirectory(${TARGET_MCU})

#     add_library(motor_reference.dc.hardware_impl INTERFACE)
#     emil_build_for(motor_reference.dc.hardware_impl PREREQUISITE_BOOL USB_HOST_BUILD_EXAMPLES)

#     target_compile_definitions(motor_reference.dc.hardware_impl INTERFACE
#         HARDWARE_MANAGER_IMPL_HEADER="examples/usb_host_cdc/hardware_abstraction/${TARGET_MCU}/HardwareManagerImpl.hpp"
#     )

#     target_link_libraries(motor_reference.dc.hardware_impl INTERFACE
#         motor_reference.dc.hardware_${TARGET_MCU}
#     )
# endif()
