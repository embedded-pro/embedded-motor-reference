add_library(motor_ref.dc.components ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.dc.components PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.dc.components PUBLIC
    infra.util
    numerical.controllers.interfaces
    motor_ref.hardware
    services.util
    services.tracer
)

target_sources(motor_ref.dc.components PRIVATE
    MotorPidController.hpp
    MotorPidControllerImpl.cpp
    MotorPidControllerImpl.hpp
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
