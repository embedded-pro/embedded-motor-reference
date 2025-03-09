add_library(motor_reference.logic ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.logic PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.logic PUBLIC
    infra.util
    motor_reference.pid
    motor_reference.hardware
    services.util
    services.tracer
)

target_sources(motor_reference.logic PRIVATE
    MotorController.hpp
    MotorControllerImpl.cpp
    MotorControllerImpl.hpp
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
