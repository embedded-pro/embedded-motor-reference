add_library(motor_reference.logic ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.logic PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.logic PUBLIC
    infra.util
    numerical.math
    numerical.controllers
    motor_reference.hardware
)

target_sources(motor_reference.logic PRIVATE
    MotorController.hpp
    MotorControllerImpl.cpp
    MotorControllerImpl.hpp
)

add_subdirectory(test)
