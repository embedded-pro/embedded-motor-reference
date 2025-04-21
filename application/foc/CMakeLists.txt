add_library(motor_reference.foc ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.foc PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.foc PUBLIC
    infra.util
    numerical.math
    numerical.controllers
    motor_reference.pid
)

target_sources(motor_reference.foc PRIVATE
    FieldOrientedControllerInterface.hpp
    MotorFieldOrientedController.cpp
    MotorFieldOrientedController.hpp
)

add_subdirectory(instantiations)
add_subdirectory(test)
add_subdirectory(test_doubles)
