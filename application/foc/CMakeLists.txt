add_library(motor_ref.foc ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.foc PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.foc PUBLIC
    infra.util
    numerical.math
    numerical.controllers
    motor_ref.pid
)

target_sources(motor_ref.foc PRIVATE
    FieldOrientedController.hpp
    MotorFieldOrientedControllerInterface.hpp
    MotorFieldOrientedController.cpp
    MotorFieldOrientedController.hpp
)

add_subdirectory(instantiations)
add_subdirectory(test)
add_subdirectory(test_doubles)
