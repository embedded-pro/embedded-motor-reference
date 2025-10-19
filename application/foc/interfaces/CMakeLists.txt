add_library(motor_ref.foc.interfaces INTERFACE ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.foc.interfaces INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.foc.interfaces INTERFACE
    infra.util
    numerical.math
    numerical.controllers
    motor_ref.pid
)

target_sources(motor_ref.foc.interfaces PRIVATE
    Controller.hpp
    Driver.hpp
    FieldOrientedController.hpp
)
