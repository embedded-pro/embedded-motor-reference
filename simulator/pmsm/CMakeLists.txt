add_library(motor_ref.simulator.pmsm ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.simulator.pmsm PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.simulator.pmsm PUBLIC
    infra.util
    numerical.math
    numerical.controllers.implementations
    motor_ref.foc.instantiations
)

target_sources(motor_ref.simulator.pmsm PRIVATE
    Model.hpp
    Model.cpp
)
