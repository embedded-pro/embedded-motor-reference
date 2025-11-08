add_library(motor_ref.sync_foc_sensored.torque.components ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.sync_foc_sensored.torque.components PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.sync_foc_sensored.torque.components PUBLIC
    infra.util
    motor_ref.hardware
    motor_ref.foc.instantiations
    services.util
    services.tracer
)

target_sources(motor_ref.sync_foc_sensored.torque.components PRIVATE
    FieldOrientedControllerInteractor.hpp
    FieldOrientedControllerInteractorImpl.cpp
    FieldOrientedControllerInteractorImpl.hpp
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
