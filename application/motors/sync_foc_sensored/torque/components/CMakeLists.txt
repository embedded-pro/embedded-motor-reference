add_library(e_foc.sync_foc_sensored.torque.components ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_foc.sync_foc_sensored.torque.components PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_foc.sync_foc_sensored.torque.components PUBLIC
    infra.util
    e_foc.hardware
    e_foc.foc.instantiations
    services.util
    services.tracer
)

target_sources(e_foc.sync_foc_sensored.torque.components PRIVATE
    FieldOrientedControllerInteractor.hpp
    FieldOrientedControllerInteractorImpl.cpp
    FieldOrientedControllerInteractorImpl.hpp
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
