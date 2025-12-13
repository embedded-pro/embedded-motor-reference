add_library(e_foc.dc.components ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_foc.dc.components PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_foc.dc.components PUBLIC
    infra.util
    numerical.controllers.interfaces
    e_foc.hardware
    services.util
    services.tracer
)

target_sources(e_foc.dc.components PRIVATE
    MotorPidController.hpp
    MotorPidControllerImpl.cpp
    MotorPidControllerImpl.hpp
    Terminal.cpp
    Terminal.hpp
)

add_subdirectory(test)
