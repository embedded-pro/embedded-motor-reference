add_library(e_foc.services.terminal ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_foc.services.terminal PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_foc.services.terminal PUBLIC
    infra.util
    e_foc.hardware
    e_foc.foc.interfaces
    services.util
    services.tracer
)

target_sources(e_foc.services.terminal PRIVATE
    FocInteractor.hpp
    FocTorqueInteractorImpl.cpp
    FocTorqueInteractorImpl.hpp
    TerminalTorque.cpp
    TerminalTorque.hpp
)

add_subdirectory(test)
