add_library(e_foc.services.parameter_identification ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_foc.services.parameter_identification PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_foc.services.parameter_identification PUBLIC
    infra.util
    e_foc.hardware
    e_foc.foc.interfaces
)

target_sources(e_foc.services.parameter_identification PRIVATE
    MotorIdentificationImpl.cpp
    MotorIdentificationImpl.hpp
    MotorIdentification.hpp
    TerminalMotorIdentification.cpp
    TerminalMotorIdentification.hpp
)

add_subdirectory(test)
