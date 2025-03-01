add_library(motor_reference.dc.instantiations ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(motor_reference.dc.instantiations)

target_include_directories(motor_reference.dc.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.dc.instantiations PUBLIC
    motor_reference.hardware
    motor_reference.logic
    numerical.controllers
)

target_sources(motor_reference.dc.instantiations PRIVATE
    Logic.cpp
    Logic.hpp
)
