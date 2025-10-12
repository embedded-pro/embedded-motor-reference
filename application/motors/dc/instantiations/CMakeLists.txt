add_library(motor_ref.dc.instantiations ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(motor_ref.dc.instantiations)

target_include_directories(motor_ref.dc.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.dc.instantiations PUBLIC
    motor_ref.hardware
    motor_ref.dc.components
    numerical.controllers
)

target_sources(motor_ref.dc.instantiations PRIVATE
    Logic.cpp
    Logic.hpp
)
