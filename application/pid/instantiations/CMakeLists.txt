add_library(motor_reference.pid.instantiations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.pid.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.pid.instantiations PUBLIC
    motor_reference.pid
)

target_sources(motor_reference.pid.instantiations PRIVATE
    PidImpl.cpp
    PidImpl.hpp
)
