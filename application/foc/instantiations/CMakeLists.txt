add_library(motor_reference.foc.instantiations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.foc.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.foc.instantiations PUBLIC
    motor_reference.foc
)

target_sources(motor_reference.foc.instantiations PRIVATE
    FieldOrientedControllerImpl.cpp
    FieldOrientedControllerImpl.hpp
)
