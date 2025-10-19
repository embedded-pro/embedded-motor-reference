add_library(motor_ref.foc.instantiations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.foc.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.foc.instantiations PUBLIC
    motor_ref.foc.implementations
)

target_sources(motor_ref.foc.instantiations PRIVATE
    FieldOrientedControllerImpl.cpp
    FieldOrientedControllerImpl.hpp
    TrigonometricImpl.cpp
    TrigonometricImpl.hpp
)
