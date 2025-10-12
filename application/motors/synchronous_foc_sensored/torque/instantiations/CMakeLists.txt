add_library(motor_ref.sync_foc_sensored.torque.inst ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(motor_ref.sync_foc_sensored.torque.inst)

target_include_directories(motor_ref.sync_foc_sensored.torque.inst PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.sync_foc_sensored.torque.inst PUBLIC
    motor_ref.hardware
    motor_ref.sync_foc_sensored.torque.components
    motor_ref.foc.instantiations
    numerical.controllers
)

target_sources(motor_ref.sync_foc_sensored.torque.inst PRIVATE
    Logic.cpp
    Logic.hpp
    TrigonometricImpl.cpp
    TrigonometricImpl.hpp
)
