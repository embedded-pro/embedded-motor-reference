add_library(motor_ref.foc.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.foc.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.foc.implementations PUBLIC
    motor_ref.foc.interfaces
)

target_sources(motor_ref.foc.implementations PRIVATE
    ControllerBaseImpl.cpp
    ControllerBaseImpl.hpp
    SpeedControllerImpl.cpp
    SpeedControllerImpl.hpp
    TorqueControllerImpl.cpp
    TorqueControllerImpl.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
