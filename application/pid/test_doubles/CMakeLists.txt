add_library(motor_reference.pid_mock INTERFACE)
emil_build_for(motor_reference.pid_mock HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)

target_link_libraries(motor_reference.pid_mock INTERFACE
    gmock_main
    motor_reference.pid
    numerical.math_test_helper
    infra.timer_test_helper
)

target_sources(motor_reference.pid_mock PRIVATE
    PidInterfaceMock.hpp
    PidMock.hpp
)
