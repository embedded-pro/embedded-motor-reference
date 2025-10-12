add_library(motor_ref.pid_mock INTERFACE)
emil_build_for(motor_ref.pid_mock HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)

target_link_libraries(motor_ref.pid_mock INTERFACE
    gmock_main
    motor_ref.pid
    numerical.math_test_helper
    infra.timer_test_helper
)

target_sources(motor_ref.pid_mock PRIVATE
    PidInterfaceMock.hpp
    PidMock.hpp
)
