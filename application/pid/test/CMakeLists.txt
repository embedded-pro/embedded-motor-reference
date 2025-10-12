add_executable(motor_ref.pid_test)
emil_build_for(motor_ref.pid_test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_ref.pid_test)

target_link_libraries(motor_ref.pid_test PUBLIC
    gmock_main
    motor_ref.pid
    numerical.math_test_helper
    infra.timer_test_helper
    motor_ref.pid_mock
)

target_sources(motor_ref.pid_test PRIVATE
    TestPid.cpp
)
