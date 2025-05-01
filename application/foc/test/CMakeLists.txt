add_executable(motor_reference.foc_test)
emil_build_for(motor_reference.foc_test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_reference.foc_test)

target_link_libraries(motor_reference.foc_test PUBLIC
    gmock_main
    motor_reference.foc
    numerical.math_test_helper
    infra.timer_test_helper
    motor_reference.foc_mock
    motor_reference.pid_mock
)

target_sources(motor_reference.foc_test PRIVATE
    TestMotorFieldOrientedController.cpp
)
