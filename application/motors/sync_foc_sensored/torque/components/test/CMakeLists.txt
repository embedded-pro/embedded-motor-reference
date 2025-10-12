add_executable(motor_ref.motor.bldc.logic_test)
emil_build_for(motor_ref.motor.bldc.logic_test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_ref.motor.bldc.logic_test)

target_link_libraries(motor_ref.motor.bldc.logic_test PUBLIC
    gmock_main
    motor_ref.sync_foc_sensored.torque.components
    hal.interfaces_test_doubles
    infra.timer_test_helper
    infra.event_test_helper
    motor_ref.foc_mock
    motor_ref.pid_mock
)

target_sources(motor_ref.motor.bldc.logic_test PRIVATE
    TestFieldOrientedControllerInteractorImpl.cpp
    TestTerminal.cpp
)
