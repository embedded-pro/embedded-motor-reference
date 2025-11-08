add_executable(motor_ref.motor.dc.logic_test)
emil_build_for(motor_ref.motor.dc.logic_test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_ref.motor.dc.logic_test)

target_link_libraries(motor_ref.motor.dc.logic_test PUBLIC
    gmock_main
    motor_ref.dc.components
    hal.interfaces_test_doubles
    infra.timer_test_helper
    infra.event_test_helper
)

target_sources(motor_ref.motor.dc.logic_test PRIVATE
    TestMotorPidControllerImpl.cpp
    TestTerminal.cpp
)
