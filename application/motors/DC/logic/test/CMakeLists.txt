add_executable(motor_reference.motor.dc.logic_test)
emil_build_for(motor_reference.motor.dc.logic_test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_reference.motor.dc.logic_test)

target_link_libraries(motor_reference.motor.dc.logic_test PUBLIC
    gmock_main
    motor_reference.logic
    hal.interfaces_test_doubles
    infra.event_test_helper
)

target_sources(motor_reference.motor.dc.logic_test PRIVATE
    TestTerminal.cpp
)
