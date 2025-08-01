add_executable(motor_reference.hardware_test.components.test)
emil_build_for(motor_reference.hardware_test.components.test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_reference.hardware_test.components.test)

target_link_libraries(motor_reference.hardware_test.components.test PUBLIC
    gmock_main
    motor_reference.synchrounous_foc_sensored.torque.components
    hal.interfaces_test_doubles
    infra.timer_test_helper
    infra.event_test_helper
    motor_reference.foc_mock
    motor_reference.pid_mock
)

target_sources(motor_reference.hardware_test.components.test PRIVATE
    TestTerminal.cpp
)
