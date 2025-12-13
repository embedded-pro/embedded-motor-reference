add_executable(e_foc.motor.dc.logic_test)
emil_build_for(e_foc.motor.dc.logic_test HOST All BOOL E_FOC_BUILD_TESTS)
emil_add_test(e_foc.motor.dc.logic_test)

target_link_libraries(e_foc.motor.dc.logic_test PUBLIC
    gmock_main
    e_foc.dc.components
    hal.interfaces_test_doubles
    infra.timer_test_helper
    infra.event_test_helper
)

target_sources(e_foc.motor.dc.logic_test PRIVATE
    TestMotorPidControllerImpl.cpp
    TestTerminal.cpp
)
