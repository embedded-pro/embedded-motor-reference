add_executable(e_foc.motor.bldc.logic_test)
emil_build_for(e_foc.motor.bldc.logic_test HOST All BOOL E_FOC_BUILD_TESTS)
emil_add_test(e_foc.motor.bldc.logic_test)

target_link_libraries(e_foc.motor.bldc.logic_test PUBLIC
    gmock_main
    e_foc.sync_foc_sensored.torque.components
    hal.interfaces_test_doubles
    infra.timer_test_helper
    infra.event_test_helper
    e_foc.foc_mock
)

target_sources(e_foc.motor.bldc.logic_test PRIVATE
    TestFieldOrientedControllerInteractorImpl.cpp
    TestTerminal.cpp
)
