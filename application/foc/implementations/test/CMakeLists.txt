add_executable(motor_ref.foc_test)
emil_build_for(motor_ref.foc_test HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_ref.foc_test)

target_link_libraries(motor_ref.foc_test PUBLIC
    gmock_main
    motor_ref.foc.implementations
    numerical.math_test_helper
    infra.timer_test_helper
    motor_ref.foc_mock
)

target_sources(motor_ref.foc_test PRIVATE
    TestSpaceVectorModulation.cpp
    TestTorqueController.cpp
    TestTransformsClarkePark.cpp
)
