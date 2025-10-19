add_library(motor_ref.foc_mock INTERFACE)
emil_build_for(motor_ref.foc_mock HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)

target_link_libraries(motor_ref.foc_mock INTERFACE
    gmock_main
    motor_ref.foc
    numerical.math_test_helper
    infra.timer_test_helper
)

target_sources(motor_ref.foc_mock PRIVATE
    DriversMock.hpp
    FieldOrientedControllerMock.hpp
    MotorFieldOrientedControllerMock.hpp
)
