add_library(motor_reference.foc_mock)
emil_build_for(motor_reference.foc_mock HOST All BOOL MOTOR_REFERENCE_BUILD_TESTS)

target_link_libraries(motor_reference.foc_mock PUBLIC
    gmock_main
    motor_reference.foc
    numerical.math_test_helper
    infra.timer_test_helper
)

target_sources(motor_reference.foc_mock PRIVATE
    FieldOrientedControllerInterfaceMock.hpp
    FieldOrientedControllerMock.hpp
    MotorFieldOrientedControllerMock.hpp
)
