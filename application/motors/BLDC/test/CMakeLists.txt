add_executable(motor_reference.bldc_test)
emil_build_for(motor_reference.bldc_test BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(motor_reference.bldc_test)

target_link_libraries(motor_reference.bldc_test PUBLIC
    gmock_main
)

target_sources(motor_reference.bldc_test PRIVATE
    TestBldc.cpp
)
