add_executable(application.bldc_test)
emil_build_for(application.bldc_test BOOL MOTOR_REFERENCE_BUILD_TESTS)
emil_add_test(application.bldc_test)

target_link_libraries(application.bldc_test PUBLIC
    gmock_main
)

target_sources(application.bldc_test PRIVATE
    TestBldc.cpp
)
