add_executable(simulator.torque_control)
emil_build_for(simulator.torque_control HOST All PREREQUISITE_BOOL MOTOR_REFERENCE_BUILD_SIMULATOR)

target_link_libraries(simulator.torque_control PUBLIC
    sciplot
    motor_ref.simulator.pmsm
    motor_ref.foc.instantiations
)

target_sources(simulator.torque_control PRIVATE
    Main.cpp
)

if(MSVC)
    target_compile_options(simulator.torque_control PRIVATE /W4)
else()
    target_compile_options(simulator.torque_control PRIVATE 
        -Wno-error
    )
endif()
