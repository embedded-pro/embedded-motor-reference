add_executable(simulator.torque_control)
emil_build_for(simulator.torque_control HOST All PREREQUISITE_BOOL E_FOC_BUILD_SIMULATOR)

target_link_libraries(simulator.torque_control PUBLIC
    sciplot
    e_foc.simulator.pmsm
    e_foc.foc.instantiations
)

target_sources(simulator.torque_control PRIVATE
    Main.cpp
    Plot.cpp
    Plot.hpp
)

if(MSVC)
    target_compile_options(simulator.torque_control PRIVATE /W4)
else()
    target_compile_options(simulator.torque_control PRIVATE 
        -Wno-error
    )
endif()
