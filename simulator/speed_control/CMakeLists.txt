add_executable(simulator.speed_control)
emil_build_for(simulator.speed_control HOST All PREREQUISITE_BOOL E_FOC_BUILD_SIMULATOR)

target_link_libraries(simulator.speed_control PUBLIC
    sciplot
    e_foc.simulator.pmsm
    e_foc.foc.instantiations
)

target_sources(simulator.speed_control PRIVATE
    Main.cpp
    Plot.cpp
    Plot.hpp
)

if(MSVC)
    target_compile_options(simulator.speed_control PRIVATE /W4)
else()
    target_compile_options(simulator.speed_control PRIVATE 
        -Wno-error
    )
endif()
