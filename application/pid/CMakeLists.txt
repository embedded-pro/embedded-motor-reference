add_library(motor_reference.pid ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_reference.pid PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_reference.pid PUBLIC
    infra.util
    numerical.math
    numerical.controllers
)

target_sources(motor_reference.pid PRIVATE
    Pid.cpp
    Pid.hpp
    PidInterface.hpp
)

add_subdirectory(instantiations)
add_subdirectory(test)
add_subdirectory(test_doubles)
