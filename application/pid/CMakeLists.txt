add_library(motor_ref.pid ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(motor_ref.pid PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(motor_ref.pid PUBLIC
    infra.util
    numerical.math
    numerical.controllers
)

target_sources(motor_ref.pid PRIVATE
    PidImpl.cpp
    PidImpl.hpp
    PidInterface.hpp
)

add_subdirectory(test)
add_subdirectory(test_doubles)
