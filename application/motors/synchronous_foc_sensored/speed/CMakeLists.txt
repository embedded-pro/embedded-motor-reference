add_library(application.motors.synchronous_foc_sensored.speed ${EMIL_EXCLUDE_FROM_ALL} STATIC)

target_include_directories(application.motors.synchronous_foc_sensored.speed PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(application.motors.synchronous_foc_sensored.speed PUBLIC
    application.foc
    numerical.controllers
    infra.util
    hal.interfaces
)

target_sources(application.motors.synchronous_foc_sensored.speed PRIVATE
    components/SpeedController.hpp
    components/SpeedControllerImpl.cpp
)
