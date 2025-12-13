add_library(e_foc.dc.instantiations ${EMIL_EXCLUDE_FROM_ALL} STATIC)
emil_build_for(e_foc.dc.instantiations)

target_include_directories(e_foc.dc.instantiations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_foc.dc.instantiations PUBLIC
    e_foc.hardware
    e_foc.dc.components
    numerical.controllers.implementations
)

target_sources(e_foc.dc.instantiations PRIVATE
    Logic.cpp
    Logic.hpp
)
