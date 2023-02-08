add_library(tau)

# if (${fPIC})
#     set_property(TARGET tau PROPERTY POSITION_INDEPENDENT_CODE ON)
# endif ()

# Projects that include this project must #include "tau/<header-name>"
target_include_directories(tau PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(
    tau
    PUBLIC
    project_warnings
    project_options
    CONAN_PKG::jive
    CONAN_PKG::fields
    CONAN_PKG::pex
    CONAN_PKG::eigen
    CONAN_PKG::fmt)

target_sources(
    tau
    PRIVATE
    chess.cpp)

install(TARGETS tau DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/tau
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
