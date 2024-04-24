add_library(tau)

if (${fPIC})
    set_property(TARGET tau PROPERTY POSITION_INDEPENDENT_CODE ON)
endif ()

# Projects that include this project must #include "tau/<header-name>"
target_include_directories(tau PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(
    tau
    PUBLIC
    CONAN_PKG::jive
    CONAN_PKG::fields
    CONAN_PKG::pex
    CONAN_PKG::eigen
    CONAN_PKG::fmt
    CONAN_PKG::nlohmann_json)

target_sources(
    tau
    PRIVATE
    csv.cpp
    line2d.cpp
    pose.cpp
    rotation.cpp
    scale.cpp
    size.cpp
    vector2d.cpp
    wavelet.cpp
    wavelet_compression.cpp)


install(TARGETS tau DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/tau
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
