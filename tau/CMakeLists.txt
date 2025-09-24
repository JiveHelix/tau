add_library(tau)

if (${fPIC})
    set_property(TARGET tau PROPERTY POSITION_INDEPENDENT_CODE ON)
endif ()

find_package(Fmt REQUIRED)
find_package(Jive REQUIRED)
find_package(Fields REQUIRED)
find_package(Pex REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Nlohmann_json REQUIRED)

# Projects that include this project must #include "tau/<header-name>"
target_include_directories(tau PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(
    tau
    PUBLIC
    jive::jive
    fields::fields
    pex::pex
    Eigen3::Eigen
    fmt::fmt
    nlohmann_json::nlohmann_json)

target_sources(
    tau
    PRIVATE
    camera_parameters.cpp
    color_map.cpp
    color_map_settings.cpp
    csv.cpp
    dxf.cpp
    line2d.cpp
    pixel_origin.cpp
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
