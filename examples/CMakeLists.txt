add_executable(dxf_points dxf_points.cpp)

target_link_libraries(
    dxf_points
    PRIVATE
    project_warnings
    project_options
    tau)


add_executable(rotation rotation.cpp)

target_link_libraries(
    rotation
    PRIVATE
    project_warnings
    project_options
    tau)


add_executable(pyr2ypr pyr2ypr.cpp)

target_link_libraries(
    pyr2ypr
    PRIVATE
    project_warnings
    project_options
    tau)
