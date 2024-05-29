add_executable(dxf_points dxf_points.cpp)

target_link_libraries(
    dxf_points
    PRIVATE
    project_warnings
    project_options
    tau)
