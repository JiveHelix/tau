add_library(tau_catch_main STATIC catch_main.cpp)
target_link_libraries(tau_catch_main PUBLIC CONAN_PKG::catch2)
target_link_libraries(tau_catch_main PRIVATE project_options)

add_executable(
    tau_tests
    angles_tests.cpp
    arithmetic_tests.cpp
    arithmetic_sort.cpp
    bilinear_test.cpp
    color_map_test.cpp
    convolve_tests.cpp
    color_test.cpp
    eigen_test.cpp
    extrinsics_tests.cpp
    intrinsics_tests.cpp
    lens_tests.cpp
    line_tests.cpp
    percentile_tests.cpp
    polynomial_tests.cpp
    projection_tests.cpp
    region_tests.cpp
    rotation_tests.cpp
    row_convolve_tests.cpp
    size_tests.cpp
    variate_tests.cpp
    vector2d_tests.cpp
    vector3d_tests.cpp
    wavelet_tests.cpp
    csv_tests.cpp)

target_link_libraries(
    tau_tests
    PRIVATE
    # project_warnings
    # project_options
    tau_catch_main
    tau)

add_test(NAME TauTests COMMAND ${CMAKE_BINARY_DIR}/bin/tau_tests)

add_subdirectory(test_data)
