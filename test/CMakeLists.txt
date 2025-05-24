find_package(Nlohmann_json REQUIRED)

add_catch2_test(
    NAME tau_tests
    SOURCES
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
        normalize_tests.cpp
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
        csv_tests.cpp
    LINK
        tau)

add_subdirectory(test_data)
