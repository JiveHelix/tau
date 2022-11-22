add_library(tau_catch_main STATIC catch_main.cpp)
target_link_libraries(tau_catch_main PUBLIC CONAN_PKG::catch2)
target_link_libraries(tau_catch_main PRIVATE project_options)

add_executable(
    tau_tests
    angles_tests.cpp
    power_series_test.cpp
    color_map_test.cpp
    convolve_tests.cpp
    eigen_test.cpp
    color_test.cpp
    gradient_test.cpp
    harris_tests.cpp
    arithmetic_tests.cpp
    bilinear_test.cpp
    point_tests.cpp
    size_tests.cpp
    region_tests.cpp
    view_tests.cpp)

target_link_libraries(
    tau_tests
    PRIVATE
    project_warnings
    project_options
    tau_catch_main
    tau)

add_test(NAME TauTests COMMAND ${CMAKE_BINARY_DIR}/bin/tau_tests)
