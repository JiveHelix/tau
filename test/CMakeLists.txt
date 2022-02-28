add_library(tau_catch_main STATIC catch_main.cpp)
target_link_libraries(tau_catch_main PUBLIC CONAN_PKG::catch2)
target_link_libraries(tau_catch_main PRIVATE project_options)

add_executable(
    tau_tests
    angles_tests.cpp
    power_series_test.cpp
    color_map_test.cpp
    eigen_test.cpp)

target_link_libraries(
    tau_tests
    PRIVATE
    project_warnings
    project_options
    tau_catch_main
    tau)

add_test(NAME TauTests COMMAND ${CMAKE_BINARY_DIR}/bin/tau_tests)
