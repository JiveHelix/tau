add_library(tau INTERFACE)

# Projects that include this project must #include "tau/<header-name>"
target_include_directories(tau INTERFACE ${PROJECT_SOURCE_DIR})

target_link_libraries(tau INTERFACE CONAN_PKG::jive CONAN_PKG::eigen)

install(
    DIRECTORY ${PROJECT_SOURCE_DIR}/tau
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

