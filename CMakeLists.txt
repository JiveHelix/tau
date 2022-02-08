cmake_minimum_required(VERSION 3.15)

project(tau CXX)

include(${CMAKE_CURRENT_LIST_DIR}/cmake_includes/setup_project.cmake)
setup_project()

add_subdirectory(tau)

if (ENABLE_TESTING)
    if (${PROJECT_IS_TOP_LEVEL} OR ${RECURSIVE_BUILD_TESTS})
        add_subdirectory(test)
    endif ()
endif ()
