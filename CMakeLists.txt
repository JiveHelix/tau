cmake_minimum_required(VERSION 3.15)

project(tau CXX)

include(${CMAKE_CURRENT_LIST_DIR}/cmake_includes/setup_project.cmake)
setup_project()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    if (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS_EQUAL 9.4.0)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fconcepts")
    endif ()
endif ()

add_subdirectory(tau)

if (${PROJECT_IS_TOP_LEVEL})
    option(
        BUILD_EXAMPLES
        "Build the example targets"
        ${ENABLES_DEFAULT})
endif ()

if (${PROJECT_IS_TOP_LEVEL})

    if (ENABLE_TESTING)
        add_subdirectory(test)
    endif ()

    if (${BUILD_EXAMPLES})
        add_subdirectory(examples)
    endif ()

endif ()
