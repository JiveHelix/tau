set(TEST_DATA ${CMAKE_BINARY_DIR}/test_data)
file(MAKE_DIRECTORY ${TEST_DATA})

configure_file(test.csv ${TEST_DATA} COPYONLY)
