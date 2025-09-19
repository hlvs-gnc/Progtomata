# Set the test directory variable to the current source
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Gather all mock source files in the mock definitions
# And create a respective static library target
file(GLOB MOCK_SRC ${TEST_DIR}/mock/*.c)

add_library(mocks STATIC ${MOCK_SRC})

# Specify include directories for the mocks library
target_include_directories(mocks PUBLIC
  ${TEST_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/config)

# Enable CTest framework
enable_testing()

# Gather all test source files (currently only test_main.c)
file(GLOB TEST_SRC
  ${TEST_DIR}/test_main.c)

# Create the main unit test executable
add_executable(UnitTest_main ${TEST_SRC})

# Specify include directories for the unit test executable
target_include_directories(UnitTest_main PRIVATE
  ${TEST_DIR}
  ${PROJECT_ROOT}/include
  ${PROJECT_ROOT}/config)

target_link_libraries(UnitTest_main PRIVATE unity mocks)

set_target_properties(UnitTest_main PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

add_test(NAME UnitTest_main COMMAND $<TARGET_FILE:UnitTest_main>)

# Set the working directory for the test to the test output
set_tests_properties(UnitTest_main PROPERTIES
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test)

# Add a custom target 'check' to run all tests with output on failure
add_custom_target(check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS UnitTest_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})