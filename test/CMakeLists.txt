set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Collect test sources
file(GLOB TEST_SRC
  ${TEST_DIR}/*.c
  ${TEST_DIR}/mocks/*.c)

add_executable(unit_tests ${TEST_SRC})

target_include_directories(unit_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/config)

target_link_libraries(unit_tests PRIVATE unity core)

set_target_properties(unit_tests PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

enable_testing()
add_test(NAME unit_tests COMMAND unit_tests)

set_tests_properties(unit_tests PROPERTIES
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test)

add_custom_target(check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS unit_tests
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

# Helper to print the path CMake resolved for the test executable
add_custom_target(show_unit_path
  COMMAND ${CMAKE_COMMAND} -E echo "$<TARGET_FILE:unit_tests>"
  DEPENDS unit_tests)
