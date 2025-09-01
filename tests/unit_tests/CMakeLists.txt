set(UNITTEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB UNITTEST_SRC
  ${UNITTEST_DIR}/test_main.c)

add_executable(UnitTest_main ${UNITTEST_SRC})

# Link unity and the mocks library exposed by upper-level CMakeLists
target_link_libraries(UnitTest_main PRIVATE unity mocks)

set_target_properties(UnitTest_main PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test)

add_test(NAME UnitTest_main COMMAND $<TARGET_FILE:UnitTest_main>)

set_tests_properties(UnitTest_main PROPERTIES
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test)

add_custom_target(check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS UnitTest_main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})