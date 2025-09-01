set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB MOCK_SRC ${TEST_DIR}/mock/*.c)

add_library(mocks STATIC ${MOCK_SRC})

target_include_directories(mocks PUBLIC
  ${TEST_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/config)

enable_testing()

file(GLOB TEST_SRC
  ${TEST_DIR}/test_main.c)

add_executable(UnitTest_main ${TEST_SRC})

target_include_directories(UnitTest_main PRIVATE
  ${TEST_DIR}
  ${PROJECT_ROOT}/include
  ${PROJECT_ROOT}/config)

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