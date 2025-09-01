set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB MOCK_SRC ${TEST_DIR}/mock/*.c)

add_library(mocks STATIC ${MOCK_SRC})

target_include_directories(mocks PUBLIC
  ${TEST_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/config)

enable_testing()

# Add per-group test folders
add_subdirectory(unit_tests)