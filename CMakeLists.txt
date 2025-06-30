cmake_minimum_required(VERSION 3.13)
project(PROGTOMATA C)

# Set C standard
set(CMAKE_C_STANDARD 11)

# Add include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/config
)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(lib)
# add_subdirectory(test)
