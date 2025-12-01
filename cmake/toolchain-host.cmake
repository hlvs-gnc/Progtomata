# Host system toolchain for running unit tests
# This toolchain uses the native compiler on the host system

# Use the host's native tools
set(CMAKE_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_NAME})
set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_HOST_SYSTEM_PROCESSOR})

# Don't try to cross-compile
set(CMAKE_CROSSCOMPILING FALSE)

# Use native compilers (MSVC, GCC, or Clang depending on what's available)
# CMake will automatically find them

# Compiler flags for host system - will be set per-compiler in CMakeLists.txt
# Don't set them here to avoid issues with different compilers
