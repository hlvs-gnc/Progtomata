cmake_minimum_required(VERSION 3.13)
project(PROGTOMATA C)

# Project-wide options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(mcu_defs INTERFACE)

target_compile_definitions(mcu_defs INTERFACE STM32F40_41xxx USE_STDPERIPH_DRIVER)

# Global include directories
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/config)

# Sub-directories that build libraries / object code
add_subdirectory(lib)
add_subdirectory(src)

include_directories(${PROJECT_SOURCE_DIR}/lib/sound_bank)

# Final executable
add_executable(progtomata src/main.c)

# Link every static lib we produced.
# Add or remove targets here as you create new libraries.
target_link_libraries(progtomata PRIVATE 
                      core
                      stm32_framework
                      freertos_kernel
                      audio_codec
                      uart_driver
                      trice)