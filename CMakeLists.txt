cmake_minimum_required(VERSION 3.22)
project(progtomata LANGUAGES C ASM)

# Project-wide options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(BUILD_EMBEDDED "Build for STM32 target (ARM cross-compile)" ON)
option(BUILD_HOST_TESTS "Build native host unit tests" ON)

add_library(platform_defs INTERFACE)
if(BUILD_EMBEDDED)
  target_compile_definitions(platform_defs INTERFACE
    STM32F407xx STM32F40_41xxx USE_STDPERIPH_DRIVER HSE_VALUE=8000000)
endif()

if(BUILD_HOST_TESTS)
  target_compile_definitions(platform_defs INTERFACE UNIT_TEST HOST_BUILD)
endif()

# Global include directories
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/config)

include(CTest)
if(BUILD_HOST_TESTS)
  set(BUILD_TESTING ON)
endif()

# Sub-directories that build libraries / object code
add_subdirectory(lib)
add_subdirectory(src)

include_directories(${PROJECT_SOURCE_DIR}/lib/sound_bank)

if(BUILD_EMBEDDED)
  # Executable
  add_executable(${PROJECT_NAME}.elf
    src/main.c
    lib/frameworkstm/cmsis/stm32f4/startup_stm32f40xx.S)

  add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMENT "Generating .bin and .hex from ELF")
endif()

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
  core
  stm32_framework
  freertos_kernel
  audio_codec
  uart_driver
  trice)
  
# Tests live in test/ and will only be added if BUILD_TESTING=ON
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
