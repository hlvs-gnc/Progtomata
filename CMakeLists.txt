cmake_minimum_required(VERSION 3.22)
project(progtomata LANGUAGES C ASM)

# Project-wide options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Global include directories
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/config)

include(CTest)

# Sub-directories that build libraries / object code
add_subdirectory(lib)
add_subdirectory(src)

include_directories(${PROJECT_SOURCE_DIR}/lib/sound_bank)

# Executable
if (BUILD_EMBEDDED)
  add_library(platform_defs INTERFACE)
  target_compile_definitions(platform_defs INTERFACE STM32F407xx STM32F40_41xxx USE_STDPERIPH_DRIVER)
  target_compile_definitions(platform_defs INTERFACE HSE_VALUE=8000000)

  add_executable(${PROJECT_NAME}.elf src/main.c lib/framework_stm/cmsis/stm32f4/startup_stm32f40xx.S)

  target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    core
    stm32_framework
    freertos_kernel
    audio_codec
    uart_driver
    trice)

  add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMENT "Generating .bin and .hex from ELF")
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
