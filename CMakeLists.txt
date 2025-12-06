cmake_minimum_required(VERSION 3.22)

# Option to build embedded system target (default ON)
option(BUILD_EMBEDDED "Build for embedded ARM target" ON)

# Set toolchain file based on target
if(BUILD_EMBEDDED)
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-arm-none-eabi.cmake)
else()
  # For host system tests, use native toolchain
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-host.cmake)
endif()

project(progtomata LANGUAGES C ASM)

# Project-wide options
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Global include directories
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/config)

# Set compiler flags based on target
if(BUILD_EMBEDDED)
  # Embedded target flags are set in the ARM toolchain file
else()
  # Host system flags
  if(MSVC)
    add_compile_options(/Od /Zi)
  else()
    add_compile_options(-g -O0)
  endif()
endif()

# Sub-directories that build libraries and object code
add_subdirectory(lib)
add_subdirectory(src)

# Executable
if (BUILD_EMBEDDED)
  # Main firmware ELF executable (C and startup ASM)
  add_executable(${PROJECT_NAME}.elf src/main.c lib/framework_stm/cmsis/stm32f4/startup_stm32f40xx.S)

  # Link with all required static libraries
  target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    kernel_freertos
    core
    stm32_framework
    audio_codec
    uart_driver
    trice
    tband)

  # Post-build: generate .bin and .hex files from ELF
  add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMENT "Generating .bin and .hex from ELF")

  # Find STM32CubeProgrammer CLI
  find_program(STM32_PROGRAMMER_CLI 
    NAMES STM32_Programmer_CLI STM32_Programmer_CLI.exe
    PATHS 
      "C:/ST/STM32CubeCLT_1.18.0/STM32CubeProgrammer/bin"
    NO_DEFAULT_PATH)
  
  if(NOT STM32_PROGRAMMER_CLI)
    find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI)
  endif()

  if(STM32_PROGRAMMER_CLI)
    # Upload target: flash firmware to STM32F407 via ST-Link
    add_custom_target(upload
      COMMAND ${STM32_PROGRAMMER_CLI} -c port=SWD -w $<TARGET_FILE:${PROJECT_NAME}.elf> -v -rst
      DEPENDS ${PROJECT_NAME}.elf
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Uploading ${PROJECT_NAME}.elf to STM32F407VG via ST-Link")
  else()
    message(WARNING "STM32CubeProgrammer CLI not found. Upload target will not be available.")
  endif()
endif()

# Test option
if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(test)
endif()