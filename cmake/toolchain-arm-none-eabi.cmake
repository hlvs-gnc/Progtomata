# cmake/toolchain-arm-none-eabi.cmake
cmake_minimum_required(VERSION 3.22)

# Target side
set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  cortex-m4)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(COMPILER_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER   ${COMPILER_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${COMPILER_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${COMPILER_PREFIX}g++)

if(DEFINED ENV{GNUARMEMB_TOOLCHAIN})
  set(_TOOLROOT $ENV{GNUARMEMB_TOOLCHAIN})
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
  set(_TOOLROOT "C:/ST/STM32CubeCLT_1.18.0/GNU-Tools-for-STM32/bin")
else()
  set(_TOOLROOT "/opt/st/stm32cubeclt_1.18.0/GNU-tools-for-STM32/bin")
endif()

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc HINTS ${_TOOLROOT} REQUIRED)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
foreach(tool objcopy size)
  find_program(ARM_${tool} arm-none-eabi-${tool} HINTS ${_TOOLROOT} REQUIRED)
endforeach()
set(CMAKE_OBJCOPY ${ARM_objcopy})
set(CMAKE_SIZE    ${ARM_size})

set(ARCH_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(CMAKE_EXE_LINKER_FLAGS "${ARCH_FLAGS} -nostartfiles -Wl,--gc-sections -T${CMAKE_SOURCE_DIR}/stm32f407vg.ld")
set(CMAKE_ASM_FLAGS "${CPU_FLAGS} -x assembler-with-cpp")

add_compile_options(${ARCH_FLAGS} -Og -g3 -ffunction-sections -fdata-sections)
add_link_options   (${ARCH_FLAGS} -specs=nosys.specs)