# cmake/toolchain-arm-none-eabi.cmake
# ------------------------------------------------------------
# Tell CMake we are cross‑compiling bare metal firmware
set(CMAKE_SYSTEM_NAME        Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---- Tool locations --------------------------------------------------------
# One folder up from the gcc‑13.3.1 you showed in your screenshot.
set(TOOLROOT /opt/st/stm32cubeclt_1.18.0/GNU-tools-for-STM32/bin)

set(CMAKE_C_COMPILER   ${TOOLROOT}/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_OBJCOPY      ${TOOLROOT}/arm-none-eabi-objcopy)
set(CMAKE_SIZE         ${TOOLROOT}/arm-none-eabi-size)

# ---- CPU / FPU flags -------------------------------------------------------
# NOTE: **each** flag is its own list item.
set(ARCH_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

# Apply them to every target CMake will create.
add_compile_options(${ARCH_FLAGS} -ffunction-sections -fdata-sections)
add_link_options(${ARCH_FLAGS} -Wl,--gc-sections)