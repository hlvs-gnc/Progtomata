set(SPL_DIR "C:/Users/henri/.platformio/packages/framework-spl/stm32/spl")
set(SPL_CMSIS_DIR "C:/Users/henri/.platformio/packages/framework-spl/stm32/cmsis")

file(GLOB SPL_SRC
    ${SPL_DIR}/variants/stm32f4/src/*.c
    ${SPL_CMSIS_DIR}/variants/stm32f4/*.c
)

add_library(spl STATIC ${SPL_SRC})
target_include_directories(spl PUBLIC
    ${SPL_DIR}/variants/stm32f4/inc
    ${SPL_CMSIS_DIR}/variants/stm32f4
)

# ---- FreeRTOS location ------------------------------------------------------
set(FREERTOS_DIR
    "C:/Users/henri/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.1/Middlewares/Third_Party/FreeRTOS"
)

include_directories(
    ${FREERTOS_DIR}/Source/include
    ${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F
)

# Public headers for every architecture
set(FREERTOS_INC_DIR     ${FREERTOS_DIR}/Source/include)

# Port-specific headers (Cortex-M4F + GCC in your STM32F4Cube package)
set(FREERTOS_PORT_INC    ${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F)

# Header-only interface target
add_library(freertos_headers INTERFACE)
target_include_directories(freertos_headers INTERFACE
    ${FREERTOS_INC_DIR}
    ${FREERTOS_PORT_INC}

    ${SPL_CMSIS_DIR}/variants/stm32f4
    ${SPL_CMSIS_DIR}/cores/stm32
)

set(FREERTOS_SRC
  ${FREERTOS_DIR}/Source/list.c
  ${FREERTOS_DIR}/Source/queue.c
  ${FREERTOS_DIR}/Source/tasks.c
  ${FREERTOS_DIR}/Source/timers.c
  ${FREERTOS_DIR}/Source/event_groups.c

  # Memory manager you prefer (heap_1-5.c) – pick ONE:
  ${FREERTOS_DIR}/Source/portable/MemMang/heap_4.c

  # Cortex-M4F / GCC port layer:
  ${FREERTOS_DIR}/Source/portable/GCC/ARM_CM4F/port.c
)

add_library(freertos_kernel STATIC ${FREERTOS_SRC})
target_link_libraries(freertos_kernel PUBLIC spl)

# Each sub-folder is responsible for its own CMakeLists.txt
add_subdirectory(audio_codec)
add_subdirectory(drivers/UART)

file(GLOB_RECURSE TRICE_SRC trice/src/*.c)
add_library(trice STATIC ${TRICE_SRC})
target_include_directories(trice PUBLIC trice/src trice)
target_link_libraries(trice PUBLIC spl)

# If you have .c sitting directly in lib/, collect them into a helper lib
file(GLOB LIB_ROOT_SRC CONFIGURE_DEPENDS
     *.c
)

if(LIB_ROOT_SRC)
    add_library(lib STATIC ${LIB_ROOT_SRC})
    target_include_directories(lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
endif()
