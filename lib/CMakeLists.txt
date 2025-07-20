# FreeRTOS Kernel target
set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/freertos)

set(FREERTOS_PORT_DIR ${FREERTOS_DIR}/portable/GCC/ARM_CM4F)

set(FREERTOS_SRC
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/croutine.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_PORT_DIR}/port.c)

add_library(freertos_kernel STATIC ${FREERTOS_SRC})

target_include_directories(freertos_kernel PUBLIC
    ${FREERTOS_DIR}/include
    ${FREERTOS_PORT_DIR})

target_link_libraries(freertos_kernel PUBLIC mcu_defs)

# SPL configuration
set(SPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/frameworkstm/spl/stm32f4)
file(GLOB SPL_SRC
    ${SPL_DIR}/src/*.c)

# CMSIS configuration
set(CMSIS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/frameworkstm/cmsis)
file(GLOB CMSIS_SRC
    ${CMSIS_DIR}/stm32f4/*.c)

include_directories(${SPL_DIR}/inc
                    ${CMSIS_DIR}/stm32f4
                    ${CMSIS_DIR}/cores/stm32)

add_library(stm32_framework STATIC ${CMSIS_SRC} ${SPL_SRC})

target_include_directories(stm32_framework PUBLIC
    ${SPL_DIR}/inc
    ${CMSIS_DIR}/stm32f4
    ${CMSIS_DIR}/cores/stm32)

target_link_libraries(stm32_framework PUBLIC mcu_defs)

# Audio Codec configuration
set(AUDIO_CODEC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/audio_codec)
file(GLOB AUDIO_CODEC_SRC
    ${AUDIO_CODEC_DIR}/*.c)

include_directories(${AUDIO_CODEC_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/trice/src)

add_library(audio_codec STATIC ${AUDIO_CODEC_SRC})

target_include_directories(audio_codec PUBLIC ${AUDIO_CODEC_DIR})

target_link_libraries(audio_codec PUBLIC stm32_framework freertos_kernel mcu_defs)

# UART Driver configuration
set(UART_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drivers/UART)
file(GLOB UART_SRC
    ${UART_DIR}/*.c)

include_directories(${UART_DIR})

add_library(uart_driver STATIC ${UART_SRC})

target_include_directories(uart_driver PUBLIC ${UART_DIR})

target_link_libraries(uart_driver PUBLIC stm32_framework freertos_kernel mcu_defs)

# Trice configuration
set(TRICE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/trice/src)
file(GLOB TRICE_SRC
    ${TRICE_DIR}/*.c)

include_directories(${TRICE_DIR})

add_library(trice STATIC ${TRICE_SRC})

target_include_directories(trice PUBLIC ${TRICE_DIR})

target_link_libraries(trice PUBLIC core)
