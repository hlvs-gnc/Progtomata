file(GLOB SRC_FILES CONFIGURE_DEPENDS
     *.c
)

list(REMOVE_ITEM SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/main.c)

add_library(core STATIC ${SRC_FILES})

target_include_directories(core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/config
    ${PROJECT_SOURCE_DIR}/lib/audio_codec
    ${PROJECT_SOURCE_DIR}/lib/drivers/UART
    ${PROJECT_SOURCE_DIR}/lib/sound_bank
    ${PROJECT_SOURCE_DIR}/lib/trice/src
)

target_link_libraries(core
  PUBLIC
    freertos_kernel
    stm32_framework
    mcu_defs
)