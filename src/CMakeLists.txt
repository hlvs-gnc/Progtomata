# Gather all C source files in this directory
file(GLOB SRC_FILES CONFIGURE_DEPENDS *.c)

# Except main.c from the list (not part of the core library)
list(REMOVE_ITEM SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/main.c)

# Create a static library target
add_library(core STATIC ${SRC_FILES})

# Specify include directories
target_include_directories(core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/config
    ${PROJECT_SOURCE_DIR}/lib/audio_codec
    ${PROJECT_SOURCE_DIR}/lib/sound_bank
    ${PROJECT_SOURCE_DIR}/lib/trice/src)

# Link the library with FreeRTOS kernel, STM32 framework, and hardware platform definitions
target_link_libraries(core PUBLIC kernel_freertos stm32_framework platform_defs)