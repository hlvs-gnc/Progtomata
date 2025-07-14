# Grab everything in src/, except main.c which is linked directly
file(GLOB SRC_FILES CONFIGURE_DEPENDS
     *.c
)
list(REMOVE_ITEM SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/main.c)

add_library(core STATIC ${SRC_FILES})
target_link_libraries(core PUBLIC freertos_headers spl)

# Make config/ and include/ headers visible to anything that links 'core'
target_include_directories(core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/config
)
