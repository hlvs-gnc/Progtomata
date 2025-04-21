# Collect all source files
file(GLOB SRC_FILES
  *.c
)

add_library(core_sources STATIC ${SRC_FILES})
