file(GLOB FRAMEWORK_STM_SRC CONFIGURE_DEPENDS *.c)

add_library(stm32_framework STATIC ${FRAMEWORK_STM_SRC})

target_include_directories(stm32_framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(stm32_framework PUBLIC spl)