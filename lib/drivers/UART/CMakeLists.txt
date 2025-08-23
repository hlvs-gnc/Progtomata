file(GLOB UART_SRC CONFIGURE_DEPENDS *.c)

add_library(uart_driver STATIC ${UART_SRC})

target_include_directories(uart_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(uart_driver PUBLIC spl)
