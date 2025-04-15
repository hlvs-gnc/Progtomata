file(GLOB UART_DRIVER_SRC
    *.c
    *.h
)

add_library(uart_driver STATIC ${UART_DRIVER_SRC})
target_include_directories(uart_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
