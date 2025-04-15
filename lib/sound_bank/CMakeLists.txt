file(GLOB SOUND_BANK_SRC
    *.c
    *.h
)

add_library(sound_bank STATIC ${SOUND_BANK_SRC})
target_include_directories(sound_bank PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
