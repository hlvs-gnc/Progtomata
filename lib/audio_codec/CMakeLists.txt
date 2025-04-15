file(GLOB AUDIO_CODEC_SRC
    *.c
    *.h
)

add_library(audio_codec STATIC ${AUDIO_CODEC_SRC})
target_include_directories(audio_codec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
