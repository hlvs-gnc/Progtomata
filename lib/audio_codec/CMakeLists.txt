file(GLOB AUDIO_CODEC_SRC CONFIGURE_DEPENDS *.c)

add_library(audio_codec STATIC ${AUDIO_CODEC_SRC})

# Headers live alongside the sources
target_include_directories(audio_codec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(audio_codec PUBLIC spl)
