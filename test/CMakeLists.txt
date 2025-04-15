file(GLOB TEST_SRC
    *.c
)

add_executable(run_tests ${TEST_SRC})
# Link against all libraries
target_link_libraries(run_tests
    core_sources
    audio_codec
    drivers
    sound_bank
)
